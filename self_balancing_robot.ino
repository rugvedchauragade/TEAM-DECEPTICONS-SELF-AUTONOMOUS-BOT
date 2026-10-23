/*
 ============================================================================
  SELF-BALANCING PICK & DROP ROBOT  -  ESP32 + MPU6050 + TB6612FNG
 ============================================================================
  Wi-Fi (the robot creates its own hotspot):
      Name     : DAVIDPUTRA
      Password : 12345678
      Open     : http://192.168.4.1   (in your phone browser)

  NO EXTRA LIBRARIES NEEDED. Works with ESP32 Arduino core 2.x and 3.x.
  Board: "ESP32 Dev Module"

  ---------------------------- WIRING ---------------------------------------
  MPU6050 (GY-521)   VCC -> 3V3 | GND -> GND | SDA -> GPIO21 | SCL -> GPIO22
      Mount FLAT, X-arrow pointing to the FRONT of the robot, near wheel axle.

  TB6612FNG          VM   -> battery + (after the ON/OFF switch, 7.4 V)
                     VCC  -> 3V3 | GND -> GND (common ground with everything)
                     STBY -> GPIO13
                     AIN1 -> GPIO26 | AIN2 -> GPIO25 | PWMA -> GPIO27   (left motor)
                     BIN1 -> GPIO33 | BIN2 -> GPIO32 | PWMB -> GPIO4    (right motor)
                     A01/A02 -> left N20 | B01/B02 -> right N20

  Servos (MG90S)     Lift/arm servo signal -> GPIO18
                     Gripper servo signal  -> GPIO19
                     Servo red wires -> 5 V from the buck converter (NOT from ESP32 pin)
                     Servo brown wires -> GND (common)
                     Put a 470-1000 uF capacitor across the 5 V rail.

  Power              2x Li-ion (series) -> switch -> buck converter set to 5.0 V
                     Buck 5 V -> ESP32 VIN/5V pin + servos.
                     Same battery (before buck) -> TB6612 VM.

  ---------------------------- FIRST-TIME TEST ------------------------------
  1. Upload, open Serial Monitor (115200) to see status.
  2. Hold robot upright and still at power-on (gyro calibration ~2 s).
  3. Open the dashboard. Tilt the robot FORWARD by hand: angle must read
     POSITIVE. If it reads negative, change ANGLE_SIGN to -1.
  4. Hold it upright: it starts balancing after ~0.5 s of being upright.
     Tilt forward: wheels must spin FORWARD (to catch the fall).
     If they spin backward, flip BOTH MOTOR_L_INVERT and MOTOR_R_INVERT.
  5. If only one wheel is wrong, flip only that motor's invert flag.
  6. Tune Kp, Kd (and Ki) in the dashboard "Tuning" panel - see notes below.

  ---------------------------- TUNING GUIDE ---------------------------------
  Every robot is different (weight, wheel size, motors), so the gains below
  are a starting point. Use the dashboard Tuning panel (no re-upload needed):
    a) Set Ki = 0. Raise Kp until the robot tries to stand but oscillates fast.
    b) Raise Kd until the oscillation calms down (too much Kd = buzzing/vibrating).
    c) Lower Kp slightly if it jitters. Press "Apply & save" to store in flash.
    d) The "Balance point" (trim) adjusts itself while standing still, to
       cancel uneven weight (arm, battery). You normally don't touch it.
    e) Too much drive/aggressive? Lower "Turn power" and "Lean at 100% speed".
  If wheels are 6 V N20 motors on a 7.4-8.4 V battery, keep MAX_PWM near 200.
 ============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>

// ============================ USER SETTINGS ================================
const char* AP_SSID = "DAVIDPUTRA";
const char* AP_PASS = "12345678";

// ---- Pins
#define PIN_SDA   21
#define PIN_SCL   22
#define PIN_STBY  13
#define PIN_AIN1  26
#define PIN_AIN2  25
#define PIN_PWMA  27
#define PIN_BIN1  33
#define PIN_BIN2  32
#define PIN_PWMB  4
#define PIN_SERVO_LIFT 18
#define PIN_SERVO_GRIP 19

// ---- Orientation / direction (see FIRST-TIME TEST above)
#define ANGLE_SIGN       1      // set to -1 if leaning forward reads negative
#define MOTOR_L_INVERT   false  // flip if left wheel turns the wrong way
#define MOTOR_R_INVERT   false  // flip if right wheel turns the wrong way

// ---- Motor tuning
#define MAX_PWM   255           // use ~200 for 6 V motors on a 2S battery
#define MIN_PWM   22            // motor dead-band compensation (raise if motors hum but don't turn)

// ---- Safety
#define FALL_DEG   35.0f        // beyond this angle motors shut off
#define START_DEG   4.0f        // must be within this to (re)start balancing
#define CMD_TIMEOUT_MS 500      // no command from phone for this long -> stop moving

// ---- Filter
#define ALPHA      0.985f       // complementary filter (gyro weight)

// ---- Default PID (can be changed live from the dashboard and saved)
float Kp      = 22.0f;
float Ki      = 0.0f;
float Kd      = 1.2f;
float trimDeg = 0.0f;           // balance point (auto-adjusts while standing still)
float leanMax = 4.0f;           // forward lean (deg) at 100% joystick & 100% speed
float turnMax = 70.0f;          // differential PWM at 100% turn & 100% speed
const float TRIM_RATE = 0.02f;  // deg per (PWM*second): how fast balance point self-corrects
const float TRIM_LIMIT = 10.0f;

// ---- Arm / gripper angles (calibrate for YOUR mechanism, 0-180)
#define ARM_UP       130
#define ARM_MID       85
#define ARM_DOWN      40
#define GRIP_OPEN    120
#define GRIP_CLOSED   55
#define ARM_STEP_MS   15        // 1 degree every N ms (bigger = slower = gentler on balance)
// ===========================================================================

// ------------------------- PWM helpers (core 2.x / 3.x) -------------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch, freq, res) ledcAttach((pin), (freq), (res))
  #define PWM_WRITE(pin, ch, duty)       ledcWrite((pin), (duty))
#else
  #define PWM_ATTACH(pin, ch, freq, res) do { ledcSetup((ch), (freq), (res)); ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty)       ledcWrite((ch), (duty))
#endif

#define CH_MOTOR_L 8
#define CH_MOTOR_R 9
#define CH_SERVO_L 10
#define CH_SERVO_G 11

#define MPU_ADDR 0x68
#define RAD2DEG  57.29578f
#define LOOP_MS  5                 // 200 Hz control loop

enum { ST_CALIB = 0, ST_WAIT = 1, ST_BAL = 2, ST_FALLEN = 3, ST_OFF = 4, ST_IMU_ERR = 5 };

// --------------------------- shared state ---------------------------------
volatile float    g_angle = 0, g_out = 0;
volatile uint8_t  g_state = ST_CALIB;
volatile int      cmdX = 0, cmdY = 0;         // -100..100 (x: +right, y: +forward)
volatile uint32_t lastCmdMs = 0;
volatile int      speedPct = 60;              // 10..100
volatile bool     balanceOn = true;

WebServer   server(80);
Preferences prefs;

// ============================== MOTORS =====================================
bool motorsEnabled = false;

void motorsEnable(bool en) {
  if (en == motorsEnabled) return;
  motorsEnabled = en;
  if (!en) {
    PWM_WRITE(PIN_PWMA, CH_MOTOR_L, 0);
    PWM_WRITE(PIN_PWMB, CH_MOTOR_R, 0);
    digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_STBY, LOW);
  } else {
    digitalWrite(PIN_STBY, HIGH);
  }
}

// v: -255..255 (+ = wheel rolls forward)
void driveMotor(bool left, float v) {
  bool fwd = (v >= 0);
  if (left ? MOTOR_L_INVERT : MOTOR_R_INVERT) fwd = !fwd;
  float a = fabsf(v);
  if (a > 255) a = 255;
  int pwm = 0;
  if (a >= 1.0f) pwm = (int)(MIN_PWM + a * (MAX_PWM - MIN_PWM) / 255.0f);   // dead-band compensation
  if (left) {
    digitalWrite(PIN_AIN1, fwd); digitalWrite(PIN_AIN2, !fwd);
    PWM_WRITE(PIN_PWMA, CH_MOTOR_L, pwm);
  } else {
    digitalWrite(PIN_BIN1, fwd); digitalWrite(PIN_BIN2, !fwd);
    PWM_WRITE(PIN_PWMB, CH_MOTOR_R, pwm);
  }
}

// ============================== MPU6050 ====================================
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuInit() {
  if (!mpuWrite(0x6B, 0x00)) return false;   // wake up
  delay(50);
  mpuWrite(0x6B, 0x01);                      // clock = gyro X PLL (more stable)
  mpuWrite(0x1A, 0x03);                      // digital low-pass ~44 Hz
  mpuWrite(0x1B, 0x08);                      // gyro  +-500 deg/s  (65.5 LSB per deg/s)
  mpuWrite(0x1C, 0x00);                      // accel +-2 g        (16384 LSB per g)
  mpuWrite(0x19, 0x04);                      // sample rate 200 Hz
  return true;
}

static int16_t rd16() { uint8_t h = Wire.read(); uint8_t l = Wire.read(); return (int16_t)((h << 8) | l); }

bool mpuRead(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)14) != 14) return false;
  ax = rd16(); ay = rd16(); az = rd16();
  rd16();                                    // temperature (ignored)
  gx = rd16(); gy = rd16(); gz = rd16();
  return true;
}

// ========================= BALANCE CONTROL TASK ============================
void balanceTask(void*) {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);

  while (!mpuInit()) {
    g_state = ST_IMU_ERR;
    Serial.println("MPU6050 not found - check SDA/SCL/power");
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  // ---- calibrate gyro bias + starting angle (robot must be still)
  g_state = ST_CALIB;
  vTaskDelay(pdMS_TO_TICKS(500));
  float gSum = 0, aSum = 0; int n = 0;
  for (int i = 0; i < 400; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuRead(ax, ay, az, gx, gy, gz)) {
      gSum += gy;
      aSum += ANGLE_SIGN * atan2f(-(float)ax, (float)az) * RAD2DEG;
      n++;
    }
    vTaskDelay(pdMS_TO_TICKS(3));
  }
  const float gyroBias = n ? gSum / n : 0;
  float angle = n ? aSum / n : 0;
  Serial.printf("Gyro calibrated (bias %.1f). Start angle %.1f\n", gyroBias, angle);

  float integral = 0, leanNow = 0, turnNow = 0;
  int   holdCnt = 0, failCnt = 0;
  uint32_t prevUs = micros();
  g_state = ST_WAIT;
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));
    uint32_t nowUs = micros();
    float dt = (nowUs - prevUs) * 1e-6f;
    prevUs = nowUs;
    if (dt <= 0 || dt > 0.05f) dt = LOOP_MS * 0.001f;

    // ---- read IMU
    int16_t ax, ay, az, gx, gy, gz;
    if (!mpuRead(ax, ay, az, gx, gy, gz)) {
      if (++failCnt > 5) { motorsEnable(false); g_state = ST_IMU_ERR; }
      continue;
    }
    if (g_state == ST_IMU_ERR) g_state = ST_WAIT;
    failCnt = 0;

    // ---- angle estimate (complementary filter). + = leaning forward
    float gyroDps  = ANGLE_SIGN * (gy - gyroBias) / 65.5f;
    float accAngle = ANGLE_SIGN * atan2f(-(float)ax, (float)az) * RAD2DEG;
    angle = ALPHA * (angle + gyroDps * dt) + (1.0f - ALPHA) * accAngle;
    g_angle = angle;

    // ---- user kill-switch
    if (!balanceOn) {
      motorsEnable(false);
      g_state = ST_OFF; g_out = 0; holdCnt = 0;
      continue;
    }
    if (g_state == ST_OFF) g_state = ST_WAIT;

    // ---- waiting / fallen: motors off until held upright
    if (g_state == ST_WAIT || g_state == ST_FALLEN) {
      motorsEnable(false);
      g_out = 0;
      if (fabsf(angle) < START_DEG) holdCnt++; else holdCnt = 0;
      if (holdCnt > 100) {                    // upright for 0.5 s -> go
        integral = 0; leanNow = 0; turnNow = 0;
        motorsEnable(true);
        g_state = ST_BAL;
      }
      continue;
    }

    // ---- balancing
    if (fabsf(angle) > FALL_DEG) {
      motorsEnable(false);
      g_state = ST_FALLEN; holdCnt = 0; g_out = 0;
      continue;
    }

    // phone commands (with failsafe)
    bool fresh = (millis() - lastCmdMs) < CMD_TIMEOUT_MS;
    int cx = fresh ? cmdX : 0;
    int cy = fresh ? cmdY : 0;
    float sp = speedPct / 100.0f;
    float leanTarget = (cy / 100.0f) * leanMax * sp;
    float turnTarget = (cx / 100.0f) * turnMax * sp;

    // smooth ramps
    float dl = leanTarget - leanNow, ml = 12.0f * dt;     // 12 deg/s
    leanNow += constrain(dl, -ml, ml);
    float dtn = turnTarget - turnNow, mt = 600.0f * dt;   // 600 PWM/s
    turnNow += constrain(dtn, -mt, mt);

    // PID (D uses the gyro directly = clean, noise-free derivative)
    float setpoint = trimDeg + leanNow;
    float err = angle - setpoint;
    integral += Ki * err * dt;
    integral = constrain(integral, -100.0f, 100.0f);
    float out = Kp * err + integral + Kd * gyroDps;
    out = constrain(out, -255.0f, 255.0f);
    g_out = out;

    // auto balance-point: when not driving, slowly cancel any constant push
    if (cy == 0 && cx == 0 && fabsf(leanNow) < 0.2f) {
      trimDeg += TRIM_RATE * out * dt;
      trimDeg = constrain(trimDeg, -TRIM_LIMIT, TRIM_LIMIT);
    }

    driveMotor(true,  out + turnNow);
    driveMotor(false, out - turnNow);
  }
}

// ============================ ARM / GRIPPER ================================
int liftCur = ARM_UP, liftTgt = ARM_UP;
int gripCur = GRIP_OPEN, gripTgt = GRIP_OPEN;

void servoWrite(int ch, int pin, int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t us   = 500 + (uint32_t)angle * (2400 - 500) / 180;   // MG90S range
  uint32_t duty = us * 16383UL / 20000UL;                       // 14-bit @ 50 Hz
  PWM_WRITE(pin, ch, duty);
}

struct Step { int lift; int grip; uint16_t hold; };   // -1 = leave unchanged

const Step SEQ_PICK[] = {
  { ARM_UP,   GRIP_OPEN,   200 },
  { ARM_DOWN, GRIP_OPEN,   300 },
  { -1,       GRIP_CLOSED, 500 },     // grab
  { ARM_UP,   -1,          200 },
};
const Step SEQ_DROP[] = {
  { ARM_MID,  -1,          300 },
  { -1,       GRIP_OPEN,   500 },     // release
  { ARM_UP,   -1,          200 },
};
const Step SEQ_HOME[] = {
  { ARM_UP,   GRIP_OPEN,   100 },
};

const Step* seqPtr = nullptr;
uint8_t  seqLen = 0, seqIdx = 0;
bool     seqRunning = false, stepApplied = false;
uint32_t holdUntil = 0, lastServoMs = 0;

void startSeq(const Step* s, uint8_t len) {
  seqPtr = s; seqLen = len; seqIdx = 0;
  seqRunning = true; stepApplied = false; holdUntil = 0;
}

void servoUpdate() {
  uint32_t now = millis();
  if (now - lastServoMs < ARM_STEP_MS) return;
  lastServoMs = now;

  if (liftCur != liftTgt) { liftCur += (liftTgt > liftCur) ? 1 : -1; servoWrite(CH_SERVO_L, PIN_SERVO_LIFT, liftCur); }
  if (gripCur != gripTgt) { gripCur += (gripTgt > gripCur) ? 1 : -1; servoWrite(CH_SERVO_G, PIN_SERVO_GRIP, gripCur); }

  if (!seqRunning) return;
  const Step &st = seqPtr[seqIdx];
  if (!stepApplied) {
    if (st.lift >= 0) liftTgt = st.lift;
    if (st.grip >= 0) gripTgt = st.grip;
    stepApplied = true; holdUntil = 0;
  } else if (liftCur == liftTgt && gripCur == gripTgt) {
    if (holdUntil == 0) holdUntil = (now + st.hold) | 1;
    else if ((int32_t)(now - holdUntil) >= 0) {
      seqIdx++; stepApplied = false;
      if (seqIdx >= seqLen) seqRunning = false;
    }
  }
}

// ============================ SETTINGS STORAGE =============================
void loadTune() {
  prefs.begin("balbot", true);
  Kp = prefs.getFloat("kp", Kp);       Ki = prefs.getFloat("ki", Ki);
  Kd = prefs.getFloat("kd", Kd);       trimDeg = prefs.getFloat("trim", trimDeg);
  leanMax = prefs.getFloat("lean", leanMax); turnMax = prefs.getFloat("turn", turnMax);
  prefs.end();
}
void saveTune() {
  prefs.begin("balbot", false);
  prefs.putFloat("kp", Kp);   prefs.putFloat("ki", Ki);   prefs.putFloat("kd", Kd);
  prefs.putFloat("trim", trimDeg);
  prefs.putFloat("lean", leanMax); prefs.putFloat("turn", turnMax);
  prefs.end();
}

// ============================ WEB DASHBOARD ================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>DAVIDPUTRA Bot</title>
<style>
:root{--bg:#dde4ea;--panel:#f5f7f9;--ink:#16222d;--mute:#5a6876;--line:#c2ccd5;--go:#1f4fe0;--stop:#d92d27;--ok:#17834d;--warn:#b86e00}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;background:var(--bg);color:var(--ink);font-family:"Avenir Next","Segoe UI",Roboto,system-ui,sans-serif}
body{max-width:520px;margin:0 auto;padding:14px 14px 40px;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
header{display:flex;align-items:center;justify-content:space-between;margin:4px 2px 12px}
h1{font-size:22px;margin:0;font-weight:800;letter-spacing:.2px}
h1 small{display:block;font-size:12.5px;font-weight:500;color:var(--mute);letter-spacing:0}
.chip{font-size:13px;font-weight:700;padding:7px 12px;border-radius:999px;background:#fff;border:1.5px solid var(--line);color:var(--mute);white-space:nowrap}
.chip.ok{color:var(--ok);border-color:var(--ok)} .chip.warn{color:var(--warn);border-color:var(--warn)} .chip.bad{color:var(--stop);border-color:var(--stop)}
.card{background:var(--panel);border:1.5px solid var(--line);border-radius:20px;padding:16px;margin-bottom:12px}
h2{font-size:15px;margin:0 0 12px;font-weight:800}
.tilt{display:flex;align-items:center;gap:14px}
.tilt svg{width:120px;height:140px;flex:none;background:#e8edf1;border-radius:14px}
#bot{transform-origin:60px 110px;transform-box:view-box;transition:transform .12s linear}
.ang{font-size:46px;font-weight:800;font-variant-numeric:tabular-nums;line-height:1}
.ang span{font-size:20px;color:var(--mute);font-weight:600}
.msg{font-size:14px;color:var(--mute);margin:6px 0 10px;min-height:20px}
.btn{font:inherit;font-weight:700;font-size:15px;border:1.5px solid var(--ink);background:#fff;color:var(--ink);border-radius:14px;padding:12px 14px;touch-action:manipulation}
.btn:active{background:var(--ink);color:#fff}
.btn.pri{background:var(--go);border-color:var(--go);color:#fff}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:330px;margin:0 auto}
.dp{aspect-ratio:1/1;border-radius:22px;border:2px solid var(--ink);background:#fff;font-size:34px;color:var(--ink);touch-action:none;font-family:inherit;padding:0}
.dp.on{background:var(--go);border-color:var(--go);color:#fff;transform:scale(.96)}
.dp.stop{background:var(--stop);border-color:var(--stop);color:#fff;font-size:17px;font-weight:800;border-radius:50%}
.dp.stop:active{background:#a81d19}
.row{display:flex;align-items:center;gap:12px;margin-top:14px}
.row label{font-size:14px;font-weight:700;min-width:74px}
.row output{font-variant-numeric:tabular-nums;font-weight:700;min-width:44px;text-align:right}
input[type=range]{flex:1;accent-color:var(--go);height:32px}
.btns{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:14px}
details summary{font-weight:800;font-size:15px;cursor:pointer;list-style:none}
details summary::after{content:"+";float:right;color:var(--mute)} details[open] summary::after{content:"\2212"}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
.grid2 label{font-size:13px;font-weight:700;color:var(--mute);display:block}
.grid2 input{width:100%;font:inherit;font-size:16px;font-weight:700;padding:9px 10px;border:1.5px solid var(--line);border-radius:12px;background:#fff;color:var(--ink);-webkit-user-select:text;user-select:text}
.note{font-size:12.5px;color:var(--mute);margin-top:10px;line-height:1.4}
</style></head><body>
<header>
  <h1>DAVIDPUTRA<small>Self-balancing pick &amp; drop bot</small></h1>
  <div id="conn" class="chip bad">Offline</div>
</header>

<section class="card">
  <div class="tilt">
    <svg viewBox="0 0 120 140" aria-label="Robot tilt">
      <line x1="4" y1="126" x2="116" y2="126" stroke="#9aa7b3" stroke-width="3"/>
      <g id="bot">
        <rect x="42" y="42" width="36" height="62" rx="9" fill="#16222d"/>
        <rect x="48" y="50" width="24" height="10" rx="3" fill="#1f4fe0"/>
        <polyline points="78,56 94,40 94,26" fill="none" stroke="#16222d" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/>
        <path d="M88,24 L94,16 L100,24" fill="none" stroke="#1f4fe0" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>
        <circle cx="60" cy="110" r="16" fill="#f5f7f9" stroke="#16222d" stroke-width="5"/>
        <circle cx="60" cy="110" r="3" fill="#16222d"/>
      </g>
    </svg>
    <div style="flex:1">
      <div class="ang"><b id="angle">0.0</b><span>&deg;</span></div>
      <div class="msg" id="msg">Connecting&hellip;</div>
      <button class="btn" id="balBtn" style="width:100%">Balance: ON</button>
    </div>
  </div>
</section>

<section class="card">
  <h2>Drive (press and hold)</h2>
  <div class="pad">
    <span></span><button class="dp" data-x="0" data-y="100" aria-label="Forward">&#9650;</button><span></span>
    <button class="dp" data-x="-100" data-y="0" aria-label="Left">&#9664;</button>
    <button class="dp stop" id="stopBtn" aria-label="Stop">STOP</button>
    <button class="dp" data-x="100" data-y="0" aria-label="Right">&#9654;</button>
    <span></span><button class="dp" data-x="0" data-y="-100" aria-label="Backward">&#9660;</button><span></span>
  </div>
  <div class="row"><label for="spd">Speed</label><input id="spd" type="range" min="10" max="100" value="60"><output id="spdV">60%</output></div>
  <div class="note">Letting go of a button stops the robot. If the phone loses Wi-Fi, the robot stops moving by itself and keeps balancing.</div>
</section>

<section class="card">
  <h2>Arm &amp; gripper</h2>
  <div class="row" style="margin-top:0"><label for="lift">Arm</label><input id="lift" type="range" min="0" max="180" value="130"><output id="liftV">130</output></div>
  <div class="row"><label for="grip">Gripper</label><input id="grip" type="range" min="0" max="180" value="120"><output id="gripV">120</output></div>
  <div class="btns">
    <button class="btn pri" data-seq="pick">Pick</button>
    <button class="btn pri" data-seq="drop">Drop</button>
    <button class="btn" data-seq="home">Home</button>
    <button class="btn" data-seq="open">Open</button>
    <button class="btn" data-seq="close">Close</button>
    <span></span>
  </div>
</section>

<section class="card">
  <details id="tune">
    <summary>Tuning</summary>
    <div class="grid2">
      <div><label for="kp">Kp (stiffness)</label><input id="kp" type="number" step="0.5" inputmode="decimal"></div>
      <div><label for="kd">Kd (damping)</label><input id="kd" type="number" step="0.05" inputmode="decimal"></div>
      <div><label for="ki">Ki (drift fix)</label><input id="ki" type="number" step="1" inputmode="decimal"></div>
      <div><label for="trim">Balance point (&deg;)</label><input id="trim" type="number" step="0.1" inputmode="decimal"></div>
      <div><label for="lean">Lean at 100% (&deg;)</label><input id="lean" type="number" step="0.5" inputmode="decimal"></div>
      <div><label for="turn">Turn power</label><input id="turn" type="number" step="5" inputmode="decimal"></div>
    </div>
    <div class="btns" style="grid-template-columns:1fr 1fr">
      <button class="btn" id="apply">Apply</button>
      <button class="btn pri" id="save">Apply &amp; save</button>
    </div>
    <div class="note" id="tuneMsg">Raise Kp until it stands but shakes, then raise Kd to calm it. Save when it feels good. Balance point adjusts itself while standing still.</div>
  </details>
</section>

<script>
const $=id=>document.getElementById(id);
async function api(p,t){t=t||900;const c=new AbortController();const id=setTimeout(()=>c.abort(),t);
  try{const r=await fetch(p,{signal:c.signal,cache:'no-store'});return await r.json();}catch(e){return null}finally{clearTimeout(id)}}

/* ---------- drive ---------- */
let dx=0,dy=0,inflight=false;
async function pushMove(){if(inflight)return;inflight=true;await api('/api/move?x='+dx+'&y='+dy,500);inflight=false}
setInterval(()=>{if(dx||dy)pushMove()},100);
function hold(x,y){dx=x;dy=y;pushMove()}
function release(){dx=0;dy=0;api('/api/stop',800)}
document.querySelectorAll('.dp[data-x]').forEach(b=>{
  const x=+b.dataset.x,y=+b.dataset.y;
  b.addEventListener('pointerdown',e=>{e.preventDefault();try{b.setPointerCapture(e.pointerId)}catch(_){}b.classList.add('on');hold(x,y)});
  ['pointerup','pointercancel'].forEach(ev=>b.addEventListener(ev,()=>{b.classList.remove('on');release()}));
  b.addEventListener('contextmenu',e=>e.preventDefault());
});
$('stopBtn').addEventListener('click',release);
window.addEventListener('blur',release);
document.addEventListener('visibilitychange',()=>{if(document.hidden)release()});
const keys={ArrowUp:[0,100],w:[0,100],ArrowDown:[0,-100],s:[0,-100],ArrowLeft:[-100,0],a:[-100,0],ArrowRight:[100,0],d:[100,0]};
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT'&&e.target.type==='number')return;
  if(e.key===' '){release();return}const k=keys[e.key];if(k&&!e.repeat)hold(k[0],k[1])});
addEventListener('keyup',e=>{if(keys[e.key])release()});

/* ---------- speed ---------- */
$('spd').addEventListener('input',e=>{$('spdV').textContent=e.target.value+'%'});
$('spd').addEventListener('change',e=>api('/api/speed?v='+e.target.value));

/* ---------- arm ---------- */
let armT=0;
function armSend(){const now=Date.now();if(now-armT<90)return;armT=now;api('/api/arm?lift='+$('lift').value+'&grip='+$('grip').value)}
['lift','grip'].forEach(id=>{
  $(id).addEventListener('input',e=>{$(id+'V').textContent=e.target.value;armSend()});
  $(id).addEventListener('change',()=>{armT=0;armSend()});
});
document.querySelectorAll('[data-seq]').forEach(b=>b.addEventListener('click',()=>api('/api/seq?name='+b.dataset.seq)));

/* ---------- balance switch ---------- */
let balOn=true;
$('balBtn').addEventListener('click',()=>{api('/api/balance?on='+(balOn?0:1))});

/* ---------- tuning ---------- */
function tuneQuery(save){return '/api/tune?kp='+$('kp').value+'&kd='+$('kd').value+'&ki='+$('ki').value+'&trim='+$('trim').value+'&lean='+$('lean').value+'&turn='+$('turn').value+(save?'&save=1':'')}
$('apply').addEventListener('click',async()=>{const r=await api(tuneQuery(false));$('tuneMsg').textContent=r?'Applied.':'Could not reach the robot.'});
$('save').addEventListener('click',async()=>{const r=await api(tuneQuery(true),2000);$('tuneMsg').textContent=r?'Applied and saved to the robot.':'Could not reach the robot.'});

/* ---------- live status ---------- */
const MSG=['Calibrating. Keep me still.','Hold me upright to start.','Balancing.','I fell. Stand me up to restart.','Balance is off. Wheels are free.','IMU error. Check the MPU6050 wiring.'];
const CHIP=[['Calibrating','warn'],['Waiting','warn'],['Balancing','ok'],['Fallen','bad'],['Off','warn'],['IMU error','bad']];
let first=true,miss=0;
async function poll(){
  const s=await api('/api/state',700);
  if(!s){if(++miss>2){$('conn').textContent='Offline';$('conn').className='chip bad'}return}
  miss=0;
  const c=CHIP[s.state]||['?','warn'];$('conn').textContent=c[0];$('conn').className='chip '+c[1];
  $('msg').textContent=MSG[s.state]||'';
  $('angle').textContent=s.angle.toFixed(1);
  const a=Math.max(-45,Math.min(45,s.angle));$('bot').style.transform='rotate('+a+'deg)';
  balOn=!!s.bal;$('balBtn').textContent='Balance: '+(balOn?'ON':'OFF');
  if(first){first=false;
    ['kp','kd','ki','trim','lean','turn'].forEach(k=>{$(k).value=s[k]});
    $('spd').value=s.speed;$('spdV').textContent=s.speed+'%';
    $('lift').value=s.lift;$('liftV').textContent=s.lift;$('grip').value=s.grip;$('gripV').textContent=s.grip;}
  if(!$('tune').open)$('trim').value=s.trim;
}
setInterval(poll,300);poll();
</script></body></html>)rawliteral";

void sendOk() { server.send(200, "application/json", "{\"ok\":1}"); }

void setupRoutes() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });

  server.on("/api/move", HTTP_GET, []() {
    cmdX = constrain((int)server.arg("x").toInt(), -100, 100);
    cmdY = constrain((int)server.arg("y").toInt(), -100, 100);
    lastCmdMs = millis();
    sendOk();
  });
  server.on("/api/stop", HTTP_GET, []() { cmdX = 0; cmdY = 0; lastCmdMs = millis(); sendOk(); });

  server.on("/api/speed", HTTP_GET, []() {
    speedPct = constrain((int)server.arg("v").toInt(), 10, 100);
    sendOk();
  });

  server.on("/api/balance", HTTP_GET, []() {
    balanceOn = server.arg("on").toInt() != 0;
    sendOk();
  });

  server.on("/api/arm", HTTP_GET, []() {
    if (server.hasArg("lift")) liftTgt = constrain((int)server.arg("lift").toInt(), 0, 180);
    if (server.hasArg("grip")) gripTgt = constrain((int)server.arg("grip").toInt(), 0, 180);
    seqRunning = false;                     // manual control cancels any running sequence
    sendOk();
  });

  server.on("/api/seq", HTTP_GET, []() {
    String n = server.arg("name");
    if      (n == "pick")  startSeq(SEQ_PICK, sizeof(SEQ_PICK) / sizeof(Step));
    else if (n == "drop")  startSeq(SEQ_DROP, sizeof(SEQ_DROP) / sizeof(Step));
    else if (n == "home")  startSeq(SEQ_HOME, sizeof(SEQ_HOME) / sizeof(Step));
    else if (n == "open")  { seqRunning = false; gripTgt = GRIP_OPEN; }
    else if (n == "close") { seqRunning = false; gripTgt = GRIP_CLOSED; }
    sendOk();
  });

  server.on("/api/tune", HTTP_GET, []() {
    if (server.hasArg("kp"))   Kp      = server.arg("kp").toFloat();
    if (server.hasArg("ki"))   Ki      = server.arg("ki").toFloat();
    if (server.hasArg("kd"))   Kd      = server.arg("kd").toFloat();
    if (server.hasArg("trim")) trimDeg = constrain(server.arg("trim").toFloat(), -TRIM_LIMIT, TRIM_LIMIT);
    if (server.hasArg("lean")) leanMax = constrain(server.arg("lean").toFloat(), 0.0f, 10.0f);
    if (server.hasArg("turn")) turnMax = constrain(server.arg("turn").toFloat(), 0.0f, 200.0f);
    if (server.hasArg("save")) saveTune();
    sendOk();
  });

  server.on("/api/state", HTTP_GET, []() {
    char buf[360];
    snprintf(buf, sizeof(buf),
      "{\"angle\":%.1f,\"out\":%.0f,\"state\":%d,\"trim\":%.2f,\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,"
      "\"lean\":%.1f,\"turn\":%.0f,\"speed\":%d,\"lift\":%d,\"grip\":%d,\"seq\":%d,\"bal\":%d}",
      (float)g_angle, (float)g_out, (int)g_state, trimDeg, Kp, Ki, Kd,
      leanMax, turnMax, (int)speedPct, liftTgt, gripTgt, seqRunning ? 1 : 0, balanceOn ? 1 : 0);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", buf);
  });

  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
}

// Web server + servos run on core 0 (with the Wi-Fi stack);
// the balance loop owns core 1, so the network can never disturb balancing.
void webTask(void*) {
  for (;;) {
    server.handleClient();
    servoUpdate();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ================================ SETUP ====================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Self-balancing pick & drop robot ===");

  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, LOW);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);

  PWM_ATTACH(PIN_PWMA, CH_MOTOR_L, 20000, 8);     // 20 kHz = silent motors
  PWM_ATTACH(PIN_PWMB, CH_MOTOR_R, 20000, 8);
  PWM_WRITE(PIN_PWMA, CH_MOTOR_L, 0);
  PWM_WRITE(PIN_PWMB, CH_MOTOR_R, 0);

  PWM_ATTACH(PIN_SERVO_LIFT, CH_SERVO_L, 50, 14);
  PWM_ATTACH(PIN_SERVO_GRIP, CH_SERVO_G, 50, 14);
  servoWrite(CH_SERVO_L, PIN_SERVO_LIFT, liftCur);
  servoWrite(CH_SERVO_G, PIN_SERVO_GRIP, gripCur);

  loadTune();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  WiFi.setSleep(false);
  Serial.printf("Wi-Fi: %s / %s   Dashboard: http://%s\n", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());

  setupRoutes();
  server.begin();

  xTaskCreatePinnedToCore(balanceTask, "balance", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(webTask,     "web",     8192, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);   // everything runs in the two tasks above
}

# 🤖 DAVIDPUTRA — Self-Balancing Pick & Drop Robot
**Team Decepticons | Kalpataru Innovation Competition**

An autonomous, two-wheeled self-balancing inverted-pendulum robot equipped with a 2-DOF robotic arm and gripper[cite: 1, 3]. Powered by an **ESP32** running a dual-core FreeRTOS architecture, **DAVIDPUTRA** maintains dynamic equilibrium at 200 Hz while hosting a live mobile web dashboard for wireless driving, automated pick-and-place sequences, and real-time PID tuning saved directly to non-volatile flash memory[cite: 1].

---

## 🛠️ 3D CAD Model & Mechanical Assembly

The multi-layer chassis, motor mounts, and lightweight manipulator arm/gripper assembly were designed in **Autodesk Fusion 360**[cite: 3].

🔗 **3D CAD Repository (`.f3d` Source & Printable Files):**  
👉 **[View Hardware & 3D CAD Models]([https://github.com/pruthvirajpethe/YOUR-CAD-REPO-NAME](https://github.com/Earth-dlc/TEAM-DECEPTICONS-SELF-AUTONOMOUS-BOT-3D-file))**
---

## ✨ Key Engineering Features

* **Dual-Core FreeRTOS Separation:**
  * **Core 1 (`balanceTask`):** Dedicated 200 Hz (`5 ms`) real-time control loop handling 400 kHz I2C IMU sampling, complementary filtering, and PID motor control[cite: 1].
  * **Core 0 (`webTask`):** Handles the Wi-Fi Access Point, asynchronous HTTP web dashboard, and non-blocking servo choreography so network traffic never disturbs balance[cite: 1].
* **Zero External Library Dependencies:** Uses direct register-level I2C communication (`0x68`) for the MPU-6050 and native ESP32 libraries (`WiFi`, `WebServer`, `Wire`, `Preferences`). Compatible with ESP32 Arduino Core **2.x and 3.x**[cite: 1, 3].
* **Dynamic Auto-Trim (CoG Compensation):** Automatically adjusts the target balance angle (`trimDeg`) while stationary to counteract weight shifts caused by battery placement or payloads held by the gripper[cite: 1].
* **Motion-Profiled Pick & Drop Arm:** Slew-rate limited servo actuation (`1° per 15 ms`) prevents sudden mechanical jerk from destabilizing the inverted pendulum during automated **Pick**, **Drop**, and **Home** routines[cite: 1].
* **Built-In Mobile Web Dashboard:** Responsive HTML5/SVG interface (`http://192.168.4.1`) with live tilt visualization, touch D-pad + keyboard (`WASD`/Arrow) controls, servo sliders, and live PID flash storage[cite: 1].
* **Hardware Safety & Failsafes:** Includes 20 kHz silent motor PWM, motor dead-band compensation (`MIN_PWM = 22`), automatic motor cutoff beyond $\pm 35^\circ$ (`FALL_DEG`), upright auto-recovery (`< 4°` for `0.5 s`), and a `500 ms` Wi-Fi command watchdog[cite: 1].

---

## 📦 Bill of Materials (BOM)

| Qty | Component | Specifications / Notes |
| :-: | :--- | :--- |
| 1 | **ESP32 DevKit** | 30 or 38 pin (`ESP32 Dev Module` in Arduino IDE)[cite: 3] |
| 1 | **MPU-6050 (GY-521)** | 6-Axis Accelerometer + Gyroscope module (I2C)[cite: 3] |
| 1 | **TB6612FNG** | Dual H-Bridge Motor Driver (1.2 A/channel continuous)[cite: 3] |
| 2 | **N20 Geared DC Motors** | 6 V, ~200–300 RPM with 40–43 mm rubber wheels & brackets[cite: 3] |
| 2 | **MG90S Micro Servos** | Metal gear recommended (1× Arm Lift, 1× Gripper Claw)[cite: 3] |
| 1 | **DC-DC Buck Converter** | LM2596 or similar, adjusted to **5.0 V** ($\ge 3\text{ A}$) for ESP32 + Servos[cite: 2, 3] |
| 2 | **18650 Li-ion Cells** | 3.7 V cells wired in series (**2S = 7.4 V nominal / 8.4 V full**) + 2S BMS[cite: 2, 3] |
| 2 | **Decoupling Capacitors** | `470–1000 µF` (16 V) on 5 V servo rail; `100 µF` (16 V) across `VM` & `GND`[cite: 2, 3] |

*(See [`COMPONENTS.txt`](./COMPONENTS.txt) for the complete hardware and tool checklist[cite: 3].)*

---

## 🔌 Pinout & Wiring Architecture

```text
                       +------------------+
2x Li-ion (7.4V) ----> | SWITCH | 2S BMS  |
                       +--------+---------+
                                |
               +----------------+----------------+
               |                                 |
         +-----v------+                    +-----v------+
         | BUCK 5.0 V |                    |  TB6612FNG |--> Left N20
         +--+----+----+                    |  (VM=7.4V) |--> Right N20
            |    |                         +-----^------+
     +------v-+  +--> Servo 1 + Servo 2          | 6 control wires
     | ESP32  |------------------------------------+ + STBY
     | VIN    |--- I2C (21,22) ---> MPU6050 (3V3)
     +--------+--- GPIO 18, 19 ---> servo signals

```

### Pin Mapping Table

| Subsystem | Component Pin | ESP32 Pin / Power Rail | Function |
| --- | --- | --- | --- |
| **IMU (MPU-6050)** | `VCC` / `GND` | `3V3` / `GND` | Logic power (`AD0` $\rightarrow$ `GND` for `0x68`)

 |
|  | `SDA` / `SCL` | `GPIO 21` / `GPIO 22` | 400 kHz I2C bus

 |
| **Motor Driver (TB6612FNG)** | `STBY` | `GPIO 13` | H-Bridge enable / standby

 |
|  | `AIN1` / `AIN2` / `PWMA` | `GPIO 26` / `GPIO 25` / `GPIO 27` | Left N20 direction & 20 kHz speed PWM

 |
|  | `BIN1` / `BIN2` / `PWMB` | `GPIO 33` / `GPIO 32` / `GPIO 4` | Right N20 direction & 20 kHz speed PWM

 |
|  | `VM` / `VCC` | `Switched 7.4V` / `3V3` | Motor drive power / logic power

 |
| **Manipulator (MG90S)** | Arm Lift Signal | `GPIO 18` | 50 Hz (14-bit) PWM lift control

 |
|  | Gripper Signal | `GPIO 19` | 50 Hz (14-bit) PWM claw control

 |
|  | Servo `VCC` / `GND` | `Buck 5.0V` / `Common GND` | **Never** power servos from ESP32 pins

 |

> ⚠️ **Critical Safety Rules:**
> 1. Join **all grounds (`GND`)** together (Battery, Buck Converter, ESP32, TB6612FNG, MPU-6050, and both servos).
> 
> 
> 2. Calibrate the buck converter output to **5.0 V** with a multimeter *before* connecting the ESP32 or servos.
> 
> 
> 3. **Switch the robot OFF** before plugging in USB to upload code.
> 
> 
> 
> 

(See [`CONNECTIONS.txt`](https://www.google.com/search?q=./CONNECTIONS.txt) for step-by-step wiring details.)

---

## 🧠 Control Loop & Math

1. **Complementary Filter ($\alpha = 0.985$):** Combines high-frequency gyroscope integration ($\pm 500^\circ/\text{s}$, 44 Hz DLPF) with low-frequency accelerometer tilt ($\pm 2g$) at $200\text{ Hz}$:



$$\theta_k = 0.985 \cdot (\theta_{k-1} + \omega_y \cdot \Delta t) + 0.015 \cdot \text{atan2}(-a_x, a_z)$$


2. **PID Control with Clean Gyro Derivative:** Avoids derivative kick and sensor noise by applying $K_d$ directly to the bias-corrected pitch angular velocity ($\omega_y$):



$$u(t) = K_p \cdot e(t) + \int K_i \cdot e(t)\,dt + K_d \cdot \omega_y$$



where $e(t) = \theta - (\theta_{\text{trim}} + \theta_{\text{lean}})$.


3. **Auto Balance-Point Trim:** When joystick inputs are zero, slow integral feedback adapts $\theta_{\text{trim}}$ at $0.02^\circ/\text{(PWM}\cdot\text{s)}$ (clamped to $\pm 10^\circ$) to cancel static pitch offsets.



---

## 🚀 Setup, Flashing & First-Time Calibration

### 1. Flash the Firmware

1. Install **Arduino IDE** and add the **ESP32 board package** (works with Core 2.x and 3.x).


2. Open [`self_balancing_robot.ino`](https://www.google.com/search?q=./self_balancing_robot.ino).


3. Select **Board:** `ESP32 Dev Module` and set **Upload Speed:** `115200` or `921600`.


4. Ensure the robot's main power switch is **OFF**, connect via USB, and click **Upload**.



### 2. Connect to the Web Dashboard

1. Unplug USB, place or hold the robot upright and still on a flat surface, and switch it **ON** (keep it still for ~2 seconds while the gyroscope calibrates).


2. Connect your phone or laptop to the robot's Wi-Fi hotspot:


* **SSID:** `DAVIDPUTRA`

* **Password:** `12345678`



3. Open **`http://192.168.4.1`** in your web browser.



### 3. Orientation & PID Tuning Guide

1. **Verify Angle Sign:** Tilt the robot **forward** by hand while watching the dashboard. The angle must read **POSITIVE**. If it reads negative, set `ANGLE_SIGN` to `-1` in the code.


2. **Verify Motor Direction:** Hold the robot upright for `0.5 s` until the status chip turns green (`Balancing`). Tilt it forward—both wheels must spin **forward** to catch the fall. If reversed, flip `MOTOR_L_INVERT` and/or `MOTOR_R_INVERT`.


3. **Live PID Tuning (via Dashboard "Tuning" Panel):**
* Start with $K_i = 0$ (Defaults: $K_p = 22.0$, $K_d = 1.2$, $K_i = 0.0$).


* Increase **$K_p$** until the robot stays upright but oscillates rapidly.


* Increase **$K_d$** to dampen the oscillation without causing high-frequency motor buzzing.


* Click **Apply & save** to write your tuned parameters to the ESP32's flash memory.





---

## 📁 Repository Structure

```text
├── self_balancing_robot.ino   # Complete ESP32 firmware + embedded PROGMEM Web UI
├── CONNECTIONS.txt            # Pinout tables, power rules, and ASCII wiring diagram
├── COMPONENTS.txt             # Full Bill of Materials (BOM) and build guidelines
└── README.md                  # Project documentation

```

```

```

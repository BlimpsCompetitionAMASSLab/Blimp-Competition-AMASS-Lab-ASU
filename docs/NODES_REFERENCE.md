# ROS 2 Nodes Reference Guide

Complete reference for all ROS 2 nodes, their topics, services, parameters, and usage.

---

## Sensor Nodes

### 1. read_imu (Python)

**Package:** `sensors`  
**File:** `sensors/sensors/barometer.py` (should be IMU reader)  
**Launch Command:**
```bash
ros2 run sensors read_imu
```

**Description:**  
Reads 9-DOF orientation, linear acceleration, and angular velocity from Adafruit BNO055 IMU via I2C bus.

**Published Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/imu_data` | `blimp_interfaces/ImuData` | 200 Hz | Full 9-DOF IMU output |

**Topic Details:**
```yaml
/imu_data:
  header:
    stamp: {sec: 1707336475, nsec: 123456789}
    frame_id: 'imu_link'
  orientation:        # Euler angles (radians)
    x: 0.1234        # Roll (φ)
    y: -0.0456       # Pitch (θ)
    z: 0.9876        # Yaw (ψ)
  linear_acceleration:  # m/s² in body frame
    x: 0.05
    y: 0.02
    z: 9.78          # ~9.81 when level
  angular_velocity:     # rad/s in body frame
    x: 0.0           # Roll rate
    y: 0.0           # Pitch rate
    z: 0.0           # Yaw rate
```

**I2C Configuration:**
- Device Address: `0x28` (or `0x29` if COM3 pulled high)
- Bus: `/dev/i2c-1`
- Speed: 400 kHz
- Sampling Rate: 200 Hz

**Debug/Monitor:**
```bash
ros2 topic echo /imu_data
ros2 topic hz /imu_data          # Should show 200 Hz
ros2 topic bw /imu_data          # Show bandwidth
```

**Known Issues:**
- Unrealistic attitude angles → Check I2C connection
- Noise/jitter > 5° → Verify sensor calibration
- Missing data → Ensure I2C pull-ups are present

---

### 2. read_barometer (Python)

**Package:** `sensors`  
**File:** `sensors/sensors/barometer.py`  
**Launch Command:**
```bash
ros2 run sensors read_barometer
```

**Description:**  
Reads barometric pressure and derives altitude from Adafruit BMP390 sensor via I2C.

**Published Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/barometer_data` | `blimp_interfaces/BaroData` | 50 Hz | Altitude + pressure + temperature |

**Topic Details:**
```yaml
/barometer_data:
  header:
    stamp: {sec: 1707336475, nsec: 123456789}
    frame_id: 'baro_link'
  pressure: 101325.0       # Pressure in Pa (11100 Pa at 1m elevation)
  temperature: 22.5        # Temperature in °C
  altitude: 0.0            # Altitude in m (relative to sea level)
```

**I2C Configuration:**
- Device Address: `0x76` (or `0x77` if SDO pulled high)
- Bus: `/dev/i2c-1`
- Speed: 400 kHz
- Sampling Rate: 50 Hz
- Sea Level Pressure: Configure in code (default: 101325.0 Pa)

**Altitude Calculation:**
$$\text{altitude} = 44330 \left[ 1 - \left( \frac{P}{P_0} \right)^{1/5.255} \right]$$

Where:
- $P$ = current pressure (Pa)
- $P_0$ = reference sea-level pressure (Pa)

**Debug/Monitor:**
```bash
ros2 topic echo /barometer_data
ros2 topic hz /barometer_data          # Should show 50 Hz

# Record to file
ros2 topic echo /barometer_data > baro_data.log
```

**Calibration:**
```python
# Update sea-level pressure if running at altitude
# Edit barometer.py and modify:
SEA_LEVEL_PRESSURE = 101325.0  # Change to local pressure
```

**Known Issues:**
- Altitude reads wrong (e.g., 1000m when at sea level) → Recalibrate sea-level pressure
- Noise ±0.5m → Apply Kalman filter (already done in autonomy nodes)
- Sensor not responding → Check power supply 5V is stable

---

### 3. detect_node (C++)

**Package:** `sensors_cpp`  
**File:** `sensors_cpp/src/detect_node.cpp`  
**Build:** C++ (CMake)  
**Launch Command:**
```bash
ros2 run sensors_cpp detect_node
```

**Description:**  
Real-time object detection using YOLOv5 trained model. Captures images from Raspberry Pi Camera and publishes balloon detections with bounding boxes and centroid.

**Published Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/cam_data` | `blimp_interfaces/CameraCoord` | ~10 Hz | Detected balloons with bbox + centroid |

**Topic Details:**
```yaml
/cam_data:
  detections:
    - bbox_x1: 150       # Top-left corner X (pixels)
      bbox_y1: 100       # Top-left corner Y
      bbox_x2: 200       # Bottom-right corner X
      bbox_y2: 150       # Bottom-right corner Y
      centroid_x: 175    # Bounding box center X
      centroid_y: 125    # Bounding box center Y
      confidence: 0.92   # Detection confidence (0-1)
      class_name: "balloon"  # Object class
      width: 50          # Bounding box width (pixels)
      height: 50         # Bounding box height (pixels)
```

**Parameters:**
- **Model:** YOLOv5s (quantized for CPU)
- **Confidence Threshold:** 0.5
- **Input Resolution:** 640x480 (configurable)
- **Inference Time:** ~80-150ms on Pi 4 (CPU)
- **Frame Rate:** Limited by YOLOv5 processing; ~10 Hz typical

**Configuration:**
```bash
# Model path (in detect_node.cpp)
const std::string MODEL_PATH = "/path/to/yolov5s_balloon.pt";

# Confidence threshold
const float CONF_THRESHOLD = 0.5;

# Image input size
const int IMG_WIDTH = 640;
const int IMG_HEIGHT = 480;
```

**Debug/Monitor:**
```bash
# View detections
ros2 topic echo /cam_data

# Monitor frequency
ros2 topic hz /cam_data          # Should show ~10 Hz

# View processed images (if visualization enabled)
ros2 run image_view image_view image:=/camera/detection_debug
```

**Expected Behavior:**
- When olive balloon visible: detection confidence > 0.75
- Centroid coordinates update smoothly
- Processing latency < 150ms
- No detections when balloon out of view

**Known Issues:**
- Inference too slow → Use quantized model (already done)
- False positives in low light → Improve training dataset
- Centroid jitter > 10px → Add temporal averaging filter

---

## Control Nodes

### 4. esc_driver (Python)

**Package:** `controls`  
**File:** `controls/controls/esc_driver.py`  
**Launch Command:**
```bash
ros2 run controls esc_driver
```

**Description:**  
Low-level ESC PWM driver using pigpio daemon. Converts ROS 2 ESC input messages to GPIO PWM signals controlling 4 brushless ESCs, each driving one motor.

**Subscribed Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/esc_input` | `blimp_interfaces/EscInput` | 50 Hz | Motor PWM targets (1000-2000 µs) |

**Message Format:**
```yaml
/esc_input:
  pwm:
    - 1500  # ESC 1 PWM (µs) -> Motor 1
    - 1500  # ESC 2 PWM (µs) -> Motor 2
    - 1500  # ESC 3 PWM (µs) -> Motor 3
    - 1500  # ESC 4 PWM (µs) -> Motor 4
```

**GPIO Pin Mapping (ESCs to Motors):**
- ESC 1 (GPIO 5): Controls Motor 1 (left)
- ESC 2 (GPIO 6): Controls Motor 2 (right)
- ESC 3 (GPIO 13): Controls Motor 3 (bottom - vertical thrust)
- ESC 4 (GPIO 26): Unused - reserved for future expansion

**Architecture:**
- Each GPIO pin sends PWM signal to one ESC
- Each ESC receives 3 connections: Signal (from GPIO), Power (from battery), Ground (common)
- Motor connects only to its ESC, not directly to GPIO
- ESC provides all power and control to motor

**PWM Specifications:**
- **Frequency:** 50 Hz (20 ms period)
- **Pulse Range:** 1000-2000 µs (5-10% duty cycle)
- **Neutral:** 1500 µs (motor idle)
- **Min Thrust:** 1000 µs (motor off/minimum throttle)
- **Max Thrust:** 2000 µs (maximum throttle)

**Dependencies:**
- `pigpio` daemon must be running: `sudo systemctl status pigpiod`
- Requires hardware PWM support (Raspberry Pi has native PWM)

**Critical Notes:**
- ⚠️ **Propellers must be removed during debugging!**
- ESC requires armed state before responding to commands
- See `arming.py` launch file for ESC arming sequence

**Debug/Monitor:**
```bash
# View published commands
ros2 topic echo /esc_input

# Send test command (DANGEROUS! Remove propellers first!)
ros2 topic pub /esc_input blimp_interfaces/EscInput "{pwm: [1500, 1500, 1500, 1500]}" -r 10
```

**Troubleshooting:**
```bash
# Check pigpiod running
sudo systemctl status pigpiod

# Restart pigpiod
sudo systemctl restart pigpiod

# Verify GPIO pins accessible
ls -l /dev/gpiomem

# Manual PWM test via command line (remove propellers first!)
pigs hwm 5 50 1500000   # ESC 1 (Motor 1) to neutral (1500 µs)
pigs hwm 6 50 1500000   # ESC 2 (Motor 2) to neutral
pigs hwm 13 50 1500000  # ESC 3 (Motor 3) to neutral
pigs hwm 26 50 1500000  # ESC 4 (Motor 4) to neutral

pigs hwm 5 50 0         # Stop ESC 1 PWM signal
pigs hwm 6 50 0         # Stop ESC 2 PWM signal
pigs hwm 13 50 0        # Stop ESC 3 PWM signal
pigs hwm 26 50 0        # Stop ESC 4 PWM signal
```

---

### 5. mode_switcher (Python)

**Package:** `controls`  
**File:** `controls/controls/mode_switcher.py`  
**Launch Command:**
```bash
ros2 run controls mode_switcher
```

**Description:**  
Mode multiplexer that selects between manual joystick input and autonomous control output. Listens to mode selection signal and routes appropriate PWM commands to ESC.

**Subscribed Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/esc_manual_input` | `blimp_interfaces/EscInput` | 50 Hz | Joystick-based PWM (manual mode) |
| `/esc_balloon_input` | `blimp_interfaces/EscInput` | 50 Hz | Autonomous control PWM output |
| `/joy` | `sensor_msgs/Joy` | 50 Hz | Joystick input (for mode toggle button) |

**Published Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/esc_input` | `blimp_interfaces/EscInput` | 50 Hz | Final selected PWM command (→ esc_driver) |

**Mode Selection Logic:**
```
if joystick_mode_button_pressed:
    /esc_input ← /esc_manual_input
else:
    /esc_input ← /esc_balloon_input
```

**Joystick Button Mapping:**
- **Mode Toggle Button:** RB (right shoulder)
- **Press trigger:** Switch between MANUAL ↔ AUTONOMOUS
- **LED Indicator:** (Future) LED changes color based on mode

**Configuration:**
```python
# Default mode at startup
DEFAULT_MODE = "MANUAL"

# Joystick button for mode switching
MODE_SWITCH_BUTTON = 5  # RB button (check joy message)
```

**Debug/Monitor:**
```bash
# Monitor selected mode
ros2 topic echo /esc_input

# Check joystick input
ros2 topic echo /joy

# Monitor manual input
ros2 topic echo /esc_manual_input

# Monitor autonomous input
ros2 topic echo /esc_balloon_input
```

**Behavior:**
- **MANUAL Mode:** Joystick directly controls motors via `/esc_manual_input`
- **AUTONOMOUS Mode:** Control system computes commands; publishes to `/esc_balloon_input`
- Mode switching is instantaneous (potential safety issue if not handled carefully)

---

### 6. joy_to_esc_input (Python)

**Package:** `manual_control`  
**File:** `manual_control/manual_control/joy_to_esc_input.py`  
**Launch Command:**
```bash
ros2 run manual_control joy_to_esc_input
```

**Description:**  
Converts Xbox/joystick analog input to ESC PWM commands for manual flight control.

**Subscribed Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/joy` | `sensor_msgs/Joy` | 50 Hz | Joystick input from `game_controller_node` |

**Published Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/esc_manual_input` | `blimp_interfaces/EscInput` | 50 Hz | Mapped motor commands |

**Joystick Mapping:**

```yaml
Xbox Controller Layout (Manual Mode):
   
   LB      RB
   ┌───────────┐
   │ Y    X    │
   │A    B     │ (RT: Right Trigger = yaw right)
   │ LS    RS  │ (LT: Left Trigger = yaw left)
   │ ───────── │ (LStick Y: altitude up/down)
   │ Back  Start│ (RStick Y: altitude control)
   └───────────┘

Mapping to Motors (3-motor configuration):
  LStick Y: Altitude control (both left & right motors)
  LT/RT: Yaw control (differential thrust left vs right)
  RStick Y: Altitude control (bottom motor vertical thrust)
  RB button: Mode switch (manual vs autonomous)
```

**Control Laws (Example):**
```python
# Throttle (trigger sticks 0-1 range → 1500-2000 µs PWM)
throttle = 1500 + (right_trigger - left_trigger) * 500 / 2

# Forward/backward (left stick Y: -1 to +1)
forward_cmd = left_stick_y * 200

# Left/right (left stick X: -1 to +1)
yaw_cmd = left_stick_x * 150

# Compute individual motor commands
motor_1_pwm = throttle + forward_cmd - yaw_cmd
motor_2_pwm = throttle + forward_cmd + yaw_cmd
motor_3_pwm = throttle - forward_cmd - yaw_cmd
motor_4_pwm = throttle - forward_cmd + yaw_cmd

# Clamp to valid range [1100, 1900]
motor_pwm = clamp(motor_pwm, 1100, 1900)
```

**Debug/Monitor:**
```bash
# View joystick raw input
ros2 topic echo /joy

# View mapped ESC output
ros2 topic echo /esc_manual_input

# Expected output (neutral stick, centered throttle):
# pwm: [1500, 1500, 1500, 1500]
```

---

## Autonomy Nodes

### 7. inv_kine (C++)

**Package:** `sensors_cpp`  
**File:** `sensors_cpp/src/inv_kine.cpp`  
**Launch Command:**
```bash
ros2 run sensors_cpp inv_kine
```

**Description:**  
Inverse kinematics solver. Converts desired vforce vector (Fx, Fy, Fz, τx, τy, τz) from control system into individual motor PWM commands using pseudo-inverse transformation.

**Subscribed Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/imu_data` | `blimp_interfaces/ImuData` | 200 Hz | Orientation for frame transformation |
| `/barometer_data` | `blimp_interfaces/BaroData` | 50 Hz | Altitude feedback |
| `/balloon_input` | `blimp_interfaces/CartCoord` | Variable | Desired forces/moments |

**Published Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/forces` | `blimp_interfaces/CartCoord` | 50 Hz | Computed motor forces |
| `/esc_balloon_input` | `blimp_interfaces/EscInput` | 50 Hz | Final motor PWM commands (1000-2000 µs) |

**Control Algorithm:**

The inverse kinematics solves:
$$\mathbf{u} = \mathbf{T}^{\dagger} \mathbf{f}$$

Where:
- $\mathbf{f}$ = desired force vector [Fx, Fy, Fz, τx, τy, τz]ᵀ
- $\mathbf{T}$ = thrust configuration matrix (3×6, non-square)
- $\mathbf{T}^{\dagger}$ = Moore-Penrose pseudo-inverse
- $\mathbf{u}$ = motor commands [u1, u2, u3]ᵀ

**Thrust Configuration Matrix:**
```
Motor positions (body frame):
  Motor 1 (GPIO 5): Left motor [-y position, yaw control]
  Motor 2 (GPIO 6): Right motor [+y position, yaw control]
  Motor 3 (GPIO 13): Bottom motor [+z direction, vertical thrust]

Thrust directions:
  - Left & Right motors: Horizontal for yaw control, vertical for altitude
  - Bottom motor: Vertical (+z, adjusted by frame orientation)
```

**Debug/Monitor:**
```bash
# View computed forces
ros2 topic echo /forces

# View output motor commands
ros2 topic echo /esc_balloon_input
```

---

### 8. pi_controller (C++)

**Package:** `sensors_cpp`  
**File:** `sensors_cpp/src/test.cpp` (may be renamed to pi_controller)  
**Launch Command:**
```bash
ros2 run sensors_cpp pi_controller
```

**Description:**  
Vision-based PI controller computing desired forces based on balloon centroid position and altitude error.

**Subscribed Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/cam_data` | `blimp_interfaces/CameraCoord` | ~10 Hz | Balloon detection with centroid |
| `/barometer_data` | `blimp_interfaces/BaroData` | 50 Hz | Altitude for feedback |

**Published Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/balloon_input` | Custom force vector | 50 Hz | Desired forces [Fx, Fy, Fz, τx, τy, τz] |

**Control Laws:**

**Yaw Control (from camera centroid X):**
```
error_u = balloon_centroid.x - FRAME_CENTER_X  (pixels)
if abs(error_u) < DEAD_ZONE:
    yaw_cmd = 0
else:
    yaw_cmd = Kp_yaw * error_u + Ki_yaw * integral(error_u)
```

**Altitude Control (from barometer) :**
```
altitude_err = current_altitude - TARGET_ALTITUDE  (meters)
heave_cmd = Kp_alt * altitude_err + Ki_alt * integral(altitude_err)
```

**Default PID Gains:**
```python
# Yaw PI gains
Kp_yaw = 0.5
Ki_yaw = 0.1

# Altitude PI gains
Kp_alt = 0.3
Ki_alt = 0.05

# Camera frame center (pixels, 640x480)
FRAME_CENTER_X = 320
FRAME_CENTER_Y = 240
ERROR_THRESHOLD = 50  # Dead zone in pixels
```

**Debug/Monitor:**
```bash
# View computed commands
ros2 topic echo /balloon_input

# Monitor controller state
ros2 run rqt_gui  # Open rqt and add generic plot
```

**Tuning Guidelines:**
- **Kp too high:** Oscillations (> 1.5 Hz). Reduce by factor of 1.5
- **Kp too low:** Sluggish response. Increase by factor of 1.5
- **Ki too high:** Integral windup. Reduce or disable (set to 0)
- **Ki too low:** Steady-state error persists. Increase gradually

---

## Utility Nodes

### 9. game_controller_node (ROS 2 standard)

**Package:** `joy` (ROS 2 built-in)  
**Launch Command:**
```bash
ros2 run joy game_controller_node
```

**Description:**  
Standard ROS 2 joy package node. Reads input from mapped Xbox/joystick and publishes `/joy` topic.

**Published Topics:**

| Topic | Message Type | Frequency | Description |
|-------|---|---|---|
| `/joy` | `sensor_msgs/Joy` | 50 Hz | Joystick axes and buttons |

**Joy Message Format:**
```yaml
/joy:
  header: {stamp, frame_id: 'joy'}
  axes:
    [0]  float: Left stick X (-1 left → +1 right)
    [1]  float: Left stick Y (-1 down → +1 up)
    [2]  float: LT trigger (-1 released → +1 pressed)
    [3]  float: Right stick X (-1 left → +1 right)
    [4]  float: Right stick Y (-1 down → +1 up)
    [5]  float: RT trigger (-1 released → +1 pressed)
    [6]  float: D-pad X (-1 left, 0 center, +1 right)
    [7]  float: D-pad Y (-1 down, 0 center, +1 up)
  buttons:
    [0]  int: A button
    [1]  int: B button
    [2]  int: X button
    [3]  int: Y button
    [4]  int: LB button
    [5]  int: RB button (mode switch)
    [6]  int: Back button
    [7]  int: Start button
    [8]  int: Left stick click
    [9]  int: Right stick click
    [10] int: Xbox button
```

**Setup:**
```bash
# Connect Xbox controller via Bluetooth or USB
# Verify device recognized
ls -l /dev/input/

# Test joystick directly
jstest /dev/input/js0

# Launch game_controller_node
ros2 run joy game_controller_node --ros-args -p device_name:=/dev/input/js0
```

---

## Complete Node Dependency Graph

```
Sensor Nodes:
  read_imu ──────────┐
                     ├──→ inv_kine ──────┬──→ esc_balloon_input
  read_barometer ────┤                   │
                     ├──→ pi_controller ──┤
  detect_node ───────┘                   │
                                         ├──→ mode_switcher ──→ esc_input ──→ esc_driver → Motors
  game_controller_node ──→ joy_to_esc_input ──→ esc_manual_input ──┘
```

---

## Launch System Overview

### Complete Autonomous System
```bash
ros2 launch launch updated_launch.py
# Starts: read_imu, read_barometer, detect_node, inv_kine, pi_controller,
#         esc_driver, mode_switcher, game_controller_node, joy_to_esc_input
```

### Manual Control Only
```bash
ros2 launch launch drone_manual_launch.py
# Starts: joy_to_esc_input, mode_switcher, esc_driver, game_controller_node
```

### ESC Arming (before flight)
```bash
ros2 launch launch arming.py
# Sends ESC calibration/arming signals via esc_driver
```

---

**Last Updated:** February 9, 2026  
**Release Tag:** `platform_report_2025_02`

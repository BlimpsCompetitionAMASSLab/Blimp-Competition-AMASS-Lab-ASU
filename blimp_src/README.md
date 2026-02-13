# BLIMP Platform: ROS 2 Source Code Repository
---

## Table of Contents
1. [Quick Start](#quick-start)
2. [Repository Structure](#repository-structure)
3. [Package Descriptions](#package-descriptions)
4. [System Architecture](#system-architecture)
5. [Installation & Setup](#installation--setup)
6. [Running the System](#running-the-system)
7. [ROS 2 Topics & Nodes](#ros-2-topics--nodes)
8. [Configuration Files](#configuration-files)
9. [Testing & Validation](#testing--validation)
10. [Troubleshooting](#troubleshooting)

---

## Quick Start

For experienced ROS 2 users:

```bash
# 1. Clone repository
mkdir -p ~/blimp_ws/src
cd ~/blimp_ws/src
git clone https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU.git blimp_src

# 2. Install dependencies
cd ~/blimp_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/blimp_src/requirements.txt

# 3. Build (interfaces first)
colcon build --packages-select blimp_interfaces
source install/setup.bash
colcon build

# 4. Launch system
ros2 launch bomber_launch.py  # Or other launch script in launch/ directory

# 5. In another terminal, verify nodes
ros2 node list
```

For detailed instructions, see [Installation & Setup](#installation--setup) section below.

---

## Repository Structure

```
blimp_src/
├── README.md                                    # This file
├── LICENSE                                      # Project license
├── requirements.txt                             # Python package dependencies
├── consolidated_setup.sh                        # Automated setup script
├── test_data                                    # General test data files
│
├── blimp_interfaces/                            # ROS 2 message & service definitions
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/                                     # Custom message types
│   │   ├── ImuData.msg                         # 9-DOF IMU output
│   │   ├── BaroData.msg                        # Barometer output (altitude + temp)
│   │   ├── CartCoord.msg                       # Cartesian coordinates
│   │   ├── CameraCoord.msg                     # Camera detection coordinates
│   │   ├── EscInput.msg                        # ESC PWM commands
│   │   ├── Bool.msg                            # Boolean wrapper
│   │   ├── LidarData.msg                       # Lidar range data
│   │   └── UtcTime.msg                         # Timestamp message
│   └── srv/                                     # Service definitions
│       ├── Detection.srv                        # Camera detection service
│       └── GetCamera.srv                        # Camera retrieval service
│
├── bag_files/                                   # ROS 2 bag recordings
│   ├── Barometer_ros2bag/                      # Barometer data recordings
│   ├── rosbag2_2026_02_11-12_11_47/           # Test recording session 1
│   └── rosbag2_2026_02_11-12_19_11/           # Test recording session 2
│
├── logs/                                        # Test data and flight logs
│   ├── client.py                               # Data logging client
│   ├── PI_Data_2.txt                           # Position control test data
│   ├── PI_Data_3.txt                           # Position control test data
│   ├── data_record_2.txt                       # Comprehensive test log
│   ├── data_record_3.txt                       # Comprehensive test log
│   ├── data_record_4.txt                       # IMU + motor test (Test 1)
│   ├── data_record_5.txt                       # IMU + motor test (Test 2)
│   ├── data_record_6.txt                       # (corrupted - not used)
│   ├── vicom_test_1 through vicom_test_7       # Vicon motion capture tests
│   └── test_data                               # Additional test datasets
│
├── sensors/ (Python ROS 2 Nodes)                # Sensor reader nodes
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── sensors/
│   │   ├── __init__.py
│   │   ├── barometer.py                        # BMP390 barometer reader
│   │   ├── BNO055.py                           # BNO055 IMU driver (deprecated)
│   │   ├── BNO085.py                           # BNO085 IMU driver (alternative)
│   │   ├── LED.py                              # LED indicator control
│   │   ├── Lidar.py                            # Lidar range sensor driver
│   │   ├── record_data.py                      # Data logging utility
│   │   └── Sonar.py                            # Sonar/ultrasonic driver
│   └── test/                                    # Unit tests
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
│
├── sensors_cpp/ (C++ ROS 2 Nodes)               # High-performance sensor processing
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/
│   │   └── shared_frame.h                      # Coordinate frame definitions
│   └── src/
│       ├── Balloon_pi.cpp                      # (Experimental)
│       ├── Extremum_Seeking.cpp                # Extremum seeking control algorithm
│       ├── force_to_ESC_input.cpp              # Force-to-PWM transformation (inverse kinematics)
│       ├── inv_kine.cpp                        # Inverse kinematics solver
│       ├── old_cam.cpp                         # Legacy camera driver (deprecated)
│       └── detect_node.cpp                     # YOLOv5 balloon detection node
│
├── camera/ (Python Camera Nodes)                # Vision and camera processing
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── camera/
│   │   ├── __init__.py
│   │   └── (camera processing nodes)
│   ├── resource/
│   └── test/
│
├── controls/ (Python Control Nodes)             # Motor & control driver nodes
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── controls/
│   │   ├── __init__.py
│   │   ├── baro_control.py                     # Barometric altitude control
│   │   ├── Bomber_Cntrl.py                     # Main control logic (legacy)
│   │   ├── bomber_mux.py                       # Control mode multiplexer
│   │   ├── esc_driver.py                       # ESC PWM control driver (pigpio-based)
│   │   ├── mode_switcher.py                    # Manual/Autonomous mode selector
│   │   ├── mode_switcher.py.save               # Backup of mode_switcher.py
│   │   ├── net_servo.py                        # Network servo control
│   │   ├── random_walk.py                      # Random walk algorithm (testing)
│   │   └── rudolph_mode_switcher.py            # Alternative mode switcher
│   └── test/                                    # Unit tests
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
│
├── manual_control/ (Python Manual Control)      # Joystick input handling
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── manual_control/
│   │   ├── __init__.py
│   │   ├── joy_to_esc_input.py                 # Joystick-to-motor mapping
│   │   └── manual_bridge.py                    # Manual control bridge
│   └── test/
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
│
└── launch/ (Launch Files)                       # ROS 2 launch configurations
    ├── arming.py                               # ESC arming sequence launcher
    ├── autonomous_launch.py                    # Autonomous flight launcher
    ├── BNO085_calibrate.py                     # IMU calibration launcher
    ├── bomber_launch.py                        # Main blimp launcher (legacy)
    ├── cpp_auto_launch.py                      # C++ autonomous nodes launcher
    ├── drone_manual_launch.py                  # Manual drone control launcher
    ├── Extremum_test.py                        # Extremum seeking test launcher
    ├── LED_test.py                             # LED test launcher
    ├── program_esc.py                          # ESC programming launcher
    ├── Rudolph_Launch.py                       # Rudolph variant launcher
    ├── servo_test.py                           # Servo motor test launcher
    ├── test_launch.py                          # General test launcher
    ├── updated_launch.py                       # MAIN LAUNCHER (use this for full system)
    ├── pi_DBlue_1 through pi_DBlue_4           # Device-specific configurations
    └── recorded data storage/                   # Legacy test data location
```

---

## Package Descriptions

### 1. **blimp_interfaces** - Custom ROS 2 Message Types
**Purpose:** Defines custom ROS 2 messages and services used across all nodes  
**Key Messages:**
- `ImuData.msg` - 9-DOF IMU output (orientation, linear acceleration, angular velocity)
- `BaroData.msg` - Barometric pressure and temperature readings
- `CartCoord.msg` - Cartesian coordinates (x, y, z)
- `CameraCoord.msg` - Camera detection bounding box & centroid
- `EscInput.msg` - ESC PWM input commands
- `LidarData.msg` - Lidar/range sensor data

**Build:** C++ (CMake)  
**Dependency:** Always build first before other packages!

```bash
colcon build --packages-select blimp_interfaces
```

---

### 2. **sensors** - Python Sensor Reader Nodes
**Purpose:** Python-based drivers for IMU, barometer, and other sensors  
**Key Nodes:**
- `read_barometer.py` - Publishes to `/barometer_data` (BMP390 readings) @ 50 Hz
- `read_imu.py` - Publishes to `/imu_data` (BNO055 readings) @ 200 Hz (9-DOF)
- `LED.py` - LED indicator control
- `Lidar.py` - Lidar distance measurements
- `Sonar.py` - Ultrasonic range sensor
- `record_data.py` - Data logging utility

**Build:** Python  
**Nodes to Launch:**
```bash
ros2 run sensors read_barometer
ros2 run sensors read_imu
```

---

### 3. **sensors_cpp** - C++ Sensor Processing Nodes
**Purpose:** High-performance C++ nodes for computer vision & advanced control  
**Key Nodes:**
- `detect_node` - YOLOv5 balloon detection (publishes to `/cam_data`)
- `inv_kine` - Inverse kinematics solver (subscribes to `/imu_data`, `/barometer_data`, publishes forces)
- `force_to_esc_input` - Transforms forces into PWM commands
- `Extremum_Seeking` - Extremum seeking control algorithm

**Build:** C++ (CMake)  
**Launch:**
```bash
ros2 run sensors_cpp detect_node
ros2 run sensors_cpp inv_kine
```

---

### 4. **camera** - Vision and Camera Processing
**Purpose:** Camera-based vision processing and target detection  
**Key Nodes:**
- Camera capture and processing nodes
- Vision-based position tracking
- Target detection and localization

**Build:** Python  
**Launch:**
```bash
ros2 run camera <camera_node>
```

---

### 5. **controls** - Motor & Control Driver Nodes
**Purpose:** Low-level motor control and mode switching  
**Key Nodes:**
- `esc_driver.py` - Main ESC PWM driver (uses `pigpio` library)
  - Subscribes to: `/esc_input` (PWM targets)
  - Controls GPIO pins 17, 22, 23, 24, 27 for motor PWM
- `mode_switcher.py` - Manual/Autonomous mode selector
  - Listens to: `/ESC_Manual_input`, `/ESC_balloon_input`, `/joy`
  - Publishes final: `/esc_input`
- `joy_to_esc_input.py` - Joystick-to-motor mapping (manual mode)
- `baro_control.py` - Altitude-based control
- `bomber_mux.py` - Control multiplexer

**Build:** Python  
**Key Dependency:** `pigpio` daemon must be running

```bash
sudo systemctl start pigpiod
ros2 run controls esc_driver
```

---

### 6. **manual_control** - Joystick Input Handling
**Purpose:** Convert Xbox/joystick input to motor commands  
**Key Nodes:**
- `joy_to_esc_input.py` - Maps joystick axes to ESC PWM values
- `manual_bridge.py` - Bridge between joystick and control system

**Build:** Python  
**Launch:**
```bash
ros2 run joy game_controller_node  # Standard ROS 2 joy package
ros2 run manual_control joy_to_esc
```

---

### 7. **launch** - Launch Files (Entry Points)
**Purpose:** ROS 2 launch scripts to start subsystems or complete system

**Most Important Launch File:**
```bash
ros2 launch launch updated_launch.py
```
This launches the **complete autonomous system** including:
- All sensor nodes (IMU, barometer, camera)
- Control nodes (esc_driver, mode_switcher)
- Autonomy nodes (detection, inverse kinematics, PI controller)

**Other Launch Files:**
- `arming.py` - ESC arming sequence (run before flight!)
- `autonomous_launch.py` - Autonomy-only mode
- `cpp_auto_launch.py` - C++ node launcher
- `servo_test.py` - Motor test without full system

---

### 8. **logs** - Flight Test Data & Recordings
**Purpose:** Comprehensive flight test data logs with sensor readings and motor commands

**Test Data Files:**
- **Position Control Tests:**
  - `PI_Data_2.txt` - Position control test (~261 seconds, vertical motor control)
  - `PI_Data_3.txt` - Position control test (~228 seconds, altitude stabilization)
  
- **IMU + Motor Dashboard Tests:**
  - `data_record_4.txt` - Test 1 (~54 seconds, comprehensive IMU + motor logging)
  - `data_record_5.txt` - Test 2 (~58 seconds, full sensor suite active)
  - `data_record_6.txt` - *Corrupted data - do not use*

- **Vicon Motion Capture Tests:**
  - `vicom_test_1` through `vicom_test_7` - Motion capture validation data

**Data Format:**
```
Position: x,y L Motor: val R Motor: val U Motor: val D Motor: val time: val
Euler Angles: r,p,y Gyro: x,y,z Linear Accel: x,y,z L Motor: val R Motor: val U Motor: val D Motor: val time: val
```

**Analysis Scripts:**
See `../plots/` directory for Python scripts to analyze and visualize this data.

---

### 9. **bag_files** - ROS 2 Bag Recordings
**Purpose:** ROS 2 bag file recordings of live topic data

**Recorded Sessions:**
- `Barometer_ros2bag/` - Barometer sensor data recordings
- `rosbag2_2026_02_11-12_11_47/` - Full system recording (Session 1)
- `rosbag2_2026_02_11-12_19_11/` - Full system recording (Session 2)

**Playback:**
```bash
ros2 bag play bag_files/rosbag2_2026_02_11-12_11_47/
```

**Info:**
```bash
ros2 bag info bag_files/rosbag2_2026_02_11-12_11_47/
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│           BLIMP AUTONOMOUS CONTROL SYSTEM               │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  SENSOR LAYER (20-200 Hz)                               │
│  ├─ read_imu → /imu_data (200 Hz, BNO055)               │
│  ├─ read_barometer → /barometer_data (50 Hz, BMP390)    │
│  └─ detect_node → /cam_data (30 fps, YOLOv5)            │
│                                                          │
│  ESTIMATION LAYER (50 Hz)                               │
│  └─ Kalman Filter → /state_estimate (fused pose)        │
│                                                          │
│  CONTROL LAYER (50 Hz)                                  │
│  ├─ pi_controller (vision-based yaw & altitude)         │
│  ├─ inv_kine (force-to-PWM transformation)              │
│  └─ mode_switcher (manual/autonomous selection)         │
│                                                          │
│  ACTUATION LAYER (50 Hz)                                │
│  ├─ esc_driver (GPIO PWM generation via pigpio)         │
│  └─ Motors (4x servo motors producing thrust)           │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

**ROS 2 Topic Graph:**
```
/imu_data ──┐
            ├─→ [inv_kine] ──→ /forces
/baro_data ─┤
            ├─→ [pi_controller] ──→ /esc_balloon_input
/cam_data ──┤
            │
/joy ──→ [mode_switch] ──→ /esc_input ──→ [esc_driver] ──→ GPIO PWM
            ↑
    /esc_manual_input (from joystick)
```

---

## Installation & Setup

### Prerequisites

**Hardware:**
- Raspberry Pi 4 (4GB or 8GB RAM)
- IMU: Adafruit BNO055
- Barometer: Adafruit BMP390
- Servo Motors: 4x Tower Pro SG90 (or compatible)
- ESC: 30A Electronic Speed Controller
- Camera: Raspberry Pi Camera v2 (8MP)
- Battery: 3S LiPo 5000mAh

**Software Requirements:**
- Ubuntu 22.04 LTS (Jammy) for Raspberry Pi
- Python 3.10+
- ROS 2 Humble

### Step 1: Install ROS 2 Humble

```bash
# Install ROS 2 keyring and repository
sudo apt install curl -y
curl -sSL https://repo.ros.org/setup.key | sudo apt-key add -
sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"

# Update and install
sudo apt update
sudo apt install ros-humble-desktop -y

# Add to bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Create ROS 2 Workspace

```bash
mkdir -p ~/blimp_ws/src
cd ~/blimp_ws
```

### Step 3: Clone Repository

```bash
cd ~/blimp_ws/src
git clone https://github.com/RAS598-2025-S-Team03/BLIMP-Packages.git
cd ..
```

### Step 4: Install System Dependencies

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 5: Install Python Dependencies

```bash
pip install -r src/BLIMP-Packages/requirements.txt
```

**requirements.txt includes:**
- `rclpy` - ROS 2 Python client
- `pigpio` - GPIO PWM control
- `adafruit-bno055` - IMU driver
- `adafruit-bmp3xx` - Barometer driver
- `numpy` - Numerical computing
- `opencv-python` - Computer vision
- `ultralytics` - YOLOv5 support

### Step 6: Build Workspace

```bash
# Build interfaces FIRST (critical!)
colcon build --packages-select blimp_interfaces
source install/setup.bash

# Build all packages
colcon build
```

### Step 7: Enable Hardware Interfaces

```bash
# Enable I2C
echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt

# Enable SPI
echo "dtparam=spi=on" | sudo tee -a /boot/config.txt

# Enable Camera
echo "start_x=1" | sudo tee -a /boot/config.txt
echo "gpu_mem=256" | sudo tee -a /boot/config.txt

# Reboot for changes to take effect
sudo reboot
```

### Step 8: Start pigpio Daemon

```bash
# Enable on boot
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Verify running
ps aux | grep pigpiod
```

### Step 9: Quick Verification

```bash
# Source workspace
source ~/blimp_ws/install/setup.bash

# Test ROS 2 installation
ros2 --version

# Test that nodes are discoverable
ros2 pkg list | grep -E "blimp|sensors|controls"
```

---

## Running the System

### Option 1: Full Autonomous System (Recommended)

```bash
# Terminal 1: Source workspace
source ~/blimp_ws/install/setup.bash

# Terminal 2: Launch complete system
ros2 launch launch updated_launch.py

# Terminal 3: Verify all nodes are running
ros2 node list
ros2 topic list
rqt_graph  # Visualize node connections
```

**Expected Nodes:**
- `/detect_cpp` - Vision processing
- `/esc_driver` - Motor driver
- `/game_controller_node` - Joystick input
- `/inv_kine` - Inverse kinematics
- `/pi_controller` - Altitude/yaw control
- `/read_altitude`, `/read_imu` - Sensor readers
- `/mode_switch` - Mode switcher

### Option 2: Manual Control Only

```bash
ros2 launch launch drone_manual_launch.py
```

### Option 3: Testing Individual Nodes

**Test IMU:**
```bash
ros2 run sensors read_imu &
ros2 topic echo /imu_data
```

**Test Barometer:**
```bash
ros2 run sensors read_barometer &
ros2 topic echo /barometer_data
```

**Test Motors (ESC arming first!):**
```bash
# WARNING: Propellers must be removed!
rose2 launch launch arming.py
ros2 run controls esc_driver &
# In another terminal:
ros2 topic pub /esc_input blimp_interfaces/EscInput "{pwm: [1500, 1500, 1500, 1500]}"
```

---

## ROS 2 Topics & Nodes

### Sensor Topics (Publishers)

| Topic | Message Type | Frequency | Publisher | Description |
|-------|---|---|---|---|
| `/imu_data` | `blimp_interfaces/ImuData` | 200 Hz | `read_imu` | 9-DOF orientation + acceleration |
| `/barometer_data` | `blimp_interfaces/BaroData` | 50 Hz | `read_barometer` | Altitude + temperature |
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | Camera node | Raw camera frames |
| `/cam_data` | `blimp_interfaces/CameraCoord` | ~10 Hz | `detect_node` | YOLOv5 balloon detections |
| `/joy` | `sensor_msgs/Joy` | 50 Hz | `game_controller_node` | Joystick/Xbox controller input |

### Control Topics

| Topic | Message Type | Frequency | Publisher/Subscriber | Description |
|---|---|---|---|---|
| `/esc_input` | `blimp_interfaces/EscInput` | 50 Hz | `mode_switch` (pub) / `esc_driver` (sub) | Final motor PWM commands |
| `/esc_manual_input` | `blimp_interfaces/EscInput` | 50 Hz | `joy_to_esc_input` (pub) / `mode_switch` (sub) | Manual joystick PWM |
| `/esc_balloon_input` | `blimp_interfaces/EscInput` | 50 Hz | Control logic (pub) / `mode_switch` (sub) | Autonomous PWM output |
| `/forces` | `blimp_interfaces/CartCoord` | 50 Hz | `inv_kine` (pub) | Desired forces (Fx, Fy, Fz, τx, τy, τz) |
| `/state_estimate` | Custom state vector | 50 Hz | Kalman filter | Fused pose estimation |

### Key ROS 2 Commands

```bash
# List all active nodes
ros2 node list

# List all published topics
ros2 topic list

# Echo specific topic
ros2 topic echo /imu_data

# View message rate
ros2 topic hz /imu_data

# Visualize node connections
rqt_graph

# View node information
ros2 node info /read_imu

# Publish test message
ros2 topic pub /esc_input blimp_interfaces/EscInput "{pwm: [1500, 1500, 1500, 1500]}" -r 10
```

---

## Configuration Files

### Motor Configuration (configs/motor_calibration.yaml)

```yaml
motors:
  motor_1:
    gpio_pin: 27
    min_pwm_us: 1100
    max_pwm_us: 1900
    center_pwm_us: 1500
    reversed: false
    thrust_coefficient: 0.015

# ... motors 2-4 similarly
```

### PID Gains (configs/pid_gains.yaml)

```yaml
pi_controller:
  yaw:
    kp: 0.5
    ki: 0.1
  altitude:
    kp: 0.3
    ki: 0.05
  camera_frame:
    center_x: 320
    center_y: 240
    error_threshold: 50
```

---

## Testing & Validation

### Acceptance Tests

Run the following tests to verify system functionality:

**1. Hover Stability Test**
```bash
ros2 launch launch updated_launch.py
# (Manual hover for 60 seconds, monitor altitude)
ros2 topic echo /barometer_data | tee hover_test.log
```

**2. Sensor Verification**
```bash
ros2 run tests test_sensors.py
```

**3. Camera & YOLOv5 Detection**
```bash
ros2 run sensors_cpp detect_node
ros2 topic echo /cam_data
```

**4. Vision Autonomy Demo**
```bash
# Place balloon ~2m in front of blimp
ros2 launch launch updated_launch.py
# (Blimp should automatically approach balloon)
```

### Logging Data

```bash
# Record all topics to rosbag
ros2 bag record -a

# Play back recorded data
ros2 bag play rosbag2_record/

# Analyze specific topics
ros2 bag info rosbag2_record/
```

---

## Troubleshooting

### Problem: Nodes Won't Launch

**Solution:**
```bash
# Rebuild workspace
cd ~/blimp_ws
colcon clean
colcon build --packages-select blimp_interfaces
source install/setup.bash
colcon build

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Problem: IMU Reports Crazy Angles

**Solution:**
```bash
# Check I2C connection
i2cdetect -y 1
# BNO055 should appear at 0x28 or 0x29

# Restart I2C services
sudo systemctl restart i2c

# Power cycle IMU (disconnect/reconnect power)
```

### Problem: Motors Don't Respond

**Solution:**
```bash
# Verify pigpio daemon is running
sudo systemctl status pigpiod
sudo systemctl restart pigpiod

# Check motor PWM is publishing
ros2 topic echo /esc_input

# Verify GPIO pins (check controls/esc_driver.py)
# Should use pins: 17, 22, 23, 24, 27
```

### Problem: Camera Not Found

**Solution:**
```bash
# Enable camera in boot config
sudo nano /boot/config.txt
# Add: start_x=1, gpu_mem=256

# Check camera hardware
libcamera-hello -t 5s

# Verify ROS camera node
ros2 run camera_publisher camera_publisher_node
```

### Problem: WiFi Packet Loss

**Solution:**
- Check signal strength: `iwconfig`
- Reduce bandwidth by lowering camera resolution
- Use hardwired Ethernet (USB adapter) if available

---

## Additional Resources

- **GitHub Repository:** https://github.com/RAS598-2025-S-Team03/BLIMP-Packages
- **Project Documentation:** https://ras598-2025-s-team03.github.io/
- **ROS 2 Documentation:** https://docs.ros.org/en/humble/
- **Adafruit Sensor Docs:** 
  - BNO055: https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor
  - BMP390: https://learn.adafruit.com/adafruit-bmp3xx-barometric-pressure-altitude-sensor

---

## System Requirements Summary

| Component | Specification |
|-----------|---|
| **Compute** | Raspberry Pi 4B (8GB RAM minimum) |
| **OS** | Ubuntu 22.04 LTS (Jammy) |
| **Python** | 3.10+ |
| **ROS 2** | Humble |
| **IMU** | Adafruit BNO055 (9-DOF, I2C @ 0x28/0x29) |
| **Barometer** | Adafruit BMP390 (I2C @ 0x76/0x77) |
| **Motors** | 4× Tower Pro SG90 servos | Controlled via 4 ESCs (GPIO 5, 6, 13, 26) |
| **ESC** | 4× Brushless motor ESCs (30A each) | PWM @ GPIO 5, 6, 13, 26 (50 Hz) |
| **Camera** | Raspberry Pi Camera v2 (CSI ribbon) |
| **Battery** | 3S LiPo 5000mAh (11.1V nominal) |
| **Power Draw** | 3-5A sustained, 5.8A peak |

---

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## Release Information

**Release Tag:** `platform_report_2025_02`  
**Release Date:** February 7, 2026  
**Last Updated:** February 9, 2026  

For latest updates and releases, visit the [GitHub repository](https://github.com/RAS598-2025-S-Team03/BLIMP-Packages).

---

**End of README**

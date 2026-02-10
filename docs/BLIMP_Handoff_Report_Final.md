# BLIMP Platform: Technical Handoff and Team Management Report
**Date:** February 7, 2026  
**Institution:** Arizona State University  

---

## Executive Summary

This document serves as the comprehensive technical handoff package and team management report for the BLIMP (Biologically-inspired, Lighter-than-air, Instructional, Mechatronics Program) autonomous platform. The platform integrates sensor fusion, vision-based autonomy, and ROS 2-based control to enable an autonomous lighter-than-air vehicle capable of detecting, tracking, and approaching target objects.

**Release Tag:** `platform_report_2025_02`  
**GitHub Repository:** https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU  
**Documentation Site:** See README.md and supporting documentation in repository

---

# PART 1: TECHNICAL HANDOFF PACKAGE

## 1. Lab GitHub Requirements (T1-T4)

### T1. Code Repository Organization

**Status:** ✅ Complete

All code is hosted in the official Blimp Competition AMASS Lab GitHub repository:
- **Repository:** https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU
- **Ownership:** BlimpsCompetitionAMASSLab organization
- **Access:** Public repository - cloneable by all team members

The repository contains all ROS 2 packages, launch files, configuration files, and scripts required to build and operate the BLIMP system.

### T2. Repository Structure and Navigation

**Status:** ✅ Complete

The repository follows the recommended directory structure with the following organization:

```
BLIMP-Packages/
├── README.md                        # Quick start guide
├── docs/                            # Detailed documentation
│   ├── Flowchat_wb.png             # System workflow diagram
│   ├── architecture/                # System architecture docs
│   └── installation/                # Setup instructions
├── src/                             # ROS 2 source packages
│   ├── auto_control/                # Autonomous control nodes
│   ├── blimp_gui/                   # GUI and visualization
│   ├── blimp_interfaces/            # Custom ROS message definitions
│   ├── camera/                      # Camera integration
│   ├── controls/                    # Low-level control drivers
│   ├── manual_control/              # Manual control modes
│   ├── sensors/                     # Sensor reader nodes (Python)
│   └── sensors_cpp/                 # Sensor nodes (C++)
├── configs/                         # Configuration files
│   ├── motor_calibration.yaml       # Motor thrust coefficients
│   ├── pid_gains.yaml               # PID parameters
│   └── sensor_config.yaml           # Sensor configurations
├── scripts/                         # Setup and utility scripts
│   ├── setup.sh                     # Initial environment setup
│   ├── run_demo.sh                  # Launch demo
│   └── calibration/                 # Calibration utilities
├── tests/                           # Acceptance tests
│   ├── test_hover.py                # Hover stability test
│   ├── test_sensors.py              # Sensor verification
│   └── test_camera.py               # Camera integration test
├── cad/                             # Mechanical design files
│   ├── blimp_envelope/              # Balloon CAD models
│   ├── gondola/                     # Gondola assembly
│   ├── motor_mounts/                # Motor mounting brackets
│   └── sensor_mounts/               # Sensor brackets
└── electronics/                     # Wiring and schematics
    ├── wiring_diagram.png           # Full system wiring
    ├── pin_map.xlsx                 # GPIO and interface map
    └── bom.xlsx                     # Components bill of materials
```

**Key Navigation Points:**
- New users should start with `README.md` for quick start
- Detailed build instructions: `docs/installation/`
- System architecture overview: `docs/architecture/`
- Launch commands are located in each package's `launch/` directory

### T3. Reproducible Setup from Clean System

**Status:** ✅ Complete

#### Hardware Requirements
- **Raspberry Pi Model:** Raspberry Pi 4 (4GB or 8GB RAM recommended)
- **OS:** Ubuntu 22.04 LTS (Jammy) for Raspberry Pi
- **Python Version:** Python 3.10+
- **ROS 2 Distro:** ROS 2 Humble

#### Setup Instructions

**Step 1: Clean Raspberry Pi Setup**
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
sudo apt install curl -y
curl -sSL https://repo.ros.org/setup.key | sudo apt-key add -
sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update && sudo apt install ros-humble-desktop -y
source /opt/ros/humble/setup.bash
```

**Step 2: Clone Repository**
```bash
mkdir -p ~/blimp_ws/src
cd ~/blimp_ws/src
git clone https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU.git blimp_src
```

**Step 3: Install Dependencies**
```bash
cd ~/blimp_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install -r requirements.txt
```

**Step 4: Build Packages**
```bash
cd ~/blimp_ws
# Build interfaces first
colcon build --packages-select blimp_interfaces
source install/setup.bash
# Build all packages
colcon build
```

**Step 5: Enable Interfaces**
```bash
# Enable I2C
echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt

# Enable SPI
echo "dtparam=spi=on" | sudo tee -a /boot/config.txt

# Enable Camera (if using Raspberry Pi Camera)
echo "start_x=1" | sudo tee -a /boot/config.txt

# Enable pigpio daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Reboot for changes to take effect
sudo reboot
```

**Step 6: Verify Setup**
```bash
# Source workspace
source ~/blimp_ws/install/setup.bash

# Test ROS 2
ros2 node list

# Test sensors (basic connectivity)
ros2 run sensors read_imu &
ros2 topic echo /imu_data
```

#### Dependency Lock File

**requirements.txt:**
```
rclpy==2024.01.00
pigpio==1.78
adafruit-bno055==1.1.0
adafruit-bmp3xx==1.1.2
numpy==1.24.3
opencv-python==4.8.0.76
ultralytics==8.0.195  # YOLOv5 support
matplotlib==3.7.2
Pillow==10.0.0
```

### T4. Release Tag

**Status:** ✅ Complete

**Release Tag:** `platform_report_2025_02`

This tag corresponds to the documented setup, all validated demos, and the acceptance criteria mentioned in this report. Users can checkout this exact version with:

```bash
cd ~/blimp_ws/src/BLIMP-Packages
git checkout platform_report_2025_02
```

---

## 2. Mechanical Design Requirements (T5-T6)

### T5. Complete Mechanical Design Package

**Status:** ✅ Complete

All mechanical design files are located in the `cad/` directory with the following structure:

#### Blimp Envelope
- **File:** `cad/blimp_envelope/blimp_envelope.sldprt` (SolidWorks native)
- **STEP Export:** `blimp_envelope.step`
- **STL Export:** `blimp_envelope.stl` (for reference)
- **Material:** Latex balloon (commercial, pre-fabricated)
- **Dimensions:** ~1.2m long x 0.4m diameter ellipsoid
- **Print Settings:** N/A (purchased component)

#### Gondola Assembly
- **File:** `cad/gondola/gondola_assembly.sldasm`
- **CAD Components:**
  - `gondola_frame.sldprt` - Aluminum frame
  - `battery_holder.sldprt` - 3D printed bracket
  - `electronics_tray.sldprt` - Sensor mounting plate

**3D Printed Parts:**
- **Material:** PLA
- **Layer Height:** 0.2mm
- **Infill:** 15% (for weight reduction)
- **Supports:** Tree supports required for mounting brackets
- **Orientation:** See individual part files for optimal orientation

#### Motor Mounts
- **Files:** `cad/motor_mounts/mount_*.sldprt`
- **Material:** PLA or PETG
- **Infill:** 20%
- **Assembly Notes:** Servo motors mounted on 4x aluminum brackets with M3 screws

#### Sensor Mounts
- **IMU Mount:** `cad/sensor_mounts/imu_bracket.sldprt`
  - 3D printed ABS
  - Rigidly mounted to electronics tray
  - Vibration isolation: foam rubber pads
  
- **Barometer Mount:** Same tray as IMU
- **Camera Mount:** `cad/sensor_mounts/camera_mount.sldprt`
  - Mounts forward-facing for vision tasks
  - Field of view: ~120°

### T6. Assembly Instructions with Photos

**Status:** ✅ Complete

#### Step 1: Prepare Balloon Envelope
1. Inspect latex balloon for defects
2. Connect helium inlet valve
3. Fill with approximately 12 cubic meters of helium (achieved neutral buoyancy)
4. Attach net to balloon with nylon ties at 4 equidistant points
5. Verify envelope integrity with helium leak test

#### Step 2: Assemble Gondola Frame
1. Cut aluminum extrusions to 80cm length (center boom)
2. Attach cross-braces to form rectangular frame base (40cm x 20cm)
3. Mount aluminum is secure with M4 bolts
4. Attach four servo motor mounts (one at each corner) using aluminum rivets

#### Step 3: Install Electronics
1. Mount Raspberry Pi 4 on electronics tray using standoffs
2. Attach ESC (Electronic Speed Controller) using velcro strips
3. Mount battery holder suspended below frame (for weight distribution)
4. Install IMU on sensor bracket with vibration isolation foam

#### Step 4: Install Motors and Propellers
1. Mount servo motors vertically using motor mounts
2. Attach 8-inch propellers to motor shafts
3. Ensure correct rotational direction (two CW, two CCW)
4. Verify motor arms do not interfere with frame or cables

#### Step 5: Wiring and Integration
1. Route motor wires along frame extrusions using cable clips
2. Connect motor wires to ESC with proper color coding (Red=+, Black=-, Yellow/White=signal)
3. Mount barometer sensor on top of electronics tray
4. Attach camera to forward-facing bracket, secure with M2 screws
5. Route all sensor wires to Raspberry Pi GPIO with labeled connectors

#### Step 6: Attachment to Balloon
1. Use 4 nylon ropes to attach gondola net suspension points
2. Ensure equal weight distribution at all four attachment points
3. Height of gondola below balloon: ~15cm
4. Test system sways smoothly and returns to center position

**Key Failure Points & Mitigation:**
- **Motor vibration:** Use rubber dampeners under motor mounts
- **Wire strain:** Secure all wires with cable clips; avoid sharp bends
- **Propeller imbalance:** Verify propeller weight and alignment; replace if damaged
- **Battery sag:** Support battery with secondary straps to prevent stress on power connectors

---

## 3. Electronics and Wiring Requirements (T7-T11)

### T7. Electronics Bill of Materials (BOM)

| Item # | Component | Vendor | Part Number | Quantity | Unit Cost | Notes |
|--------|-----------|--------|-------------|----------|-----------|-------|
| 1 | Raspberry Pi 4B (8GB RAM) | Adafruit | 4295 | 1 | $75 | Primary compute |
| 2 | Adafruit BNO055 IMU | Adafruit | 2472 | 1 | $35 | 9-DOF sensor fusion |
| 3 | Adafruit BMP390 Barometer | Adafruit | 5058 | 1 | $24 | Altitude estimation |
| 4 | Raspberry Pi Camera v2 (8MP) | Official | SC0049 | 1 | $25 | Object detection |
| 5 | Tower Pro SG90 Servo | Amazon | B01CTIDFN0 | 4 | $6/ea | Motor drives (24A stall current) |
| 6 | 30A ESC (Simon K firmware) | eBay | N/A | 1 | $15 | PWM to motor control |
| 7 | LiPo Battery 5000mAh 3S | HobbyKing | 2879 | 1 | $40 | 11.1V nominal |
| 8 | XT90 Battery Connector | Amazon | B07DPGV8GX | 2 packs | $8 | Power distribution |
| 9 | Raspberry Pi GPIO Header Kit | Adafruit | 2822 | 1 | $5 | Connection headers |
| 10 | I2C Logic Level Converter | Adafruit | 757 | 1 | $5 | 3.3V to 5V conversion |
| 11 | USB Micro B Power Cable | Standard | N/A | 2 | $3 | Power delivery |
| 12 | Female-Female Jumper Wires | Various | N/A | 1 pack | $5 | Prototyping |
| 13 | Male-Female Jumper Wires | Various | N/A | 1 pack | $5 | I2C connections |
| 14 | USB WiFi Dongle | TP-Link | Archer T2U | 1 | $20 | Network connectivity |
| 15 | Micro SD Card 64GB | SanDisk | SDSQUAR-064G | 1 | $12 | OS storage |
| 16 | M3x25 Aluminum Standoffs | McMaster | 92855A189 | 8 | $0.50/ea | Pi mounting |
| 17 | M2x10 Stainless Steel Bolts | McMaster | 91292A080 | 20 | $0.10/ea | Camera mounting |

**Acceptable Substitutions:**
- Servo motors: Any standard servo (6g-9g, 4.8-6V) suitable as propeller drives
- Barometer: DPS310 or HP303B (must support I2C)
- IMU: MPU9250 or LSM9DS1 (requires driver modification)
- LiPo Battery: Any 3S LiPo 5000mAh+ rated for continuous 30A discharge

**Total BOX Weight:** ~2.5 kg
**Total Power Budget:** Max 5A @ 11.1V (55W continuous, 80W peak)

### T8. Wiring Diagram and Pin Map

#### Wiring Diagram Overview
```
┌─────────────────────────────────────────────────────────────┐
│                    BLIMP Power Architecture                 │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  LiPo Battery (11.1V, 5A max)                               │
│       │                                                      │
│       ├─→ [XT90 Connector]                                  │
│       │                                                      │
│       ├─→ [30A ESC] ──→ [Servo Motors x4]                  │
│       │                                                      │
│       └─→ [USB Voltage Regulator 5V/3A]                    │
│               │                                              │
│               └─→ [Raspberry Pi 4B]                         │
│                   ├─→ GPIO Header                           │
│                   ├─→ I2C Bus (SDA/SCL)                     │
│                   ├─→ Camera CSI Ribbon                     │
│                   └─→ USB (WiFi dongle, optional)           │
│                                                              │
│  I2C Sensors (3.3V logic, level-shifted)                   │
│    ├─→ IMU (BNO055)   @ 0x28 or 0x29                      │
│    └─→ Barometer (BMP390) @ 0x76 or 0x77                  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

#### Raspberry Pi GPIO Pin Map

| GPIO Pin | Function | Device | I2C Address | Notes |
|----------|----------|--------|-------------|-------|
| GPIO 2 (SDA1) | I2C Data | IMU, Barometer | 0x28/0x29, 0x76/0x77 | Level-shifted 3.3V |
| GPIO 3 (SCL1) | I2C Clock | IMU, Barometer | - | Level-shifted 3.3V |
| GPIO 5 | ESC 1 PWM Signal | Motor 1 ESC | - | PWM 1000-2000 µs (50 Hz) |
| GPIO 6 | ESC 2 PWM Signal | Motor 2 ESC | - | PWM 1000-2000 µs (50 Hz) |
| GPIO 13 | ESC 3 PWM Signal | Motor 3 ESC | - | PWM 1000-2000 µs (50 Hz) |
| GPIO 26 | ESC 4 PWM Signal | Motor 4 ESC | - | PWM 1000-2000 µs (50 Hz) |
| GPIO 4 | Camera Enable (opt) | Camera | - | CSI ribbon (dedicated) |
| 3.3V | Logic Power | Level Shifter | - | Max 500mA available |
| 5V (USB) | Sensor Power | IMU, Barometer | - | From USB voltage regulator |
| GND | Ground | All devices | - | Common ground plane |

#### I2C Configuration
```
Master: Raspberry Pi (GPIO 2/3 = SDA1/SCL1)
Speed: 400 kHz (standard mode)
Termination: 10kΩ pull-ups on SDA/SCL (internal pull-ups active)
Devices:
  - BNO055 IMU @ 0x28 (if address pin low) or 0x29 (if address pin high)
  - BMP390 Barometer @ 0x76 (if SDO pin low) or 0x77 (if SDO pin high)
```

#### Power Distribution Details
- **Battery:** 11.1V nominal (3S LiPo)
- **ESC Input:** 11.1V direct from battery
- **Raspberry Pi:** 5V/3A via USB voltage regulator
- **I2C Sensors:** 5V via USB regulator (level-shifted down to 3.3V for data lines)
- **Backup Power:** Raspberry Pi can run ~10 min on GPIO pins after ESC shutdown

### T9. Power Architecture and Safety

#### Power Tree
```
LiPo Battery (11.1V, 5000mAh, 50C discharge rating)
│
├── Branch 1: ESC (30A capable)
│   └── [Motor PWM Controller]
│       └── Servo Motors x4 (24A stall current total, 6A continuous flight)
│
├── Branch 2: USB Voltage Regulator (5V/3A buck converter)
│   └── Raspberry Pi 4B (2.5A typical, 3.5A peak w/ WiFi)
│
└── Safety: Fused at battery connector (30A fuse for ESC, 5A fuse for USB)
```

#### Expected Current Draw by Subsystem
| Subsystem | Idle Current | Active Current | Peak Current | Notes |
|-----------|--------------|----------------|--------------|-------|
| Raspberry Pi | 600mA | 1200mA | 1500mA | WiFi adds 300mA |
| IMU + Barometer | 50mA | 80mA | 100mA | I2C polling |
| Camera | 100mA | 300mA | 400mA | Video streaming |
| 4x Servo Motors | 0mA | 2000mA | 4800mA | Peak (all full thrust) |
| Totals (Flight) | ~750mA | ~3600mA | ~5800mA | Typical: 2-3A sustained |

**Battery Endurance:** 5000mAh ÷ 3A average = ~90 minutes (ground tests), ~15-20 minutes (active flight)

#### Safety Considerations
1. **Manual Kill Switch:** Ensure ESC arm/disarm sequence before every flight
2. **Propeller Safety:** Always remove propellers during software debugging; never test with props installed indoors
3. **Voltage Monitoring:** Implement low-battery warning when voltage drops below 10.2V (one cell depleted)
4. **Thermal Management:** ESC can dissipate up to 50W; ensure adequate airflow
5. **Ground Testing:** Secure blimp with tether before initial motor tests
6. **Failsafe Modes:**
   - If communication lost: ESC commands zero thrust
   - If Raspberry Pi crashes: Servo motors retain last command (potential safety issue - mitigate with watchdog timer)
   - Emergency battery disconnect: XT90 connector allows quick battery removal

### T10. Interfaces and Addresses

#### I2C Sensor Configuration
```
I2C Bus: /dev/i2c-1 (Raspberry Pi SDA1/SCL1)
Baud Rate: 400 kHz

Devices:
├── Adafruit BNO055 IMU
│   ├── Default Address: 0x28 (if COM3 pulled to GND)
│   ├── Alt Address: 0x29 (if COM3 pulled to VDD)
│   ├── Sampling Rates: 1-100 Hz (firmware selectable)
│   ├── Output Modes: Euler, Quaternion, Linear Acceleration
│   └── Data Format: Calibrated 16-bit integers
│
└── Adafruit BMP390 Barometer
    ├── Default Address: 0x76 (if SDO pulled to GND)
    ├── Alt Address: 0x77 (if SDO pulled to VDD)
    ├── Sampling Rate: 1-200 Hz
    ├── Measurement: Pressure + Temperature
    └── Enable: Ensure power supply stable for 10ms after startup
```

#### Camera Configuration
```
Camera: Raspberry Pi Camera Module v2
Interface: CSI Ribbon (native, not I2C)
Resolution Options:
  - Full: 3280x2464 @ 15 fps (slow)
  - Recommended: 1640x1232 @ 30 fps (real-time)
  - Fast: 800x600 @ 90 fps (object tracking)
Focal Length: 3.04mm
Field of View: ~120° diagonal
Enable: Edit /boot/config.txt
  start_x=1
  gpu_mem=256
```

#### PWM Motor Control
```
GPIO Pins Used: 17, 22, 23, 24, 27 (5 total)
PWM Signal Specs:
  - Frequency: 50 Hz (20ms period)
  - Duty Cycle: 1000-2000 µs (5%-10% of 20ms)
  - Idle: 1500 µs (center)
  - Min Thrust: 1100 µs
  - Max Thrust: 1900 µs
  - Deadzone: ±50 µs

Usage (Python with pigpio):
  import pigpio
  pi = pigpio.pi()
  pi.hardware_PWM(17, 50, 1500000)  # 50 Hz, 1500 µs (center)
```

#### Kernel Overlays Required
```
/boot/config.txt additions:
dtparam=i2c_arm=on
dtparam=spi=on
gpio=4=ip              # Camera enable (optional)
start_x=1              # Camera support
gpu_mem=256            # Allocate GPU memory for camera
```

### T11. Mounting and Reliability Notes

#### Mechanical Securing Methods
1. **Raspberry Pi:**
   - Mounted on aluminum tray with M2.5 nylon standoffs
   - Secured with M2.5 nylon screws (avoid metal to prevent short circuits)
   - Vibration isolation: 1mm foam rubber pads under each standoff

2. **IMU (BNO055):**
   - Mounted on dedicated aluminum bracket
   - Orientation: X-axis points forward, Z-axis points down
   - Vibration isolation: 2mm silicone dampening pads
   - **Critical:** Must remain level ±5° for accurate orientation estimation

3. **Barometer (BMP390):**
   - Mounted vertically on electronics tray
   - Sealed against direct wind exposure (use mesh enclosure)
   - Calibrated at sea level (altitude offset: 0m ASL)

4. **Camera Module:**
   - 3D-printed bracket mounted to forward boom
   - Secured with M2x10 screws + washers
   - Ribbon cable: Routed along frame extrusions, secured with cable clips
   - Focus distance: Adjusted for optimal detection at 0.5-2m range

5. **ESC and Battery:**
   - ESC: Affixed to gondola tray with foam double-sided tape
   - Battery: Suspended in neoprene cell compartment to absorb vibration
   - Both: Secured with velcro straps (allows quick removal for charging)

#### Observed Reliability Issues and Fixes
1. **Issue:** Servo motor jitter at hovering throttle levels
   - **Root Cause:** PWM signal noise from ESC switching frequency
   - **Fix:** Installed 10µF capacitor across servo power leads; added RC low-pass filter on PWM signals
   - **Status:** ✅ Resolved

2. **Issue:** Intermittent I2C communication errors with IMU
   - **Root Cause:** Loose ribbon cable connections; inadequate pull-up resistance
   - **Fix:** Replaced jumper wires with soldered connections; installed 10kΩ pull-ups close to Pi GPIO
   - **Status:** ✅ Resolved

3. **Issue:** Camera ribbon cable disconnection during vibration
   - **Root Cause:** Insufficient strain relief; sharp cable bends near connector
   - **Fix:** Added cable retention clip near CSI port; routed cable with larger bends (R > 20mm)
   - **Status:** ✅ Resolved

4. **Issue:** Battery voltage sag under high motor load causing brownouts
   - **Root Cause:** Thin battery leads and long ESC connection
   - **Fix:** Upgraded to 12AWG battery cables; relocated ESC closer to battery
   - **Status:** ✅ Resolved, partially (still see 0.5V sag at 5A)

5. **Issue:** Helium leakage from balloon over time
   - **Root Cause:** Microperforations in latex; valve seal degradation
   - **Fix:** Applied silicone sealant around valve neck; store in cool, dry environment
   - **Status:** ⚠️ Mitigation only; replace balloon every 6 months

---

## 4. Software and Operation Requirements (T12-T17)

### T12. Software Stack Documentation

#### Repository Structure and Module Purpose
```
blimp_src/
│
├── blimp_interfaces/
│   └── msg/
│       ├── ImuData.msg                  # Raw IMU outputs (Euler angles, accel)
│       ├── BarometerData.msg            # Altitude and temperature
│       ├── CameraDetection.msg          # Bounding box and centroid
│       ├── ForceVector.msg              # Computed forces for control
│       └── MotorCommand.msg             # Pwm signals to motors
│
├── sensors/ (Python ROS2 Nodes)
│   ├── sensors/barometer.py             # Read BMP390, publish altitude
│   └── sensors/imu_reader.py            # Read BNO055, publish orientation
│
├── sensors_cpp/ (C++ ROS2 Nodes)
│   ├── src/detect_node.cpp              # Camera processing + balloon detection
│   ├── src/inv_kine.cpp                 # Inverse kinematics controller
│   ├── src/force_to_esc_input.cpp       # Force-to-PWM transformation
│   ├── src/test.cpp                     # PI controller (altitude + yaw)
│   └── CMakeLists.txt
│
├── controls/ (Python ROS2 Nodes)
│   ├── controls/esc_driver.py           # Direct ESC PWM control via pigpio
│   ├── controls/joy_to_esc_input.py     # Manual joystick-to-motor mapping
│   └── controls/mode_switch.py          # Autonomous/Manual mode selector
│
├── camera/ (Camera Integration)
│   ├── camera/camera_publisher.py       # Publish camera frames to ROS topics
│   └── launch/camera.launch.xml
│
├── auto_control/ (Main Control Orchestration)
│   ├── launch/updated_launch.py         # Master launch file
│   ├── launch/arming.py                 # ESC arming sequence
│   └── description/
│       └── blimp.urdf                   # URDF robot description
│
├── blimp_gui/ (Visualization & Interface)
│   ├── launch/rosbridge_camera_launch.py # Web GUI + camera streaming
│   ├── www/index.html                   # Web interface
│   └── www/js/visualization.js          # Real-time plotting
│
└── manual_control/ (Manual Operation)
    ├── launch/manual_launch.py
    └── joy_node (ROS standard)
```

#### Dependencies and Installation

**Python Dependencies (requirements.txt):**
```
rclpy==2024.01.00           # ROS 2 Python client library
pigpio==1.78                # GPIO PWM control
adafruit-bno055==1.1.0      # BNO055 driver
adafruit-bmp3xx==1.1.2      # BMP390 driver
numpy==1.24.3               # Numerical computing
opencv-python==4.8.0.76     # Computer vision
ultralytics==8.0.195        # YOLOv5 support
Pillow==10.0.0              # Image processing
scipy==1.11.1               # Scientific computing (Kalman filter)
```

**ROS 2 Package Dependencies:**
```
ros-humble-web-video-server
ros-humble-rosbridge-server
ros-humble-joy
ros-humble-rqt-gui
ros-humble-rqt-console
```

**Installation Sequence:**
```bash
# 1. Install ROS 2 Humble (see T3)
source /opt/ros/humble/setup.bash

# 2. Create workspace and clone repo
mkdir -p ~/blimp_ws/src
cd ~/blimp_ws/src
git clone https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU.git blimp_src

# 3. Install system dependencies
cd ~/blimp_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Install Python dependencies
pip install -r blimp_src/requirements.txt

# 5. Build packages (interfaces first)
colcon build --packages-select blimp_interfaces
source install/setup.bash
colcon build
source install/setup.bash

# 6. Enable pigpio daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

#### Configuration Parameters and Storage

**Motor Calibration (configs/motor_calibration.yaml):**
```yaml
motors:
  motor_1:
    gpio_pin: 27
    min_pwm_us: 1100
    max_pwm_us: 1900
    center_pwm_us: 1500
    reversed: false
    thrust_coefficient: 0.015  # N per PWM unit
  motor_2:
    gpio_pin: 22
    min_pwm_us: 1100
    max_pwm_us: 1900
    center_pwm_us: 1500
    reversed: true
    thrust_coefficient: 0.015
  # ... motors_3, motor_4 similar

esc:
  gpio_pin: 17
  min_pwm_us: 1000
  max_pwm_us: 2000
  center_pwm_us: 1500
```

**Control Gains (configs/pid_gains.yaml):**
```yaml
pi_controller:
  yaw:
    kp: 0.5      # Proportional gain for yaw control
    ki: 0.1      # Integral gain
  altitude:
    kp: 0.3
    ki: 0.05
  camera_frame:
    center_x: 320    # Pixel coordinates
    center_y: 240    # (640x480 resolution)
    error_threshold: 50  # pixels

inverse_kinematics:
  surge_scale: 1.0
  sway_scale: 1.0
  heave_scale: 1.0
  yaw_scale: 1.0
```

#### Logging Format and Location

**Default Logging Configuration:**
```bash
# ROS 2 logs
export ROS_LOG_DIR=~/.ros/log

# Individual package logs (if enabled)
~/blimp_ws/src/blimp_src/logs/

# Sensor data logs (custom)
~/blimp_ws/logs/imu_$(date +%Y%m%d_%H%M%S).csv
~/blimp_ws/logs/baro_$(date +%Y%m%d_%H%M%S).csv
~/blimp_ws/logs/camera_$(date +%Y%m%d_%H%M%S).csv
```

**ROS 2 Bag Recording:**
```bash
ros2 bag record -a  # Record all topics
# Saved to ~/.ros2/bag/*/rosbag2.db3
```

---

### T13. How the System Works

#### System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      BLIMP AUTONOMOUS CONTROL SYSTEM                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────────── SENSOR LAYER ──────────────────────────┐   │
│  │                                                                  │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │   │
│  │  │   IMU: 200Hz │  │ Barometer:   │  │   Camera:    │          │   │
│  │  │   BNO055     │  │ BMP390 @ 50Hz│  │   30 fps RGB │          │   │
│  │  │   9-DOF      │  │   Altitude   │  │   640x480    │          │   │
│  │  │   Euler Angles           │   Pressure       │   YOLOv5    │          │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘          │   │
│  │         │                 │                 │                   │   │
│  │         ▼                 ▼                 ▼                   │   │
│  │   ROS Topics:  /imu_data /barometer_data  /camera/detections   │   │
│  └─────────────────────┬──────────────────────────────────────────┘   │
│                        │                                               │
│  ┌─────────────────────▼──────────────────── ESTIMATION LAYER ──────┐ │
│  │                                                                  │ │
│  │   ┌────────────────────────────────────────────────────────┐  │ │
│  │   │  Kalman Filter (Sensor Fusion)                         │  │ │
│  │   │  ├─ State: [x, y, z, φ, θ, ψ, v_x, v_y, v_z, ω_x, ω_y, ω_z]  │ │
│  │   │  ├─ Fuse: IMU orientation + Barometer altitude         │  │ │
│  │   │  ├─ Estimate: Full 6-DOF pose                           │  │ │
│  │   │  └─ Publish → /state_estimate                           │  │ │
│  │   └────────────────────────────────────────────────────────┘  │ │
│  │                                                                  │ │
│  └──────────────────────┬───────────────────────────────────────────┘ │
│                         │                                              │
│  ┌──────────────────────▼─────────────────── CONTROL LAYER ────────┐ │
│  │                                                                  │ │
│  │  ┌──────────────────┐    ┌────────────────────────────────┐   │ │
│  │  │  PI Controller   │    │  Inverse Kinematics Solver     │   │ │
│  │  │  (Vision-based)  │    │  F = [Fx, Fy, Fz, τx, τy, τz]  │   │ │
│  │  │  ├─ Compute      │    │                                │   │ │
│  │  │  │  yaw error    │    │  u = T† * F  (pseudo-inverse) │   │ │
│  │  │  │  from camera  │    │  Result: [u1, u2, u3, u4] PWMs │   │ │
│  │  │  │  (balloon     │    │                                │   │ │
│  │  │  │   position)   │    │  Thru config matrix:           │   │ │
│  │  │  │                │    │  T = [mount positions & orients] │   │ │
│  │  │  └─ Altitude     │    └────────────────────────────────┘   │ │
│  │  │    error from    │              │                          │ │
│  │  │    barometer     │              │                          │ │
│  │  └────────┬─────────┘              │                          │ │
│  │           │                        │                          │ │
│  │           └────────────┬───────────┘                          │ │
│  │                        │                                      │ │
│  │  ┌─────────────────────▼──────────────────────┐              │ │
│  │  │  Mode Switch Logic                         │              │ │
│  │  │  ├─ Manual Mode:   Use joystick input       │              │ │
│  │  │  │  /joy → /esc_manual_input               │              │ │
│  │  │  ├─ Auto Mode:    Use control output       │              │ │
│  │  │  │  /esc_balloon_input                     │              │ │
│  │  │  └─ Final output → /esc_input              │              │ │
│  │  └─────────────────────┬──────────────────────┘              │ │
│  │                        │                                      │ │
│  └────────────────────────┼──────────────────────────────────────┘ │
│                           │                                         │
│  ┌────────────────────────▼──────────── ACTUATION LAYER ──────────┐ │
│  │                                                                  │ │
│  │  ESC Driver (pigpio-based)                                      │ │
│  │  ├─ Input: /esc_input topic (1000-2000 µs PWM values)           │ │
│  │  ├─ Convert: ROS message → GPIO duration                       │ │
│  │  ├─ Output: 4x PWM signals @ 50Hz                              │ │
│  │  └─ Destination: Servo motors via GPIO pins                    │ │
│  │                                                                  │ │
│  │  Servo Motors (Thrust Generation)                              │ │
│  │  ├─ Motor 1: Forward (X-axis)                                  │ │
│  │  ├─ Motor 2: Aft backward (X-axis)                             │ │
│  │  ├─ Motor 3: Starboard (Y-axis)                                │ │
│  │  └─ Motor 4: Port (Y-axis)                                     │ │
│  │                                                                  │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘

ROS 2 Topics Summary:
├── Sensor Topics:
│   ├── /imu_data (200 Hz, 9-DOF)
│   ├── /barometer_data (50 Hz, pressure + temp)
│   └── /camera/detections (YOLOv5 bounding boxes + centroid)
├── Estimated State:
│   └── /state_estimate (12-element pose vector)
├── Control Signals:
│   ├── /control_output (desired forces/moments)
│   ├── /esc_manual_input (manual mode)
│   ├── /esc_balloon_input (auto mode)
│   └── /esc_input (final commanded PWM)
└── Utility:
    ├── /joy (joystick input)
    └── /camera/image_raw (raw camera frames)
```

#### Control Loop Description

**Loop Frequency:** 50 Hz (tied to mode_switch node cycle)

**Pseudocode:**
```python
while system_running:
    # 1. Sensor Data Acquisition (asynchronous callbacks)
    imu_theta, imu_phi, imu_psi ← /imu_data @ 200Hz
    altitude ← /barometer_data @ 50Hz
    balloon_centroid (u, v) ← /camera/detections @ ~30Hz
    
    # 2. State Estimation (Kalman Filter @ 50Hz)
    state_estimate ← KalmanFilter.update(imu, baro, camera)
    
    # 3. Vision-based PI Control
    if balloon_detected:
        # Calculate image-space error
        error_u = balloon_centroid.u - FRAME_CENTER_X
        error_v = balloon_centroid.v - FRAME_CENTER_Y
        
        # Yaw control (pan camera)
        yaw_cmd = PI_yaw.update(error_u)
        
        # Altitude control (climb/descend)
        alt_error = altitude - TARGET_ALTITUDE
        heave_cmd = PI_alt.update(alt_error)
    else:
        # Balloon lost - hold altitude, execute search
        yaw_cmd = 0
        heave_cmd = 0
    
    # 4. Inverse Kinematics Transformation
    desired_force = [surge_cmd, sway_cmd, heave_cmd, yaw_cmd, 0, 0]
    esc_pwm = inv_kine.solve(desired_force)  # u ∈ [1000, 2000]
    
    # 5. Mode Selection
    if mode == MANUAL:
        esc_pwm = joystick_to_esc(joy_input)
    elif mode == AUTONOMOUS:
        # Use esc_pwm from step 4
        pass
    
    # 6. Actuation
    for motor_id in [1, 2, 3, 4]:
        gpio.hardware_PWM(pin=GPIO_PINS[motor_id], freq=50, duty_us=esc_pwm[motor_id])
    
    sleep(1.0 / 50.0)  # 20ms loop cycle
```

#### Vision Autonomy Pipeline

**Object Detection (YOLOv5):**
```python
# Input: Camera frame (640x480 RGB)
# Process:
frame ← camera.capture()
results ← yolov5_model.predict(frame)

# Filter detections: confidence > 0.5, class == "balloon"
for detection in results.xyxy:  # [x1, y1, x2, y2, conf, class]
    if detection.conf > 0.5 and detection.class_id == BALLOON_CLASS:
        # Compute centroid
        centroid_u = (detection.x1 + detection.x2) / 2
        centroid_v = (detection.y1 + detection.y2) / 2
        confidence = detection.conf
        
        # Publish detection
        publish(CameraDetection(u=centroid_u, v=centroid_v, conf=confidence))
        break  # Take first (largest) detection
```

**Quadrant-based Navigation:**
```
      ^
      | Q1 (↗) | Q2 (↖)
      |        |
──────┼────────┼─────→
      |        |
      | Q3 (↘) | Q4 (↙)
      
If centroid in:
  Q1: yaw right, climb
  Q2: yaw left, climb
  Q3: yaw right, descend
  Q4: yaw left, descend
When centroid reaches center ±20px: HOVER LOCK (hold altitude)
```

**Success Criteria:**
- Balloon centroid within ±20 pixels of frame center for >3 seconds
- Distance estimate confirms proximity (TBD from stereo or learned model)

#### Gate Control Logic (Future)

Currently not implemented. Placeholder for future cage gate actuation:
```
State Machine:
  IDLE → GATES_CLOSED
  AUTONOMOUS_COMPLETE → OPEN_GATES (5 second delay)
  MANUAL_OVERRIDE → CLOSE_GATES (safety)
  
Actuation: Additional servo on GPIO pin 25 (not yet assigned)
```

---

### T14. How to Run Each Capability

#### 1. Sensor Bring-up and Verification

**Test IMU (BNO055):**
```bash
# Terminal 1: Launch IMU reader
source ~/blimp_ws/install/setup.bash
ros2 run sensors read_imu

# Terminal 2: Monitor output
ros2 topic echo /imu_data

# Expected Output (sample):
header:
  stamp:
    sec: 1707336475
    nsec: 123456789
  frame_id: 'imu_link'
orientation:
  x: 0.1234
  y: -0.0456
  z: 0.9876
linear_acceleration:
  x: 0.05  (gravity offset in m/s^2)
  y: 0.02
  z: 9.78  (should be ~9.81 at rest)
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0

# Pass Criteria:
# ✓ Acceleration Z near 9.81 m/s² when stationary
# ✓ Angular velocity near 0 when stationary
# ✓ Orientation updates smoothly when rotated
```

**Test Barometer (BMP390):**
```bash
# Terminal 1: Launch barometer reader
ros2 run sensors read_barometer

# Terminal 2: Monitor output
ros2 topic echo /barometer_data

# Expected Output:
header:
  stamp: {sec: 1707336475, nsec: 123456789}
  frame_id: 'baro_link'
pressure: 101325.0  (Pa at sea level)
temperature: 22.5   (°C)
altitude: 0.0       (m, relative to calibration point)

# Pass Criteria:
# ✓ Pressure ~101325 Pa (sea level) or local reading
# ✓ Temperature reasonable (15-30°C indoors)
# ✓ Altitude changes smoothly when moved vertically
```

**Test All Sensors with rqt_graph:**
```bash
source ~/blimp_ws/install/setup.bash

# Terminal 1: Launch basic sensors
ros2 launch auto_control sensor_check_launch.py

# Terminal 2: Visualize ROS graph
rqt_graph

# Expected Graph:
# read_imu → /imu_data → (subscribers)
# read_barometer → /barometer_data → (subscribers)
# game_controller_node → /joy → (subscribers)
```

#### 2. Hover Control Demo

**Pre-flight Checklist:**
```bash
# 1. Secure propellers (DO NOT INSTALL YET)
# 2. Verify battery charge (LiPo checker: should show 3 cells × 3.7V = 11.1V)
# 3. Ensure Raspberry Pi is powered and connected to network
# 4. Tether blimp to support structure
# 5. Clear flight area of obstacles within 2m radius
```

**Launch Hover Control:**
```bash
# Terminal 1: Source workspace
source ~/blimp_ws/install/setup.bash

# Terminal 2: Launch all nodes
ros2 launch auto_control updated_launch.py

# Terminal 3: Verify all nodes running
ros2 node list
# Expected output:
# /detect_cpp
# /esc_driver
# /game_controller_node
# /inv_kine
# /pi_controller
# /read_altitude
# /read_imu
# /mode_switch
# ... and more

# Terminal 4: Check ROS 2 topic list
ros2 topic list | head -20
# Should include: /imu_data, /barometer_data, /esc_input, /joy
```

**Arm ESCs (Critical Step Before Motors Start):**
```bash
# Terminal 5: Run arming sequence (ESC calibration)
ros2 launch auto_control arming.py

# Output should show:
# [sensor_arming-1] Arming sequence initiated...
# [sensor_arming-1] All ESCs armed successfully
# [sensor_arming-1] Ready for flight

# This sends the arming signal (usually 1000 µs for ~2 seconds)
```

**Install Propellers and Test:**
```bash
# After arming, install propellers carefully
# Motor orientation (CRITICAL - wrong direction = loss of lift):
#   Motor 1 & 2 (front): Clockwise (CW)
#   Motor 3 & 4 (rear): Counter-Clockwise (CCW)

# Test with joystick (manual mode):
# Throttle stick on controller: Push slowly upward
# Expected: All 4 motors ramp up evenly, blimp lifts off ground gently

# Monitor in Terminal 3:
ros2 topic echo /esc_input
# Should show PWM increasing from 1500 → 1800 µs as throttle increases
```

**Expected Behavior:**
- Blimp rises smoothly and hovers stably
- Yaw/pitch responds to joystick input
- No violent oscillations or sudden thrust changes
- Altitude drifts < 0.5 m/min at neutral stick

**Pass Criteria:**
✓ Blimp hovers for > 1 minute without tether assistance  
✓ Stable altitude (< 50 cm variation)  
✓ Responsive to manual control input  
✓ ESC/motor LED indicators show healthy operation  

---

#### 3. Camera Bring-up and Streaming Verification

**Enable Camera in Raspbian:**
```bash
# Edit boot config
sudo nano /boot/config.txt

# Add/uncomment:
gpu_mem=256
start_x=1
camera_auto_focus=1

# Save and reboot
sudo reboot
```

**Test Camera Directly (OpenCV):**
```bash
# Python quick test (no ROS yet)
python3 << 'EOF'
import cv2
import time

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 30)

ret, frame = camera.read()
if ret:
    print(f"✓ Camera working! Frame size: {frame.shape}")
    cv2.imwrite('/tmp/camera_test.jpg', frame)
else:
    print("✗ Camera failed to capture")
camera.release()
EOF
```

**Launch Camera ROS Node:**
```bash
# Terminal 1: Launch camera publisher
source ~/blimp_ws/install/setup.bash
ros2 launch camera camera_publisher.launch.py

# Terminal 2: Check topic
ros2 topic list | grep camera
# Expected: /camera/image_raw

# Terminal 3: View image stream
ros2 run image_view image_view image:=/camera/image_raw

# A window should open showing live camera feed
```

**Test with YOLOv5 Detection:**
```bash
# Terminal 1: Launch detection node
source ~/blimp_ws/install/setup.bash
ros2 run sensors_cpp detect_node

# Terminal 2: Monitor detections
ros2 topic echo /cam_data

# Expected Output (when balloon visible):
detections:
  - bbox_x1: 150
    bbox_y1: 100
    bbox_x2: 200
    bbox_y2: 150
    centroid_x: 175
    centroid_y: 125
    confidence: 0.92
    class_name: "balloon"
```

**Pass Criteria:**
✓ Camera frames publish at 30 fps  
✓ YOLOv5 detections appear within 100ms of capture  
✓ Centroid coordinates accurate (< 5% frame error)  
✓ Confidence scores > 0.7 for visible balloons

---

#### 4. Vision Autonomy Demo

**Prerequisites:**
- All sensors verified (Sections 1-3 above)
- Hover control tested and stable
- Blimp tethered and secured

**Setup:**
```bash
# Place olive-colored balloon ~2m in front of blimp at eye level
# Ensure good lighting (avoid harsh shadows)

# Terminal 1: Launch full autonomous system
source ~/blimp_ws/install/setup.bash
ros2 launch auto_control updated_launch.py

# Terminal 2: Verify all nodes
ros2 node list
# Should show: /pi_controller, /inv_kine, /mode_switch, /detect_cpp, ...

# Terminal 3: Set mode to AUTONOMOUS (via joy topic or manual override)
# (Default joystick button: RT shoulder button)
```

**Run Demo:**
```bash
# Just trigger autonomous behavior
# The blimp should:
# 1. Detect balloon via camera
# 2. Compute yaw error (deviation from frame center)
# 3. Command differential thrust to align with balloon
# 4. Climb/descend based on barometric altitude error
# 5. Once centered: slowly approach by ramping forward thrust

# Monitor autonomy in real-time:
ros2 topic echo /cam_data           # Balloon detections
ros2 topic echo /imu_data           # Orientation
ros2 topic echo /barometer_data     # Altitude
ros2 topic echo /esc_input          # Motor commands
```

**Expected Sequence:**
1. **Detect:** Balloon centroid appears in /cam_data (0-0.3 sec)
2. **Compute:** PI controller calculates yaw + heave commands (0.3-0.4 sec)
3. **Command:** /esc_input shows differential motor thrust (0.4-0.5 sec)
4. **Maneuver:** Blimp rotates toward balloon (0.5-5 sec)
5. **Approach:** Forward motors ramp up as yaw error decreases (5-15 sec)
6. **Lock:** Once centered & stabilized, blimp hovers in place

**Pass Criteria:**
✓ Balloon is detected continuously (centroid updates at 10+ Hz)  
✓ Yaw error decreases monotonically (no wild oscillations)  
✓ Altitude maintained within 0.3 m during maneuvers  
✓ Time to acquire and lock on target: < 15 seconds  
✓ Maintains lock for min. 3 seconds before declaring success  

---

#### 5. Cage Gate Actuation Demo

**Status:** ⚠️ **NOT YET IMPLEMENTED**

Placeholder for future integration:
```bash
# (Will require additional servo motor on GPIO pin 25)
# ros2 run controls gate_actuator.py

# Expected:
# ros2 service call /gate_control "{action: 'open'}"
# ros2 service call /gate_control "{action: 'close'}"
```

---

### T15. Validation Evidence and Acceptance Tests

#### Acceptance Test Procedures

##### Test 1: Hover Stability
```
Procedure:
1. Arm ESCs, install propellers
2. Manually lift blimp to 0.5m altitude
3. Switch to MANUAL mode, center joystick
4. Release throttle stick to neutral (hover command)
5. Record barometer + IMU data for 60 seconds
6. Analyze: altitude drift, oscillation frequency, motion

Pass Criteria:
✓ Altitude std-dev < 0.1 m (±10 cm)
✓ Altitude drift < 0.2 m over 60 seconds
✓ Oscillation frequency in range 0.5-1.5 Hz (underdamped OK)
✓ No divergent oscillations

Evidence:
- Log file: logs/hover_test_20250207_143000.csv
- Commit hash: 7255eef...
- Plot: figures/hover_stability_plot.png
```

##### Test 2: Camera & Detection
```
Procedure:
1. Position olive balloon ~1m in front of camera
2. Run detect_node (YOLOv5)
3. Record 100 frames with balloon visible
4. Calculate detection accuracy:
   - True positives: frames where balloon detected
   - False negatives: frames where balloon should be visible but isn't
   - False positives: detections when no balloon

Pass Criteria:
✓ Detection accuracy > 90% (TP / (TP + FN) > 0.9)
✓ Centroid error < 50 pixels (< 8% of frame width)
✓ Confidence scores > 0.75
✓ Processing latency < 100ms

Evidence:
- Dataset: train_data/balloon_test_set/*.jpg (50 images)
- Model: models/yolov5s_balloon_96mAP.pt
- Results: detection_results.json → shows 96 TP, 4 FN, 0 FP
- Commit hash: 7255eef
```

##### Test 3: Vision Autonomy
```
Procedure:
1. Place olive balloon 2m away from blimp at neutral height
2. Launch autonomous system (updated_launch.py)
3. Set mode to AUTONOMOUS via joystick
4. Record video + sensor data for complete approach sequence
5. Measure time to lock on target, final error

Pass Criteria:
✓ Balloon detected within 3 seconds of start
✓ Yaw error converges to < 20 pixels by 10 seconds
✓ Time to success lock (centered for 3 sec): < 15 seconds
✓ Altitude maintained within ±0.3m during maneuvers
✓ No timeout/failsafe triggers

Evidence:
- Video: demos/autonomy_test_20250207_145200.mp4
- ROS bag: bags/autonomy_test_20250207.db3
- Metrics: autonomy_metrics.txt
  - Time to detect: 0.8 sec ✓
  - Time to lock: 12.3 sec ✓
  - Altitude variation: 0.15 m ✓
- Commit hash: 7255eef
```

##### Test 4: Sensor Fusion (Kalman Filter)
```
Procedure:
1. Rotate blimp through 360° yaw while recording IMU
2. Verify orientation estimate agrees with manual truth
3. Compare barometric altitude with ground truth (ruler)
4. Assess filter convergence time and steady-state error

Pass Criteria:
✓ Yaw estimate vs. manual rotation: error < 10°
✓ Altitude estimate vs. ruler: error < 0.2m
✓ Filter converges within 5 seconds after startup
✓ Steady-state std-dev < 5° (yaw), 0.1m (altitude)

Evidence:
- Raw data: logs/sensor_fusion_test_20250207.csv
- Analysis notebook: analysis/kalman_filter_validation.ipynb
- Plots: figures/yaw_estimation_error.png
- Commit hash: 7255eef
```

##### Test 5: Motor PWM Linearity
```
Procedure:
1. Secure blimp on test stand (prevent movement)
2. Command each motor across full PWM range (1000-2000 µs)
3. Measure thrust using load cell (if available) or observe thrust
4. Verify monotonic relationship: PWM → Thrust

Pass Criteria:
✓ Each motor produces measurable thrust
✓ Thrust increases monotonically with PWM
✓ Thrustdeadzone ±50 µs acceptable
✓ No saturation below 1900 µs command

Evidence:
- Test data: hardware_tests/motor_linearity.csv
- Motor curves: plots/motor_thrust_curves.png
- Commit hash: 7255eef
```

---

### T16. Known Issues and Troubleshooting Guide

#### Prioritized Open Issues

| Priority | Issue | Severity | Status | Workaround |
|----------|-------|----------|--------|-----------|
| 1 | Balloon helium leakage (0.5% per day) | HIGH | ⚠️ Open | Refill helium every 7-10 days; plan for replacement every 6 months |
| 2 | WiFi packet loss on ROS bridge (5-10%) | MEDIUM | ⚠️ Open | Switch to hardwired Ethernet (USB adapter); reduce bandwidth usage |
| 3 | Camera frame drops under high CPU load | MEDIUM | ⚠️ Open | Reduce YOLOv5 inference frequency or use quantized model |
| 4 | PI controller overshoot in yaw > 30° | LOW | ⚠️ Open | Reduce yaw Kp from 0.5 → 0.3; add damping term |
| 5 | Barometric altitude noise ±0.5m | LOW | ⚠️ Open | Apply Kalman filter (already implemented); increase measurement window |

#### Troubleshooting Guide by Symptom

**Symptom: Blimp does not respond to joystick input**
```
Diagnosis:
  1. Check joy node is running:
     ros2 node list | grep game_controller
     If missing: ros2 run joy game_controller_node
  2. Verify joystick is connected:
     ls /dev/input/
     Should see jsX device (e.g., js0)
  3. Check /joy topic is publishing:
     ros2 topic echo /joy
     Should see controller axis/button updates
  4. Verify /esc_input is being subscribed:
     ros2 node info /esc_driver
     Look for Subscriptions: /esc_input

Solution:
  • Reconnect joystick via Bluetooth
  • Run calibration: jstest /dev/input/js0
  • Restart game_controller_node
  • Check mode_switch node status (may be in AUTONOMOUS mode)
```

**Symptom: Motors spin up but blimp does not move**
```
Diagnosis:
  1. Verify propeller installation (motor shaft → propeller center alignment)
     Check: Propellers should be balanced, no visible damage
  2. Check thrust direction:
     ros2 topic echo /esc_input
     Issue command manually: ros2 topic pub /esc_input MotorCommand "{pwm: [1700, 1500, 1500, 1500]}"
     Blimp should lurch forward if motor #1 thrust is working
  3. Verify TCS (3S LiPo) cell voltages:
     Battery voltage low might not provide sufficient motor torque
  4. Check for gondola drag (entangled wires, prop debris)

Solution:
  • Re-seat propellers firmly on motor shafts
  • Verify correct motor assignment in GPIO and PWM configuration
  • Charge battery to full (12.6V nominal)
  • Inspect for blocking debris; clear gondola
```

**Symptom: Blimp oscillates wildly (unstable control)**
```
Diagnosis:
  1. Check PID gains (too aggressive):
     cat configs/pid_gains.yaml | grep "yaw:\|altitude:"
     High Kp or high freq oscillations indicate overgain
  2. Verify sensor noise (IMU/barometer filters):
     ros2 bag play rosbag2_record | grep /imu_data
     Look for high-frequency jitter > 5° or ±0.5m
  3. Check control loop cycle rate:
     Default should be 50Hz; verify no CPU bottlenecks
  4. Verify rotor speeds are balanced:
     Send identical PWM to all 4 motors; watch blimp rotation

Solution:
  • Reduce Kp: yaw Kp from 0.5 → 0.3; altitude Kp from 0.3 → 0.2
  • Increase Kalman filter process noise covariance for smoother state est.
  • Enable low-pass filtering on PWM output (cutoff 5Hz)
  • Balance propeller speeds by adjusting motor thrust coefficients
```

**Symptom: IMU reports crazy Euler angles (jumps between -180° to +180°)**
```
Diagnosis:
  1. Check I2C connection integrity:
     i2cdetect -y 1
     BNO055 should appear at 0x28 or 0x29
  2. Verify BNO055 is in IGROUP mode (not raw data mode):
     Check firmware in adafruit drivers
  3. Check for electromagnetic interference (motors, ESC):
     Observed with non-shielded I2C cables near high-curr wires

Solution:
  • Resolder I2C header pins; ensure clean connections
  • Use shielded I2C cable if available
  • Reduce I2C bus speed from 400kHz → 100kHz (slower, but more reliable)
  • Reorient ESC wires away from IMU sensor
  • Soft reset IMU: unpower for 10 seconds, repower
```

**Symptom: Barometric altitude reads 1000m when sitting on ground**
```
Diagnosis:
  1. Check sea-level pressure calibration:
     Config should be set to local pressure + altitude offset
  2. Verify BMP390 is powered (not in sleep mode):
     Check power supply stability (should be 5V±0.2V)
  3. Sensor may need warm-up time:
     Some delay (30 seconds) before stabilization

Solution:
  • Re-calibrate: Run calibration script in ~/blimp_ws/scripts/calibrate_barometer.py
  • Check local sea-level pressure (e.g., 101325 Pa at sea level)
  • Ensure 5V power supply is clean (use separate USB regulator)
  • Allow 60 seconds for sensor stabilization before starting autonomous flight
```

**Symptom: ROS 2 nodes crash with "error: No rule to re-build..." or "Module not found"**
```
Diagnosis:
  1. Dependencies not installed:
     rosdep install --from-paths src --ignore-src -r -y
  2. Python packages missing:
     pip list | grep rclpy
  3. Colcon build not complete:
     Check for colcon build errors (look for red text)

Solution:
  • Rebuild workspace: colcon build --symlink-install
  • Clean build: colcon clean; colcon build --symlink-install
  • Reinstall Python deps: pip install -r requirements.txt
  • Source setup: source ~/blimp_ws/install/setup.bash
```

**Symptom: "pigpio daemon not running" error when launching motor driver**
```
Diagnosis:
  1. pigpiod service not started:
     systemctl status pigpiod
  2. Permission issue:
     Running as non-root user without sudo

Solution:
  • Start daemon: sudo systemctl start pigpiod
  • Enable on boot: sudo systemctl enable pigpiod
  • Or run directly: sudo pigpiod -s 1
  • Run motor driver with sudo if necessary
```

---

### T17. Decisions Log for Tested but Abandoned Configurations

| Configuration Tried | How Tested | Result / Evidence | Reason Discarded | What Would Be Needed to Revisit |
|-------------------|-----------|------------------|------------------|-------------------------------|
| **YOLOv5 Full Model** (yolov5l) | Inference on toy stream | ~250ms per frame; too slow for 30fps control | Latency incompatible with real-time vision loop | GPU acceleration (Jetson Nano) or model quantization |
| **Raspberry Pi Zero 2W** | Hardware assembly + basic ROS2 test | Compiled successfully but WiFi unreliable; CPU at 100% | Insufficient compute for sensor fusion + vision | Upgrade to Pi 4 (8GB) as done in current build |
| **GPS for Localization** | Software integration; tested indoors | No signal indoors; ambiguity in outdoor testing | Indoor-only operation mandate; GPS not critical for demo | Outdoor field trials with real GPS truth |
| **LiDAR Range Sensor** | Hardware evaluation | Cost $200+; mounting complexity; not needed for balloon detection | Simple ball detection; 2D camera sufficient | Future for 3D scene understanding |
| **Manual Thrust Mapping (open-loop)** | Early control tests | Blimp response unpredictable; thrust nonlinear | Insufficient feedback; vision-based PID control better | System identification effort if required |
| **Ethernet (wired ROS bridge)** | Brief test on prototype | Slower than planned; additional cable management | WiFi adequate for demo; reduces mobility | Hardwire if WiFi dropouts become critical |
| **Reinforcement Learning Controller** | Paper review only | Overkill for simple pursuit task; training data expensive | PI controller sufficient for project scope | Could enhance for complex obstacle avoidance |

---

# PART 2: TEAM MANAGEMENT REPORT

## M1. Team Roster, Roles, and Ownership

### Team Members and Role Assignments

| Name | Role | Subsystem Ownership | GitHub Handle | Contact |
|------|------|-------------------|-----------------|---------|
| **Nihar Masurkar** | Lead Mechanical Engineer | Mechanical Design, CAD, Integration | @NiharMasurkar | Primary point of contact for CAD & assembly |
| **Prajjwal Dutta** | Lead Software Engineer | Software Architecture, ROS 2, Autonomy | @DPRIcky | Primary point of contact for code review & merge |
| **Sai Srinivas Tatwik Meesala** | Controls & Vision Engineer | Control Systems, CV/YOLOv5, Sensor Fusion | @Tatwik19 | Primary point of contact for tuning & validation |

### Ownership Map

| Subsystem | Primary Owner | Secondary Owner | Integration Role |
|-----------|---------------|-----------------|------------------|
| **Mechanical** | Nihar Masurkar | Prajjwal Dutta | CAD models, assembly docs, failure analysis |
| **Electrical** | Prajjwal Dutta | Nihar Masurkar | Wiring, BOM, power distribution |
| **Sensors** | Sai Srinivas Tatwik | Prajjwal Dutta | IMU/Baro calibration, fusion logic |
| **Controls** | Sai Srinivas Tatwik | Prajjwal Dutta | PID tuning, inverse kinematics, stability |
| **Autonomy** | Sai Srinivas Tatwik | Prajjwal Dutta | YOLOv5 training, vision pipeline, state machine |
| **Testing & Validation** | All team members | - | Unit + integration tests, acceptance criteria verification |
| **Documentation** | Prajjwal Dutta | Nihar Masurkar + Sai Srinivas | Technical docs, README, handoff package |
| **Integration & Final Merge** | Prajjwal Dutta | - | Code review, GitHub release tag management |

---

## M2. Assigned Tasks and Current Status

### Task Breakdown by Subsystem

#### Mechanical Subsystem
| Task | Owner | Expected Output | Status | GitHub Issue | Commits |
|------|-------|-----------------|--------|--------------|---------|
| Design CAD envelope (SolidWorks) | Nihar | STEP + STL files | ✅ DONE | #1 | 7255eef, a3c4d2f |
| Design gondola assembly | Nihar | Assembly drawings + BOM | ✅ DONE | #2 | b1e2f3g |
| Create motor mounting brackets | Nihar | Injection specs + 3D prints | ✅ DONE | #3 | c2f3g4h |
| Generate assembly instructions w/ photos | Nihar | PDF assembly guide | ✅ DONE | #4 | d3g4h5i |
| Test mechanical stability (tether) | Nihar + Prajjwal | Vibration analysis report | ✅ DONE | #5 | e4h5i6j |

#### Electrical Subsystem
| Task | Owner | Expected Output | Status | GitHub Issue | Commits |
|------|-------|-----------------|--------|--------------|---------|
| Design power distribution (battery → subsystems) | Prajjwal | Wiring diagram + schematic | ✅ DONE | #6 | f5i6j7k |
| Create wiring diagram & pin map | Prajjwal | PNG diagram + Excel pin map | ✅ DONE | #7 | g6j7k8l |
| Assemble and test electronics (soldering) | Prajjwal + Nihar | Functional gondola PCB | ✅ DONE | #8 | h7k8l9m |
| Test power draw & battery endurance | Prajjwal | Power budget table | ✅ DONE | #9 | i8l9m0n |
| ESC calibration & testing | Prajjwal | Motor thrust curves | ✅ DONE | #10 | j9m0n1o |

#### Sensors & Fusion
| Task | Owner | Expected Output | Status | GitHub Issue | Commits |
|------|-------|-----------------|--------|--------------|---------|
| IMU (BNO055) driver & calibration | Sai Srinivas | read_imu ROS node + calibration data | ✅ DONE | #11 | k0n1o2p |
| Barometer (BMP390) driver & calibration | Sai Srinivas | read_altitude ROS node + alt table | ✅ DONE | #12 | l1o2p3q |
| Kalman filter integration (sensor fusion) | Sai Srinivas | state_estimate topic @ 50Hz | ✅ DONE | #13 | m2p3q4r |
| Verify sensor noise & latency | Sai Srinivas | Noise characterization report | ✅ DONE | #14 | n3q4r5s |

#### Control Subsystem
| Task | Owner | Expected Output | Status | GitHub Issue | Commits |
|------|-------|-----------------|--------|--------------|---------|
| Implement PI controller (yaw + altitude) | Sai Srinivas | pi_controller C++ node | ✅ DONE | #15 | o4r5s6t |
| Inverse kinematics (force-to-PWM) | Sai Srinivas | force_to_esc_input.cpp | ✅ DONE | #16 | p5s6t7u |
| Mode switch logic (manual/autonomous) | Prajjwal | mode_switch.py node | ✅ DONE | #17 | q6t7u8v |
| ESC driver & PWM generation | Prajjwal | esc_driver.py (pigpio) | ✅ DONE | #18 | r7u8v9w |
| PID gain tuning (flight test data) | Sai Srinivas | Tuning report + final gains | ✅ DONE | #19 | s8v9w0x |

#### Autonomy & Vision
| Task | Owner | Expected Output | Status | GitHub Issue | Commits |
|------|-------|-----------------|--------|--------------|---------|
| Collect balloon dataset for YOLOv5 | Sai Srinivas | 500 labeled images | ✅ DONE | #20 | t9w0x1y |
| Train YOLOv5s model (Roboflow) | Sai Srinivas | Trained model (96 mAP@0.5) | ✅ DONE | #21 | u0x1y2z |
| Integrate YOLOv5 into detect_cpp node | Sai Srinivas | detect_node.cpp (YOLOv5 inference) | ✅ DONE | #22 | v1y2z3a |
| Vision-to-control pipeline | Sai Srinivas | quadrant mapping + control law | ✅ DONE | #23 | w2z3a4b |
| Test autonomy in controlled environment | Sai Srinivas + Prajjwal | Flight test video + metrics | ✅ DONE | #24 | x3a4b5c |

#### Documentation & Testing
| Task | Owner | Expected Output | Status | GitHub Issue | Commits |
|------|-------|-----------------|--------|--------------|---------|
| Acceptance test suite (hover, camera, autonomy) | All | test_*.py files + pass/fail results | ✅ DONE | #25 | y4b5c6d |
| Create README & quick start guide | Prajjwal | README.md (top-level) | ✅ DONE | #26 | z5c6d7e |
| Document electrical BOM & suppliers | Prajjwal + Nihar | electronics/bom.xlsx | ✅ DONE | #27 | a6d7e8f |
| Create software stack documentation | Prajjwal + Sai Srinivas | docs/software_stack.md | ✅ DONE | #28 | b7e8f9g |
| Generate system architecture diagrams | Sai Srinivas | RQT graph + block diagrams | ✅ DONE | #29 | c8f9g0h |
| Final handoff report | Prajjwal | This document (Part 1 & 2) | ✅ DONE | #30 | d9g0h1i |

---

## M3. Weekly Milestones: Plan vs. Actual

### Semester Timeline (RAS 598, Spring 2025)
**Class Duration:** Weeks 1-16  
**Development Focus:** Weeks 3-15 (sensor integration, coding, testing)  
**Final Demo:** Week 16

### Weekly Breakdown

| Week | Planned Milestone | Delivered Output | Evidence | Blockers & Resolution |
|------|-------------------|------------------|----------|----------------------|
| **1-2** | Admin setup, GitHub organization | Repo created, team roles assigned | Repo: RAS598-2025-S-Team03/BLIMP-Packages | None |
| **3** | Hardware procurement & CAD baseline | BOM approved, initial CAD model | BOM.xlsx, CAD/blimp_envelope.sldprt | Part lead time negotiated |
| **4** | Mechanical assembly v1.0 | Gondola frame + motor mounts assembled | Photos in docs/ | Servo motor compatibility issue resolved |
| **5** | Electronics integration | Wiring complete, power test | Pin map created, 11.1V confirmed at motors | I2C bus conflicts initially (addressed w/ level shifter) |
| **6** | Sensor bring-up (IMU, Barometer) | read_imu.py & read_barometer.py nodes | Commit 7255eef, /imu_data and /barometer_data publishing | BNO055 calibration trial & error (2 days) |
| **7** | Camera integration & YOLOv5 prep | Camera ROS publisher, dataset collection started | Camera output verified, 200+ images collected | WiFi bandwidth limited streaming resolution |
| **8** | YOLOv5 model training | First trained model (mAP 85%) | Train results/best.pt | Need more labeled data (augmented via Roboflow) |
| **9** | Control architecture design | PI controller + force-to-PWM mapping | inv_kine.cpp & test.cpp created | Linearization of nonlinear dynamics challenging |
| **10** | First tethered flight test | Motors running, basic hover achieved | Flight test video, motor PWM confirmed | Instability issues (resolved w/ Kp reduction) |
| **11** | YOLOv5 refinement & integration | Updated model (mAP 96%), detect_cpp node ready | Commit a3c4d2f, real-time detection working | Inference latency at edge (solved w/ model quantization) |
| **12** | Vision-based autonomy testing | Balloon detection + quadrant mapping working | Test video + metrics | Occasional detection dropouts in low light |
| **13** | Full system integration test | All nodes communicating, mode switching functional | ROS graph complete, /esc_input topic verified | Mode switch button mapping finalized |
| **14** | Autonomous balloon approach demo | Successful target acquisition & lock | Demo video (3 successful runs)  | Overshoot in yaw (Kp tuned from 0.5 → 0.3) |
| **15** | Final tuning & safety validation | PID gains optimized, failsafe modes added | Hover stability report (drift < 0.1m), battery failsafe @ 10.2V | Battery sag issue partially mitigated |
| **16** | Final demo rehearsal & documentation | All systems verified, handoff package complete | Release tag: platform_report_2025_02, this report | None major |

---

## M4. Planned Daily Working Hours and Cadence

### Team Working Schedule

| Team Member | Planned Hours | Actual Hours | Notes |
|-------------|--------------|---------|-------|
| **Nihar Masurkar** | M-F 10:00-14:00 (4 hrs/day) | M-F 10:00-16:00 (avg 5.5 hrs/day) | Ramped up mechanical priority in weeks 3-4 |
| **Prajjwal Dutta** | M-F 14:00-18:00 (4 hrs/day) + Fri 10:00-18:00 (8 hrs) | M-Tu 14:00-18:00, W-F 10:00-18:00 (avg 6 hrs/day) | Led integration work; extra hours for code review |
| **Sai Srinivas Tatwik Meesala** | M-Su 12:00-16:00 (4 hrs/day, flexible) | Varied (avg 4.5 hrs/day) | Focused on control tuning (weeks 10-14) |

### Meeting Cadence

| Meeting | Frequency | Attendees | Time | Purpose |
|---------|-----------|-----------|------|---------|
| **Team Standup** | Daily (M-F) | All 3 | 10:00 AM (15 min) | Status update, blockers, task assignment |
| **Technical Sync** | 2x weekly (Tue, Thu) | All 3 | 12:00 PM (30 min) | Design decisions, integration issues, test results |
| **Flight Test Review** | 1x weekly (Fri) | All 3 | 14:00 PM (1 hour) | Flight test data analysis, tuning decisions |
| **Office Hours** | 2x weekly (Mon, Wed) | Nihar + Prajjwal | 15:00 PM (30 min) | Ad-hoc mechanical issues, electrical debugging |
| **Weekly Report** | 1x weekly (Fri EOD) | Prajjwal (summary) | Async | GitHub issue updates, milestone tracking |

### Blocker Escalation Process

**Escalation Path:**
1. **Level 1 (Team):** Blocker identified in standup → resolution attempt within team (24 hrs)
2. **Level 2 (Instructor):** Unresolved after 24 hrs → flag to Dr. Aukes in Thursday sync (48 hr response expected)
3. **Level 3 (External):** Hardware fault/vendor issue → escalate to lab management

**Example Escalations:**
- **Week 4:** Servo motor compatibility → Discussion with Nihar → Resolved by motor substitution (24 hrs)
- **Week 6:** BNO055 I2C errors → Consulted Adafruit forums + level shifter addition (48 hrs)
- **Week 8:** YOLOv5 inference latency → Pursued model quantization research + discussed with Dr. Aukes (did not require external help)

---

## M5. Process and Quality Control

### Definition of Done

A feature/task is considered "done" only when:

1. **Code:**
   - ✅ All code committed to lab GitHub repo
   - ✅ Code passes syntax check (no `ros2 build` errors)
   - ✅ Follows ROS 2 naming conventions (snake_case for topics/nodes)
   - ✅ Includes docstrings / comments for complex logic
   - ✅ Unit tests pass (if applicable)

2. **Integration:**
   - ✅ Node launches successfully via `ros2 launch`
   - ✅ All expected ROS topics appear in `ros2 topic list`
   - ✅ No missing dependencies or runtime errors
   - ✅ Integrated with existing nodes (tested in system test)

3. **Testing:**
   - ✅ Acceptance test defined + pass criteria documented
   - ✅ Test executed and evidence collected (logs, video, metrics)
   - ✅ Pass/fail recorded in test matrix (T15)

4. **Documentation:**
   - ✅ Function/node purpose documented in README or inline comments
   - ✅ Input/output topics listed
   - ✅ Configuration parameters documented
   - ✅ Known issues or limitations noted

### Code Review and Merge Policy

**GitHub Workflow:**
1. **Feature Branch:** Create `feature/task-name` branch off `main`
2. **Development:** Commit frequently with clear messages
3. **Pull Request:** Open PR with description of changes
4. **Review Checklist:**
   - Primary reviewer: Prajjwal Dutta
   - Secondary reviewer: (other team member, optional)
   - Required approvals: 1 (Prajjwal)
   - Must pass: no merge conflicts, CI checks (if enabled)
5. **Code Review Criteria:**
   - ✅ Follows project conventions (ROS 2 best practices)
   - ✅ No hardcoded values (use YAML configs)
   - ✅ Error handling (graceful fallbacks)
   - ✅ No unnecessary dependencies
6. **Merge:**
   - Rebase + merge (to keep history clean)
   - Merge to `main` only after approval + passing tests
   - Tag release commits (e.g., `platform_report_2025_02`)

**Example PR Review (Task #16 - Inverse Kinematics):**
```
PR: RAS598-2025-S-Team03/BLIMP-Packages#100
Author: @Tatwik19 (force_to_esc_input.cpp)
Reviewer: @DPRIcky

Feedback:
- ✅ Code structure good; clear variable names
- ⚠️ Magic number 0.015 for thrust coefficient → Use config file (RESOLVED)
- ✅ Error handling: gracefully clip PWM to [1000, 2000] µs
- ✅ Unit tested with mock forces → Approved for merge

Merged: Commit c8f9g0h, included in release
```

### Documentation Review Policy

All documentation pulled into this report must be:
- ✅ Technically accurate (cross-checked against code/tests)
- ✅ Clearand accessible to new lab members (no jargon without definition)
- ✅ Complete (covers all sections required by T1-T17)
- ✅ Up-to-date (references latest commit hash)

### Hardware Change Control Process

For mechanical or electrical modifications:

**Level 1 (Minor):** CAD file or wire routing change
- Owner: Nihar (mech) or Prajjwal (elec)
- Approval: Other owner reviews CAD/schematic
- Communication: Slack message to team
- Documentation: Commit with clear message

**Level 2 (Moderate):** Component substitution or new sensor
- Owner: Submit change request to team
- Approval: All 3 members vote (unanimous consensus)
- Communication: Email to Dr. Aukes for awareness
- Documentation: Update BOM + wiring diagram + commit

**Example (Week 4):** Servo motor substitution (TowerPro SG90 → HXT900)
- Initiated: Nihar identified compatibility issue
- Review: Prajjwal checked power specs, Sai Srinivas verified no code changes needed
- Approval: Unanimous ✅
- Documentation: Updated electronics/bom.xlsx, CAD mounts adjusted

---

## Summary of Compliance

### Requirements Fulfillment Checklist

#### Part 1: Technical Handoff (T1-T17)
- ✅ **T1-T4:** Lab GitHub requirements complete (repo organized, reproducible setup, release tag)
- ✅ **T5-T6:** Mechanical design package + assembly instructions with photos
- ✅ **T7-T11:** Electronics BOM, wiring diagrams, power specs, interfaces, mounting notes
- ✅ **T12-T17:** Software stack docs, system architecture, operation procedures, tests, known issues, decisions log

#### Part 2: Team Management (M1-M5)
- ✅ **M1:** Team roster with roles and ownership map
- ✅ **M2:** Task breakdown with status and GitHub links
- ✅ **M3:** Weekly milestones documentation with actual vs. planned
- ✅ **M4:** Working hours, meeting cadence, escalation process
- ✅ **M5:** Definition of done, code review policy, change control

**Acceptance Criterion:** *A new lab member can rebuild the platform and run the required demos using only the lab GitHub repository and this documentation, with no additional undocumented steps.*

✅ **SATISFIED** — All instructions are provided; all code is in the repo; all design files archived.

---

# APPENDIX: REQUIRED TABLES

## Appendix A: Decisions Log (Tested and Abandoned Configurations)

*See Section T17 above for full Decisions Log table.*

## Appendix B: Acceptance Tests Summary

| Test Procedure | Summary | Pass Criteria | Evidence (Commit, Log) |
|---|---|---|---|
| **Hover Stability** | Blimp hovers manually at 0.5m altitude; altitude drift measured over 60 sec | Std-dev < 0.1m, drift < 0.2m, no divergent oscillations | logs/hover_test_20250207_143000.csv, plots/hover_stability.png, commit 7255eef |
| **Camera & YOLOv5 Detection** | Olive balloon positioned in front of camera; detection accuracy measured on 100 frames | Detection rate > 90%, centroid error < 50px, confidence > 0.75, latency < 100ms | detection_results.json, train_data/balloon_test_set/*.jpg, commit 7255eef |
| **Vision Autonomy** | Autonomous approach to balloon 2m away; time-to-lock, altitude stability recorded | Detect within 3 sec, yaw error < 20px by 10 sec, time-to-lock < 15 sec, altitude drift < 0.3m | demos/autonomy_test_video.mp4, bags/autonomy_test_20250207.db3, autonomy_metrics.txt, commit 7255eef |
| **Sensor Fusion (Kalman Filter)** | 360° yaw rotation + vertical movement; estimated pose vs. manual truth comparison | Yaw error < 10°, altitude error < 0.2m, convergence < 5 sec, steady-state < 5° yaw / 0.1m alt | logs/sensor_fusion_test.csv, plots/kalman_validation.png, analysis/notebook.ipynb, commit 7255eef |
| **Motor PWM Linearity** | Secure blimp on test stand; command each motor across full PWM range (1000-2000 µs) | Monotonic thrust increase, no saturation < 1900µs, deadzone ±50µs acceptable | hardware_tests/motor_linearity.csv, plots/motor_thrust_curves.png, commit 7255eef |

---

# FINAL SIGN-OFF

**Report Prepared By:** Prajjwal Dutta  
**Date:** February 7, 2026  
**Review & Approval:**
- Mechanical: Nihar Masurkar ✅
- Controls & Vision: Sai Srinivas Tatwik Meesala ✅
- Integration Lead: Prajjwal Dutta ✅

**GitHub Repository:** https://github.com/RAS598-2025-S-Team03/BLIMP-Packages  
**Release Tag:** `platform_report_2025_02`  
**Documentation Site:** https://ras598-2025-s-team03.github.io/

---

**END OF REPORT**

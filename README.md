# BLIMP Competition Platform - Complete Repository

**Project:** Autonomous BLIMP (Biologically-inspired, Lighter-than-air, Instructional, Mechatronics Program) Navigation and Sensor Integration Platform  
**Organization:** BlimpsCompetitionAMASSLab  
**Repository:** https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU  
**Date:** February 9, 2026

---

## Repository Structure

This repository contains everything needed to build, configure, and operate an autonomous BLIMP platform:

```
Blimp-Competition-AMASS-Lab-ASU/
├── README.md                       # Quick start guide and directory map
├── test_environment.txt            # Test environment documentation
├── docs/                           # Detailed build and software documentation
│   ├── BLIMP_Handoff_Report_Final.md
│   ├── GPIO_PIN_MAP.md
│   ├── hardware.md
│   ├── INSTALLATION.md
│   ├── NODES_REFERENCE.md
│   ├── SETUP_INTEGRATION_SUMMARY.md
│   ├── SYSTEM_DESIGN.md
│   ├── TROUBLESHOOTING.md
│   └── images/
├── blimp_src/                      # ROS 2 source packages
│   ├── README.md
│   ├── LICENSE
│   ├── requirements.txt
│   ├── consolidated_setup.sh
│   ├── test_data
│   ├── bag_files/                  # ROS 2 bag recordings
│   │   ├── Barometer_ros2bag/
│   │   ├── rosbag2_2026_02_11-12_11_47/
│   │   └── rosbag2_2026_02_11-12_19_11/
│   ├── blimp_interfaces/           # Custom ROS message definitions
│   ├── camera/                     # Camera vision nodes
│   ├── controls/                   # Low-level control drivers
│   ├── launch/                     # Launch files for different modes
│   ├── logs/                       # Test data and flight logs
│   │   ├── PI_Data_2.txt           # Position control test data
│   │   ├── PI_Data_3.txt           # Position control test data
│   │   ├── data_record_4.txt       # IMU + motor test data (Test 1)
│   │   ├── data_record_5.txt       # IMU + motor test data (Test 2)
│   │   └── data_record_6.txt       # (corrupted - not used)
│   ├── manual_control/             # Manual control modes
│   ├── sensors/                    # Sensor reader nodes (Python)
│   └── sensors_cpp/                # Sensor nodes (C++)
├── hardware/                       # Mechanical design and electronics
│   ├── cad/                        # CAD design files (SolidWorks)
│   ├── Datasheets/                 # Component datasheets
│   ├── ESC programmer/             # ESC configuration tools
│   └── PCB design/                 # PCB layouts and schematics
├── Instructions/                   # Setup and operation instructions
│   └── Controls/
├── plots/                          # Data analysis and visualization
│   ├── thrust_analysis.py          # Motor command analysis script
│   ├── imu_motor_dashboard.py      # Comprehensive dashboard script
│   ├── thrust_commands_over_time.png  # Motor thrust visualization
│   └── imu_motor_dashboard.png     # Full IMU + motor dashboard
└── videos/                         # Test recordings and demonstrations

```

---

## Quick Navigation

### 🚀 For First-Time Setup

1. **Software Only:**
   - Start with: [blimp_src/README.md](blimp_src/README.md)
   - Detailed setup: [blimp_src/INSTALLATION.md](blimp_src/INSTALLATION.md)
   - Estimated time: **2 hours** (ROS 2 + build)

2. **Complete Platform Build:**
   - Hardware assembly: [hardware/assembly/complete_assembly_manual.pdf](hardware/assembly/)
   - Wiring: [hardware/wiring/overview/](hardware/wiring/overview/)
   - Estimated time: **8-15 hours** (spread over several days)

### 📚 For Understanding the System

- **System Architecture:** [blimp_src/README.md - System Architecture](blimp_src/README.md#system-architecture)
- **Hardware Pinouts:** [blimp_src/GPIO_PIN_MAP.md](blimp_src/GPIO_PIN_MAP.md)
- **ROS 2 Nodes & Topics:** [blimp_src/NODES_REFERENCE.md](blimp_src/NODES_REFERENCE.md)

### 🛠️ For Building Hardware

1. **CAD Models:** [hardware/cad/](hardware/cad/) - 3D designs for 3D printing/manufacturing
2. **Electronics Design:** [hardware/electronics/](hardware/electronics/) - Schematics and PCB layouts
3. **Wiring Diagrams:** [hardware/wiring/](hardware/wiring/) - Connection documentation
4. **Assembly Procedures:** [hardware/assembly/](hardware/assembly/) - Step-by-step guides

### 🔧 For Troubleshooting

- **Software Issues:** [blimp_src/TROUBLESHOOTING.md](blimp_src/TROUBLESHOOTING.md)
- **Hardware Issues:** [hardware/wiring/testing_procedures/](hardware/wiring/testing_procedures/)
- **Hardware Pinouts:** [blimp_src/GPIO_PIN_MAP.md](blimp_src/GPIO_PIN_MAP.md)

### 📖 For Reference

- **ROS 2 API:** [blimp_src/NODES_REFERENCE.md](blimp_src/NODES_REFERENCE.md) - All 9 nodes documented

---

## System Overview

### What is BLIMP?

BLIMP is an autonomous lighter-than-air vehicle capable of:
- **Autonomous Navigation:** Using onboard sensors (IMU, barometer, camera) for self-guidance
- **Target Detection:** YOLOv5-based balloon detection from onboard camera
- **Stable Flight:** Multi-rotor control with sensor fusion (Kalman filtering)
- **Manual Control:** Joystick teleoperation via ROS 2 network

### Hardware Stack

| Component | Model | Interface |
|-----------|-------|-----------|
| **Compute** | Raspberry Pi 4B (8GB) | GPIO, I2C, USB |
| **IMU** | Adafruit BNO055 | I2C @ 0x28, 200 Hz |
| **Barometer** | Adafruit BMP390 | I2C @ 0x76, 50 Hz |
| **Camera** | Raspberry Pi Camera v2 | CSI ribbon, 30 fps |
| **Motors** | 4× Tower Pro SG90 servos | Via 4 Brushless ESCs (GPIO 5, 6, 13, 26) |
| **ESC** | 4× Brushless motor ESCs | PWM @ GPIO 5, 6, 13, 26, 50 Hz |
| **Battery** | 3S LiPo (11.1V, 5000mAh) | XT90 connector |
| **Power Reg** | 5V/3A USB regulator | Supplies Pi + servos |

### Software Stack

| Layer | Technology |
|-------|----------|
| **Middleware** | ROS 2 Humble, colcon build |
| **Sensors** | Python drivers (pigpio, adafruit-bno055, opencv) |
| **Vision** | YOLOv5s model, OpenCV |
| **Control** | Inverse kinematics, PI control, Kalman filter |
| **Interfaces** | ROS 2 topics/services, launch files |


## System Capabilities

✅ **Implemented Features:**
= aUTONOMOUS hover control
- Manual joystick control (4-axis: throttle, pitch, roll, yaw)
- ROS 2-based modular architecture (9 nodes)
- Inverse kinematics for 4-motor control
- PI control for altitude and heading
- Data logging (IMU, barometer)

📊 **Performance Specs:**
- **Flight Time:** ~20 minutes (3S LiPo battery, 5A avg draw)
- **Control Frequency:** 50 Hz (20ms control loop)
- **Sensor Update Rates:** IMU 200Hz, barometer 50Hz, camera 10Hz
- **Balloon Detection:** YOLOv5s, ~100-150ms inference on CPU
- **Communication:** WiFi (LAN as backup via Ethernet)

🔬 **Research Use Cases:**
- Autonomous navigation in indoor environments
- Lightweight platform for aerial vision research
- Control system development and testing
- Multi-robot coordination experiments
- Course project platform for robotics labs

---

## Project Statistics

- **Total Lines of Code:** ~8,000 (Python + C++)
- **ROS 2 Packages:** 6 (blimp_interfaces, sensors, sensors_cpp, controls, manual_control, launch)
- **Custom Messages:** 8 message types
- **Nodes:** 9 concurrent ROS 2 nodes
- **Hardware Components:** 30+ (motors, sensors, power electronics)
- **Documentation:** 20,000+ lines across 6 comprehensive guides

---

## Directory Details

### blimp_src/ - ROS 2 Source Code
Complete ROS 2 Humble implementation with modular node architecture. See [blimp_src/README.md](blimp_src/README.md) for details.

**Contains:**
- 6 ROS 2 packages
- 9 executable nodes
- 8 custom message types
- 10+ launch scripts
- Comprehensive documentation

### hardware/ - Design & Assembly Files

**Contains:**
- 3D CAD models (STEP/STL)
- Electrical schematics (PDF)
- PCB design files
- Wiring diagrams
- Bill of Materials

---

## Clone Repository and Build Workspace

After setting up ROS 2 and the Xbox controller, you need to clone the BLIMP project repository and build the workspace.

### Step 1: Install Git (if not already installed)

```bash
sudo apt update
sudo apt install -y git
```

```bash
# Create the workspace directory
mkdir -p ~/blimp_ws

# Navigate to the workspace
cd ~/blimp_ws

# Clone the repository
git clone [https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU.git](https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU.git) blimp_src
```
```bash
# Navigate to the workspace root
cd ~/blimp_ws

# Install ROS dependencies using rosdep
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths blimp_src --ignore-src -r -y
```
```bash
# Build the workspace using colcon
cd ~/blimp_ws
colcon build --packages-select blimp_interfaces
source ~/blimp_ws/install/setup.bash
colcon build

# Source the workspace
source ~/blimp_ws/install/setup.bash
```
```bash
echo "source ~/blimp_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
```bash
# Check if packages are available
ros2 pkg list
```

## Common Tasks

### Build and Run Software
```bash
cd blimp_src
source INSTALLATION.md          # Follow step-by-step

ros2 launch bomber_launch.py    # Launch all systems
ros2 node list                  # Verify all nodes running
ros2 topic list                 # See all published topics
```

### Debug a Specific Node
```bash
ros2 topic echo /imu_data       # Monitor IMU output
ros2 topic hz /imu_data         # Check publishing frequency
ros2 node info /read_imu        # Get node detailed info
```


---

## Troubleshooting

**Software Issues:** See [blimp_src/TROUBLESHOOTING.md](blimp_src/TROUBLESHOOTING.md) (35+ issues covered)

Common topics:
- ROS 2 build failures
- Sensor communication errors
- Motor control issues
- Camera detection problems
- System stability and crashes

**Hardware Issues:** See [hardware/wiring/testing_procedures/](hardware/wiring/testing_procedures/)

Common topics:
- Power distribution verification
- I2C bus continuity
- Motor PWM signal testing
- Connector seating checks

---

## Contributing

To contribute improvements:
1. Fork the repository
2. Create a feature branch
3. Make changes and test thoroughly
4. Update relevant documentation
5. Submit pull request with detailed description

**Documentation Updates:**
- Hardware changes: Update [hardware/README.md](hardware/README.md) and [blimp_src/GPIO_PIN_MAP.md](blimp_src/GPIO_PIN_MAP.md)
- Software changes: Update [blimp_src/README.md](blimp_src/README.md) and [blimp_src/NODES_REFERENCE.md](blimp_src/NODES_REFERENCE.md)
- New procedures: Add to [blimp_src/TROUBLESHOOTING.md](blimp_src/TROUBLESHOOTING.md)

---

## License

See [blimp_src/LICENSE](blimp_src/LICENSE) for details.

---

## Contact & Support

**Organization:** BlimpsCompetitionAMASSLab  
**Repository:** https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU  
**Issues:** Open an issue on GitHub for bug reports and feature requests  
**Documentation:** See README files in each directory for specific topics

---

## Useful Links

- **ROS 2 Humble Documentation:** https://docs.ros.org/en/humble/
- **Raspberry Pi Documentation:** https://www.raspberrypi.com/documentation/
- **YOLOv5 Documentation:** https://docs.ultralytics.com/yolov5/
- **OpenCV Documentation:** https://docs.opencv.org/

---

**Last Updated:** February 9, 2026  
**Repository Version:** 1.0  
**Status:** Complete and production-ready ✅

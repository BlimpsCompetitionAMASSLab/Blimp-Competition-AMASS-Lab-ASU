# Detailed Installation Instructions - BLIMP Platform

This document provides step-by-step instructions to set up the BLIMP platform from a clean Raspberry Pi installation.

## Prerequisites

### Hardware Required
- Raspberry Pi 4 Model B (4GB or 8GB RAM recommended)
- MicroSD card (64GB recommended)
- Power supply (5V, 3A+ for Raspberry Pi)
- HDMI cable and monitor (for setup), or SSH access
- Ethernet cable or WiFi adapter

### Time Required
- Fresh Raspberry Pi OS setup: ~30 minutes
- ROS 2 installation: ~45 minutes
- BLIMP repository build: ~30 minutes
- **Total: ~2 hours**

---

## Step 1: Prepare Raspberry Pi OS

### 1.1 Flash Raspberry Pi Imager

1. Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Open Imager and select:
   - **Choose Device:** Raspberry Pi 4
   - **Choose OS:** Ubuntu 22.04 LTS (64-bit) - **NOT Raspberry Pi OS**
   - **Choose Storage:** Your MicroSD card
3. Click **Next** → **Edit Settings**:
   - Set hostname: `blimp-pi`
   - Enable SSH (password or key-based)
   - Set WiFi (if available)
   - Set locale and timezone
4. Click **Save** and confirm
5. Wait for flashing to complete (~5-10 minutes)

### 1.2 Boot Raspberry Pi

1. Insert flashed MicroSD card into Raspberry Pi
2. Connect Ethernet and power
3. Wait 1-2 minutes for first boot
4. SSH into Pi: `ssh ubuntu@blimp-pi` (password: `ubuntu`)

### 1.3 System Update

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install curl wget git build-essential python3-dev -y
```

### 1.4 Configure System Locale (UTF-8)

⚠️ **Required for proper system operation and ROS 2 compatibility**

```bash
# Install locale generation tool
sudo apt install locales -y

# Generate US English UTF-8 locale
sudo locale-gen en_US en_US.UTF-8

# Update system locale settings
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Verify locale was set
locale
# Should show: LANG=en_US.UTF-8
```

---

## Step 2: Install ROS 2 Humble

### 2.1 Add ROS 2 Repository

```bash
# Install curl and add GPG key
sudo apt install curl -y
curl -sSL https://repo.ros.org/setup.key | sudo apt-key add -

# Add ROS 2 repository
sudo apt install software-properties-common -y
sudo add-apt-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
```

### 2.2 Install ROS 2 Humble

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

### 2.3 Configure Shell

```bash
# Add ROS 2 sourcing to ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc

# Source immediately
source ~/.bashrc
```

### 2.4 Verify Installation

```bash
ros2 --version
# Should output: ROS 2 Humble Hawksbill
```

---

## Step 3.5: Configure GitHub SSH Access (Optional but Recommended)

### For Pushing Code and Updates to GitHub

If you plan to push code changes back to GitHub repositories, set up SSH keys for secure authentication:

```bash
# Generate SSH key (replace with your actual GitHub email)
ssh-keygen -t ed25519 -C "your.email@asu.edu" -f ~/.ssh/github_ed25519 -N ""

# Start SSH agent and add key
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/github_ed25519

# Display public key to add to GitHub account
cat ~/.ssh/github_ed25519.pub
```

**Next steps:**
1. Copy the output from the last command
2. Go to https://github.com/settings/keys
3. Click "New SSH key"
4. Paste the public key
5. Save and test: `ssh -T git@github.com`

### How to Use SSH with Git

```bash
# Clone using SSH (instead of HTTPS)
git clone git@github.com:BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU.git

# Push changes
git push origin branch-name

# Pull updates
git pull origin main
```

---

## Step 4: Clone BLIMP Repository

```bash
cd ~/blimp_ws/src

# Clone official competition BLIMP repository
git clone https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU.git

# (If you have local copy, copy instead)
# cp -r /path/to/local/blimp_src .

# Verify structure
ls -la
```

**Expected output:**
```
src/
├── blimp_interfaces/
├── sensors/
├── sensors_cpp/
├── controls/
├── manual_control/
├── launch/
├── README.md
├── requirements.txt
├── INSTALLATION.md
├── GPIO_PIN_MAP.md
├── NODES_REFERENCE.md
└── TROUBLESHOOTING.md
```

---

## Step 5: Install Dependencies

### 5.1 System Dependencies via rosdep

```bash
cd ~/blimp_ws

# Update rosdep database
rosdep update

# Install all ROS package dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 5.2 Python Package Dependencies

```bash
# Install Python packages from requirements.txt
pip install -r src/blimp_src/requirements.txt
```

### 5.3 Additional System Packages

```bash
# Install pigpio (for motor PWM control)
sudo apt install pigpio pigpiod -y

# Install I2C tools (for sensor debugging)
sudo apt install i2c-tools -y

# Install raspi-config (for hardware configuration)
sudo apt install raspi-config -y

# Install additional ROS packages
sudo apt install ros-humble-joy ros-humble-rqt-gui ros-humble-rqt-console -y

# Install additional Python tools
sudo apt install python3-pip -y
```

---

## Step 6: Build BLIMP Packages

### 6.1 Build Interfaces First (Critical!)

```bash
cd ~/blimp_ws

# Build only blimp_interfaces
colcon build --packages-select blimp_interfaces

# Source the built interface
source install/setup.bash
```

### 6.2 Build All Packages

```bash
# Build remaining packages
colcon build

# Add to bashrc for future sourcing
echo 'source ~/blimp_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 6.3 Verify Build

```bash
# Check all packages built successfully
colcon build --packages-select blimp_interfaces sensors manual_control controls
# Should complete without errors

# Verify packages are discoverable
ros2 pkg list | grep blimp
# Should show: blimp_interfaces, sensors, controls, manual_control
```

---

## Step 7: Enable Hardware Interfaces

### 7.1 Enable I2C

```bash
# Edit boot configuration
sudo nano /boot/firmware/config.txt
```

Add or uncomment these lines:
```
dtparam=i2c_arm=on
dtparam=i2c1=on
```

### 7.2 Enable SPI (if needed)

```bash
# In /boot/firmware/config.txt, add:
dtparam=spi=on
```

### 7.3 Enable Camera Interface

```bash
# In /boot/firmware/config.txt, add:
start_x=1
gpu_mem=256
camera_auto_focus=1
```

### 7.4 Enable raspi-config (Interactive Hardware Setup)

Some hardware settings can also be configured interactively via raspi-config:

```bash
# Launch raspi-config (if installed in Step 5.3)
sudo raspi-config

# Navigate to:
# - Interface Options → I2C (Enable)
# - Interface Options → Camera (Enable)
# - Interface Options → SPI (if needed)
# Exit and reboot if prompted
```

### 7.5 Apply Changes

```bash
# Reboot to activate kernel changes
sudo reboot
```

### 7.6 Verify I2C Hardware

After reboot, verify sensors are visible:

```bash
# List I2C devices
i2cdetect -y 1

# Expected output (example):
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --  <-- BNO055 at 0x28
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 70: -- -- -- -- -- -- 76 --                          <-- BMP390 at 0x76
```

---

## Step 8: Configure pigpio Daemon

### 8.1 Enable and Start Service

```bash
# Enable pigpiod service on boot
sudo systemctl enable pigpiod

# Start the service
sudo systemctl start pigpiod

# Verify it's running
sudo systemctl status pigpiod
```

### 8.2 Test pigpio

```bash
# Install pigpio-dma (optional, for reduced CPU usage)
sudo apt install pigpio-dma -y

# Quick test (make sure no propellers are attached!)
python3 << 'EOF'
import pigpio
import time

pi = pigpio.pi()

# Test ESC 1 on GPIO 5 (neutral PWM = 1500 µs)
pi.hardware_PWM(5, 50, 1500000)  # GPIO 5, 50 Hz, 1500 µs
print("✓ pigpio ESC 1 (GPIO 5) test successful")

# Stop PWM
pi.hardware_PWM(5, 50, 0)
pi.stop()
EOF
```

---

## Step 9: Test Individual Sensors

### 9.1 Test IMU (BNO055)

```bash
# Source workspace
source ~/blimp_ws/install/setup.bash

# Run IMU node
ros2 run sensors read_imu &

# In another terminal, observe IMU data
ros2 topic echo /imu_data --once

# Expected output includes: orientation (x, y, z) and linear_acceleration
```

### 9.2 Test Barometer (BMP390)

```bash
# Run barometer node
ros2 run sensors read_barometer &

# Observe barometer data
ros2 topic echo /barometer_data --once

# Expected: pressure (Pa) and temperature (°C)
```

### 9.3 Test Camera

```bash
# List available cameras
ls /dev/video*

# Test with libcamera
libcamera-hello -t 5s  # Should show preview window

# Or test OpenCV directly
python3 << 'EOF'
import cv2
camera = cv2.VideoCapture(0)
ret, frame = camera.read()
if ret:
    print(f"✓ Camera working! Frame size: {frame.shape}")
else:
    print("✗ Camera error")
camera.release()
EOF
```

---

## Step 10: ESC Arming (REQUIRED Before Launch)

⚠️ **CRITICAL:** ESCs must be armed BEFORE launching the full system.

### 10.1 Prepare for Arming

```bash
# Source workspace
source ~/blimp_ws/install/setup.bash

# Verify pigpiod is running
sudo systemctl status pigpiod
# Should show: Active (running)
```

### 10.2 Run Arming Sequence

**IMPORTANT:** Remove propellers before arming!

```bash
# Terminal 1: Run arming sequence (one time, before first launch)
cd ~/blimp_ws
ros2 launch launch arming.py

# Expected output:
# [arming_sequence] Sending arming PWM signals to ESCs...
# [arming_sequence] ESCs armed successfully - ready for flight
```

**What happens:** Arming sends a standard RC signal sequence (1000 µs for ~2 seconds) to all 4 ESCs. This initializes them and prepares them to accept throttle commands.

### 10.3 Verification

After arming completes successfully:
- ESCs will beep (audible confirmation)
- Motors will not spin (arming does not create thrust, only initializes ESCs)
- System is ready for full launch

---

## Step 11: First System Launch

### 11.1 Launch Full System

**Prerequisites:** ESCs must be armed (Step 10) before this step!

```bash
# (If not already sourced in previous terminal)
source ~/blimp_ws/install/setup.bash

# Create log directory if needed
mkdir -p ~/blimp_ws/logs

# Terminal 1: Launch complete system
cd ~/blimp_ws
ros2 launch launch updated_launch.py 2>&1 | tee logs/system_launch_$(date +%Y%m%d_%H%M%S).log
```

**Expected output:**
```
[INFO] [ros2_launch_tool]: Starting `updated_launch.py`...
[INFO] [esc_driver]: ESC driver started
[INFO] [read_imu]: IMU node started
[INFO] [read_altitude]: Barometer node started
...
[INFO] All nodes launched successfully
```

### 11.2 Verify Nodes in Another Terminal

```bash
# Terminal 2: Check running nodes
ros2 node list

# Should show:
# /detect_cpp
# /esc_driver
# /game_controller_node
# /inv_kine
# /mode_switch
# /pi_controller
# /read_altitude
# /read_imu
```

### 11.3 Check Topics

```bash
# Terminal 3: Monitor topics
ros2 topic list | head -20

# Subscribe to sensor topics to verify data flow
ros2 topic echo /imu_data
# Ctrl+C to stop
```

---

## Step 12: Xbox Controller Setup (Manual Mode)

⚠️ **Required for manual flight control** - This step is only needed if you plan to fly in manual mode using an Xbox controller.

### 12.1 Connect Xbox Controller

1. **Via Bluetooth (Recommended):**
   ```bash
   # Power on Xbox controller (press Xbox button)
   # Hold pairing button (back side of controller) until light flashes
   
   # Make controller discoverable
   bluetoothctl
   # In bluetoothctl prompt:
   > scan on
   # Wait for "Xbox Wireless Controller" to appear
   > pair <MAC_ADDRESS>  # Use the MAC address shown
   > trust <MAC_ADDRESS>
   > quit
   ```

2. **Via USB Cable (Alternative):**
   - Connect Xbox controller to Raspberry Pi USB port using USB cable
   - System should automatically detect it

### 12.2 Install Joystick Utilities

```bash
# Install tools to calibrate and test controller
sudo apt install joystick jstest-gtk -y

# Install ROS 2 Joy node (if not already installed)
sudo apt install ros-humble-joy -y
```

### 12.3 Identify Controller Device

```bash
# List all input devices
ls -la /dev/input/js*

# Expected output:
# crw-rw----  1 root input 13,   0 Dec 10 09:45 /dev/input/js0
# (js0 is the first joystick/controller)

# Test if controller is working
jstest /dev/input/js0

# You should see live axis and button readings:
# Axes:  0: 32767  1:     0  2: 32767  3:     0  4:     0  5: 32767  6:     0  7:     0
# Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off
# Move joysticks and press buttons - values should change
# Press Ctrl+C to exit
```

### 12.4 Configure ROS 2 Joy Node

Create a Joy configuration file:

```bash
# Create config directory
mkdir -p ~/blimp_ws/config

# Create joy_config.yaml
cat > ~/blimp_ws/config/joy_config.yaml << 'EOF'
joy_node:
  ros__parameters:
    device_id: 0            # /dev/input/js0
    deadzone: 0.15          # Ignore small stick movements
    autorepeat_rate: 50.0   # Publish at 50 Hz
    coalesce_interval_ms: 1 # No delay between readings
EOF
```

### 12.5 Launch Joy Node

**Terminal 1: Start Joy Node**

```bash
# Source workspace
source ~/blimp_ws/install/setup.bash

# Launch Joy node with config
ros2 run joy joy_node --ros-args --params-file ~/blimp_ws/config/joy_config.yaml
```

**Terminal 2: Verify Joy Topic**

```bash
source ~/blimp_ws/install/setup.bash

# Monitor Joy messages (make sure to press buttons/move sticks)
ros2 topic echo /joy --once

# Expected output:
# header:
#   stamp:
#     sec: 1702220045
#     nanosec: 123456789
#   frame_id: ''
# axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]  # 8 analog axes
# buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]       # 11 buttons
```

### 12.6 Xbox Controller Axis/Button Mapping

Memory aid - Think of the controller as having these axes (values -1.0 to 1.0):

```
                     ↑ (1.0)
                     |
    LT (0 to -1)  ← Joy ↔ Joy →  RT (0 to -1)
                     |
                     ↓ (-1.0)

Index mapping (from Joy message):
axes[0]  = LStick Left/Right    (-1.0=left, 1.0=right)
axes[1]  = LStick Up/Down       (-1.0=up,   1.0=down)
axes[2]  = RStick Up/Down       (-1.0=up,   1.0=down)
axes[3]  = RT trigger           (1.0=released, -1.0=fully pressed)
axes[4]  = RStick Left/Right    (NOT USED for BLIMP)
axes[5]  = LT trigger           (1.0=released, -1.0=fully pressed)

buttons[0-10]:
buttons[0] = A button (green)
buttons[1] = B button (red)
buttons[2] = X button (blue)
buttons[3] = Y button (yellow)
buttons[4] = LB (left bumper)
buttons[5] = RB (right bumper)  ← MODE SWITCH: Press RB to switch manual/autonomous
buttons[6] = Back button
buttons[7] = Start button
buttons[8] = Left stick click
buttons[9] = Right stick click
buttons[10] = Xbox logo button
```

### 12.7 Xbox Controller to Motor Command Mapping

This is what happens when you move the controller during flight:

| Controller Input | Motor Effect | Code Reference |
|---|---|---|
| Left Stick Y (forward/back) | Forward/backward thrust | `F = 1950 + ((1050-1950)/(-2)) × (axes[1]-1)` |
| Left Trigger (0 to -1) | Right motor trim (yaw right) | `RTrim = -450 × axes[5]` |
| Right Trigger (0 to -1) | Left motor trim (yaw left) | `LTrim = 450 + ((0-450)/2) × (axes[3]+1)` |
| Right Stick Y (up/down) | Down motor thrust (altitude) | `DM = 1950 + ((1050-1950)/(-2)) × (axes[2]-1)` |
| Right Bumper (RB) | **Mode Switch** | Toggles between manual (RB pressed) and autonomous (RB released) |

**PWM Range:** 1050 - 1950 µs @ 50 Hz
- 1050 µs = Minimum throttle (least thrust)
- 1500 µs = Mid point (no command)
- 1950 µs = Maximum throttle (most thrust)

### 12.8 Test Manual Mode

**Prerequisites:**
- ESCs armed (Step 10 completed)
- Full system launched (Step 11)
- Joy node running (Step 12.5)
- **NO propellers attached!**

```bash
# Terminal 1: Launch full system (if not already running)
source ~/blimp_ws/install/setup.bash
ros2 launch launch updated_launch.py

# Terminal 2: Monitor manual control input
source ~/blimp_ws/install/setup.bash
ros2 topic echo /ESC_Manual_input --once

# Terminal 3: Move Xbox controller and see PWM changes
# Expected output when moving Left Stick forward:
# esc_pwm: [1500, 1500, 1500, 1500]  # All motors neutral
# Then:
# esc_pwm: [1700, 1700, 1500, 1500]  # Left motors increase (forward)

# Verify RB button switches mode
# Press RB (Right Bumper) and watch /esc_input topic change between topics
ros2 topic echo /esc_input
```

### 12.9 Troubleshooting Xbox Controller

**Controller not detected:**
```bash
# Check if connected
ls /dev/input/js*
# If nothing shows, controller isn't connected

# For Bluetooth issues:
bluetoothctl devices
bluetoothctl paired-devices

# Try re-pairing
bluetoothctl remove <MAC_ADDRESS>
# Then repeat section 12.1
```

**Joy node not publishing:**
```bash
# Check Joy node is running
ros2 node list | grep joy

# Monitor Joy node status
ros2 node info /joy_node

# Check for permission errors
sudo usermod -aG input $USER
# Log out and back in
```

**Controller axes/buttons mapping incorrect:**
```bash
# Interactive test
jstest /dev/input/js0
# Move sticks and press buttons to verify axis/button numbers match above mapping

# If mapping different, you may have a different Xbox controller model
# Contact development team with jstest output
```

---

## Understanding Autonomous Mode

This section explains how the BLIMP balloon detection and autonomous flight works. **Read this before attempting autonomous flight.**

### Autonomous System Architecture

When in autonomous mode (Right Bumper NOT pressed), the system follows this pipeline:

```
Raspberry Pi Camera
        ↓
  [detect_node - YOLOv5]
  Runs balloon detection
        ↓
  /cam_data topic
  (balloon location)
        ↓
  [pi_controller - PI Control]
  Calculates desired yaw/height
        ↓
  /balloon_input topic
  (desired accelerations)
        ↓
  [inv_kine - Inverse Kinematics]
  Converts to motor PWM values
        ↓
  [mode_switcher]
  Routes autonomous commands
        ↓
  [esc_driver]
  Sends PWM to motors
        ↓
  3 Motors on GPIO 5/6/13 (GPIO 26 unused)
  BLIMP moves autonomously!
```

### Step A: Camera Balloon Detection (old_cam.cpp)

**Purpose:** Detect the target balloon in camera frames and publish its location

**How it works:**
1. Raspberry Pi Camera captures video frames (1280×720 @ ~5 FPS)
2. Frame converted from BGR color to HSV (easier for color detection)
3. **Purple/Magenta detection:** Mask created for purple color range:
   - HSV range: H=115-150°, S=40-255, V=30-255
   - This matches standard purple balloons
4. **Contour finding:** All purple regions identified in frame
5. **Largest contour selected:** The biggest purple blob (assuming it's the balloon)
6. **Center calculation:** Compute (cx, cy) = centroid of largest blob
7. **Minimum radius check:** Discard small blobs (<5 pixels) as false positives
8. **Publish:** Send CameraCoord message to `/cam_data` topic:
   ```
   message CameraCoord:
     position[0] = cx (x-coordinate in pixels, 0-1280)
     position[1] = cy (y-coordinate in pixels, 0-720)
   ```

**Example output:**
```
If balloon at center of image:    position = [640, 360]
If balloon top-left of image:     position = [200, 150]
If balloon bottom-right:          position = [1100, 650]
If no balloon detected:           position = [0, 0] (no publish)
```

**Detection Rate:** ~5-10 Hz (camera processing takes ~100-200ms per frame)

**Color tuning:**
If the detection doesn't find your specific balloon, adjust HSV ranges in `sensors_cpp/src/old_cam.cpp` around lines 115-150. Other colors available:
- Yellow (goal): H=28-36°, S=80-255, V=120-255
- Orange: H=1-12°, S=120-255, V=50-255
- Red: H=120-145°, S=50-255, V=50-255
- Green: H=41-56°, S=80-255, V=80-255

### Step B: PI Control (Balloon_pi.cpp)

**Purpose:** Read balloon position and compute desired yaw/altitude commands to track it

**Input:** `/cam_data` topic (balloon centroid from camera detection)

**Output:** `/balloon_input` topic - CartCoord message with desired accelerations

**Control Strategy:**

The PI (Proportional-Integral) controller uses two independent loops:

**Loop 1: Yaw Control (Left-Right movement)**
```
Error = Balloon_X - Frame_Center_X
      = cam_data.position[0] - 640
      
If error > 0:  balloon is RIGHT of center  → yaw command turns RIGHT
If error < 0:  balloon is LEFT of center   → yaw command turns LEFT

Command = Kp_x × Error + Ki_x × Integral(Error)
        = 0.5 × Error + 0.1 × Sum(past errors × 0.02s)

Clamp output to [-100, 100] for safety
```

**Loop 2: Altitude Control (Up-Down movement)**

If **camera data is fresh** (< 5 seconds old):
```
Error_Y = Target_Y - cam_data.position[1]
        = 360 - cam_data.position[1]

If error > 0:  balloon is above target      → move UP
If error < 0:  balloon is below target      → move DOWN

Command = Kp_y_up × Error + Ki_y × Integral(Error)      if error > 0
        = Kp_y_down × Error + Ki_y × Integral(Error)    if error < 0
```

If **camera data is stale** (> 5 seconds with no detections):
```
Switch to barometer altitude hold mode:
Error_Alt = Target_Altitude - Current_Altitude (from barometer)

Command = Kp_baro × Error_Alt
```

**Control Gains (tunable parameters):**
```
kpx = 0.5    # Yaw responsiveness (increase for faster tracking, higher = more jittery)
kix = 0.1    # Yaw integral gain (helps eliminate steady-state error)
kpyu = 0.3   # Altitude up responsiveness
kpyd = 0.3   # Altitude down responsiveness (can be different from up)
kiy = 0.05   # Altitude integral gain
kpb = 0.3    # Barometer control gain (when camera loses signal)

Target altitude: iheight = 1.5 (meters, tunable)
Target Y position: y_goal = 360 (pixels, frame center)
Target X position: x_goal = 640 (pixels, frame center)
```

**Expected behavior:**

With a balloon 100 pixels to the right of center:
```
Error_x = 640 - 740 = -100 pixels
Command_yaw = 0.5 × (-100) + 0.1 × (-100×0.02) = -50 - 0.2 = -50.2 rad/s
→ Vehicle yaws LEFT to bring error to zero
```

**Timeout behavior:**

If camera loses detection for >5 seconds:
- Yaw control STAYS active (tries last known position)
- Altitude control SWITCHES to barometer-only (maintains height)
- System assumes balloon may have moved out of frame

### Step C: Inverse Kinematics (inv_kine.cpp)

**Purpose:** Convert desired accelerations into individual motor PWM commands

**Input:** `/balloon_input` - CartCoord message with (x, y, z, phi, theta, psy)

**Motor Configuration:**

BLIMP has 3 active motors in a specific arrangement:
```
           Bottom Motor (GPIO 13)
               |  ↓
               |  (Thrust: up/down)
               |  
Left Motor ←→ Right Motor
(GPIO 5)      (GPIO 6)

Motor effects:
- **Left motor (GPIO 5):** Yaw correction #1
- **Right motor (GPIO 6):** Yaw correction #2  
- **Bottom motor (GPIO 13):** Vertical thrust (altitude control)
- **GPIO 26:** Unused (reserved for future use)
```

**Thrust Conversion:**

```
Input:  [x_accel, y_accel, z_accel, φ_moment, θ_moment, ψ_moment]

Output: [PWM_1, PWM_2, PWM_3]  (3 active motor commands, 1000-2000 µs)
           |       |       |
          Left   Right   Bottom

Process: 
1. Extract yaw and thrust commands from desired accelerations
2. Allocate to motors using 3-motor configuration:
   - Yaw: Left motor = base_PWM + yaw_correction
          Right motor = base_PWM - yaw_correction
   - Thrust: Bottom motor = desired_vertical_PWM
3. Clamp each motor to [1000, 2000] µs range
4. Publish /esc_balloon_input with 3 PWM values
```

**Motor mapping (active motors):**
```
Motor 1 (GPIO 5):  Left motor   (yaw control)
Motor 2 (GPIO 6):  Right motor  (yaw control)
Motor 3 (GPIO 13): Bottom motor (vertical thrust)
Motor 4 (GPIO 26): UNUSED - reserved for future expansion
```

### Step D: Mode Switching (mode_switcher.py)

**Purpose:** Route either manual OR autonomous motor commands based on RB button

**Manual Mode (RB pressed):**
- Subscribes to `/ESC_Manual_input` (from Xbox controller)
- Publishes that directly to `/esc_input` (ESC driver)
- System responds immediately to controller input

**Autonomous Mode (RB NOT pressed):**
- Subscribes to `/esc_balloon_input` (from inverse kinematics)
- Publishes that to `/esc_input` (ESC driver)
- System follows detected balloon autonomously

**Real-time mode switching:**
- RB button state is checked constantly
- Can switch modes mid-flight (NOT RECOMMENDED for beginners!)
- Recommended: Switch on ground before arming and flying

---

## Running Autonomous Mode

### Prerequisites
- ✅ ESCs armed (Step 10)
- ✅ Full system launched (Step 11)
- ✅ Xbox controller connected (Step 12)
- ✅ Balloon present and visible to camera
- ✅ Color-matched to detection parameters (see camera section)

### Launch Steps

**Terminal 1: Launch full system**
```bash
source ~/blimp_ws/install/setup.bash
ros2 launch launch updated_launch.py
```

**Terminal 2: Verify balloon detection**
```bash
source ~/blimp_ws/install/setup.bash

# Monitor camera detection
ros2 topic echo /cam_data

# When balloon visible, should show:
# position: [640, 360]  # Example: balloon at center
# Every ~0.1-0.2 seconds

# If blank/no messages: Camera not detecting balloon
# Adjust color ranges or check lighting
```

**Terminal 3: Monitor autonomous control output**
```bash
source ~/blimp_ws/install/setup.bash

# Watch PI controller commands
ros2 topic echo /balloon_input

# Should show CartCoord with:
# x: 0.0
# y: 0.0
# z: <altitude_command>   # Changes based on balloon vertical position
# theta: 0.0
# phi: 0.0
# psy: <yaw_command>      # Changes based on balloon horizontal position
```

**Terminal 4: Monitor motor PWM commands**
```bash
source ~/blimp_ws/install/setup.bash

# Watch final ESC input
ros2 topic echo /esc_input

# Should show EscInput with:
# esc_pwm: [1500, 1500, 1500, 1500]  # Neutral
# Then changing values as controller reacts to balloon position
```

### Starting Autonomous Flight

1. **Place balloon in full camera view** - must be clearly visible
2. **Release BLIMP gently** - let it settle
3. **Press Right Bumper (RB) on Xbox controller** - switches to autonomous mode
4. **BLIMP starts tracking balloon** - moves to keep it centered
5. **To switch back to manual:** Release RB button

### Emergency Stop

At any time:
- **Press RB button** → switches to manual mode (requires active joystick control)
- **Press Start button** → (if implemented) emergency motor shutdown
- **Power off BLIMP** → immediate power loss

### Tuning Autonomous Behavior

If the BLIMP behavior isn't ideal:

**Too slow to track:**
```
Increase gains in pi_controller parameters:
ros2 param set /balloon_detect_PI kpx 1.0    # Was: 0.5
ros2 param set /balloon_detect_PI kpyu 0.5   # Was: 0.3
```

**Too jittery/oscillating:**
```
Decrease gains and increase integral terms:
ros2 param set /balloon_detect_PI kpx 0.3    # Was: 0.5
ros2 param set /balloon_detect_PI kix 0.05   # Was: 0.1  
```

**Loses tracking frequently:**
```
Adjust camera detection color ranges in sensors_cpp/src/old_cam.cpp
Check lighting conditions - HSV detection requires good lighting
```

---

## Safety Notes

⚠️ **BEFORE AUTONOMOUS FLIGHT:**

1. **NO PROPELLERS** while testing on ground
2. **Clear flight area** - no people, obstacles, or delicate objects nearby  
3. **Outdoor flight recommended** - more space and better lighting for camera
4. **Test on tether first** - attach fishing line to BLIMP for safety
5. **Start with low gains** - tune up gradually while observing behavior
6. **Manual mode backup** - always have joystick ready in case RB button needed
7. **Camera quality matters** - clean camera lens before flight
8. **Balloon size matters** - small balloons harder to detect, use ~30cm diameter or larger

---

## Troubleshooting Autonomous Mode

**Balloon not detected:**
```bash
# Check camera feed
ros2 topic echo /cam_data

# If no messages: camera not detecting purple
# Solutions:
# 1. Ensure good lighting (preferably daylight or bright room)
# 2. Check balloon color matches HSV range in old_cam.cpp
# 3. Test camera with: libcamera-hello -t 5s
# 4. If camera very dark, BLIMP may have bad camera connection

# Manually adjust HSV in old_cam.cpp lines 115-150 and rebuild:
colcon build --packages-select sensors_cpp
source ~/blimp_ws/install/setup.bash
```

**BLIMP drifts instead of tracking:**
```bash
# Check PI gains
ros2 param list | grep balloon
ros2 param get /balloon_detect_PI kpx

# Gains too low - increase:
ros2 param set /balloon_detect_PI kpx 0.8
ros2 param set /balloon_detect_PI kiy 0.15

# Wait 5 seconds without camera data:
# System switches to barometer mode only (yaw control active but altitude frozen)
```

**BLIMP oscillates around balloon:**
```bash
# Gains too high - decrease:
ros2 param set /balloon_detect_PI kpx 0.2
ros2 param set /balloon_detect_PI kix 0.05

# Increase integral action to smooth:
ros2 param set /balloon_detect_PI kiy 0.2
```

**Mode switching doesn't work:**
```bash
# Check RB button input:
ros2 topic echo /joy | grep buttons

# Count from buttons[0] onward:
# buttons[5] should toggle 0↔1 when pressing RB

# If not working:
# 1. Reconnect Xbox controller
# 2. Verify with jstest /dev/input/js0 that RB button shows
# 3. Check mode_switcher node is running
```

---

## Troubleshooting Installation

### Issue: "ModuleNotFoundError: No module named 'pigpio'"

```bash
# Solution: Install pigpio
pip install pigpio

# Or via apt
sudo apt install python3-pigpio -y

# Ensure pigpiod is running
sudo systemctl status pigpiod
```

### Issue: "FileNotFoundError: /dev/i2c-1 not found"

```bash
# Solution: Enable I2C in /boot/firmware/config.txt
sudo nano /boot/firmware/config.txt
# Add: dtparam=i2c_arm=on

# Reboot
sudo reboot
```

### Issue: "ros2: command not found"

```bash
# Solution: Source ROS 2 setup script
source /opt/ros/humble/setup.bash

# Or add to ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Issue: colcon build fails with CMake errors

```bash
# Solution: Clean and rebuild
cd ~/blimp_ws
colcon clean
colcon build --packages-select blimp_interfaces
source install/setup.bash
colcon build --packages-select sensors
colcon build --packages-select controls
colcon build
```

---

## Next Steps

1. ✅ **Installation Complete** - All packages built and ready
2. 📖 **Read README.md** - Understand system architecture
3. 🧪 **Run Tests** - Verify all sensors and systems working
4. 🚀 **Launch Demo** - Test autonomous or manual mode
5. 📝 **Review Handoff Report** - Understand full system design

---

## Quick Reference Commands

```bash
# Source workspace
source ~/blimp_ws/install/setup.bash

# Launch ESC arming (before flight!)
ros2 launch launch arming.py

# Launch full system
ros2 launch launch updated_launch.py

# Check running nodes
ros2 node list

# Monitor sensor data
ros2 topic echo /imu_data
ros2 topic echo /barometer_data
ros2 topic echo /cam_data

# View system graph
rqt_graph

# Record data
ros2 bag record -a

# Rebuild if needed
cd ~/blimp_ws && colcon build

# Kill all ROS 2 processes
killall -9 ros2
```

---

**Installation Complete!** Proceed to `README.md` for system operation instructions.

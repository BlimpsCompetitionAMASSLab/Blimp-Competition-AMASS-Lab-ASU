# Troubleshooting Guide - BLIMP Platform

Comprehensive troubleshooting guide organized by symptom and system component.

---

## Quick Reference by Symptom

| Symptom | Likely Cause | See Section |
|---------|---|---|
| ROS 2 nodes won't launch | Missing dependencies, build errors | [ROS 2 & Build Issues](#ros-2--build-issues) |
| IMU reads crazy angles | I2C connection, calibration | [IMU (BNO055) Issues](#imu-bno055-issues) |
| Barometer altitude wrong | Sea-level pressure calibration | [Barometer (BMP390) Issues](#barometer-bmp390-issues) |
| Motors don't spin | ESC not armed, pigpio not running | [Motor Control Issues](#motor-control-issues) |
| Camera not found | Interface not enabled, connection | [Camera Issues](#camera-issues) |
| WiFi drops packets | Signal weak, bandwidth overloaded | [Network Issues](#network-issues) |
| Autonomous mode not working | Vision detection failing, control unstable | [Autonomy Issues](#autonomy-issues) |
| System crashes/restarts | CPU overload, thermal throttling, power brownout | [System Stability Issues](#system-stability-issues) |

---

## ROS 2 & Build Issues

### Problem: `ros2: command not found`

**Symptoms:**
- Bash command `ros2` not recognized
- Error when running `ros2 --version`

**Diagnosis:**
```bash
# Check if ROS 2 is installed
ls /opt/ros/

# Check if setup.bash was sourced
echo $ROS_DOMAIN_ID  # Should return a number, not blank
```

**Solutions:**

1. **ROS 2 not installed:** Install following [Installation Guide](INSTALLATION.md#step-2-install-ros-2-humble)

2. **Setup not sourced:**
   ```bash
   # Source immediately
   source /opt/ros/humble/setup.bash
   
   # Add to bashrc permanently
   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Wrong shell path:**
   ```bash
   # Check shell is bash, not sh
   echo $SHELL
   
   # If using dash, switch to bash
   chsh -s /bin/bash
   ```

---

### Problem: Nodes fail to launch with errors

**Symptoms:**
- `$ ros2 run sensors read_imu` → Error message
- Launch file crashes
- Missing package error

**Diagnosis:**
```bash
# Check package exists
ros2 pkg list | grep sensors

# Check for build errors
cd ~/blimp_ws && colcon build --packages-select sensors
# Look for red/error messages

# Check if workspace sourced
echo $COLCON_PREFIX_PATH  # Should include ~/blimp_ws/install
```

**Solutions:**

1. **Package not built:**
   ```bash
   # Rebuild from scratch
   cd ~/blimp_ws
   colcon clean
   colcon build --packages-select blimp_interfaces
   source install/setup.bash
   colcon build --packages-select sensors controls
   colcon build
   ```

2. **Dependencies missing:**
   ```bash
   # Install system and Python dependencies
   rosdep install --from-paths src --ignore-src -r -y
   pip install -r src/BLIMP-Packages/requirements.txt
   ```

3. **Workspace not sourced:**
   ```bash
   # Source workspace in each terminal
   source ~/blimp_ws/install/setup.bash
   
   # Or add to ~/.bashrc
   echo 'source ~/blimp_ws/install/setup.bash' >> ~/.bashrc
   ```

---

### Problem: `colcon build` fails with CMake errors

**Symptoms:**
- Build stops with `CMake Error`
- Compiler errors (C++ only)
- Missing build tools

**Diagnosis:**
```bash
# Check build output carefully
colcon build 2>&1 | tail -50  # Show last 50 lines

# Check if build dependencies installed
dpkg -l | grep build-essential
```

**Solutions:**

1. **Build dependencies missing:**
   ```bash
   sudo apt install build-essential cmake -y
   ```

2. **Clean and rebuild:**
   ```bash
   cd ~/blimp_ws
   colcon clean  # Remove build/ install/ log/ directories
   colcon build --packages-select blimp_interfaces
   ```

3. **Specific package rebuild:**
   ```bash
   # Rebuild one package completely
   colcon build --packages-select sensors_cpp --no-cache
   ```

---

## IMU (BNO055) Issues

### Problem: IMU publishes crazy angles (jumps between -180° and +180°)

**Symptoms:**
- `/imu_data.orientation.z` jumps randomly
- Angles not smooth when rotating
- Euler angle singularity at pitch = 90°

**Diagnosis:**
```bash
# Check I2C connection
i2cdetect -y 1
# BNO055 should appear at 0x28 or 0x29

# Monitor raw data
ros2 topic echo /imu_data --once
# Check if euler_z is within [-π, π] range
```

**Root Causes & Solutions:**

1. **I2C connection loose:**
   - Reseat jumper wires on GPIO 2 (SDA), GPIO 3 (SCL)
   - Check level shifter connections (3.3V → 5V)
   - Test with short direct wires (< 10cm)

2. **I2C pull-up resistors missing/weak:**
   ```bash
   # Check resistor values (should be ~10kΩ)
   # If not present, solder 10kΩ resistors:
   # - Between SDA and 5V
   # - Between SCL and 5V
   ```

3. **EMI from motor/ESC wires:**
   - Route I2C cable away from power wires
   - Use shielded I2C cable if available
   - Add capacitors (0.1µF) across each sensor's power pins

4. **BNO055 not in NDOF mode:**
   ```python
   # In sensors/sensors/barometer.py, ensure mode is set
   sensor.mode = adafruit_bno055.OPERATION_MODE_NDOF
   # vs. OPERATION_MODE_IMUPLUS or others
   ```

5. **Sensor needs recalibration:**
   ```bash
   # Run calibration
   ros2 launch launch BNO085_calibrate.py
   # Follow on-screen instructions (rotate in 3D)
   ```

---

### Problem: IMU not responding / I2C address not found

**Symptoms:**
- `i2cdetect -y 1` shows no device at 0x28 or 0x29
- Read timeout errors
- Node crashes trying to query sensor

**Diagnosis:**
```bash
# Check I2C bus
i2cdetect -y 1

# Check power to sensor
# Connect voltmeter to VDD pin → should be 5V ±0.2V

# Check I2C signal integrity
# Oscilloscope on SDA/SCL → should be rising/falling edges
```

**Solutions:**

1. **Sensor not powered:**
   - Verify 5V USB regulator output is 5V
   - Check VDD pin soldering on breakout board
   - Test with separate 5V supply

2. **I2C bus hung (stuck LOW):**
   ```bash
   # SDA or SCL stuck at 0V → short circuit
   # Disconnect all I2C devices one by one
   # Identify which device is stuck
   
   # Reset I2C bus
   sudo i2cdetect -r 1  # Force reset attempt
   ```

3. **Level shifter not working:**
   - Check level shifter power: HV=5V, LV=3.3V
   - Test each pin individually with multimeter
   - Replace level shifter if faulty

4. **Wrong I2C address:**
   - Check COM3 pin on BNO055 (pullup → 0x28, pulldown → 0x29)
   - Check SDO pin on BMP390 (low → 0x76, high → 0x77)
   - Verify in sensor configuration code

---

### Problem: IMU drifts or reports inconsistent orientation

**Symptoms:**
- Yaw angle slowly increases/decreases even when stationary
- Roll/pitch jerks unexpectedly
- Attitude estimation inconsistent

**Diagnosis:**
```bash
# Record 1 minute of IMU data while stationary
ros2 topic echo /imu_data > imu_stationary.log &
sleep 60
killall ros2

# Analyze drift
python3 << 'EOF'
import re
import matplotlib.pyplot as plt

angles = {'z': []}
with open('imu_stationary.log') as f:
    for line in f:
        if 'z:' in line and 'orientation' in f.read(-200):
            match = re.search(r'z: ([\d.-]+)', line)
            if match:
                angles['z'].append(float(match.group(1)))

plt.plot(angles['z'])
plt.ylabel('Yaw (rad)')
plt.title('Yaw Drift Stationary')
plt.savefig('imu_drift.png')
EOF
```

**Root Causes & Solutions:**

1. **Sensor needs recalibration:**
   ```bash
   ros2 launch launch BNO085_calibrate.py
   ```

2. **Magnetic interference (compass drift):**
   - Move IMU away from motors, ESC
   - Check for large magnetic sources nearby
   - In software, disable magnetometer if drift unbearable:
     ```python
     sensor.mode = adafruit_bno055.OPERATION_MODE_IMUPLUS  # Remove compass
     ```

3. **Excessive vibration:**
   - Ensure vibration isolation pads are under IMU
   - Check screws securing IMU bracket are not loose
   - Verify propeller balance

---

## Barometer (BMP390) Issues

### Problem: Altitude reads 1000m when sitting on ground

**Symptoms:**
- `/barometer_data.altitude` is wildly incorrect
- Constant offset (not random walk)
- Changes with temperature

**Diagnosis:**
```bash
# Check raw pressure reading
ros2 topic echo /barometer_data --once
# Look at .pressure value (should be ~101325 Pa at sea level)

# Compare to know sea-level pressure
# Check weather.com or local airport METAR data
```

**Root Cause & Solution:**

**Sea-level pressure calibration incorrect:**
```python
# In sensors/sensors/barometer.py, find:
SEA_LEVEL_PRESSURE = 101325.0  # Default

# Update to local value (from weather service)
# Example for Tempe, AZ at 1100 ft elevation:
SEA_LEVEL_PRESSURE = 100650.0  # Approximate

# Or calculate from first measurement:
# If you know your altitude (e.g., 100m), record absolute pressure
# Then solve inverse of altitude formula for P0
```

**Altitude Formula:**
$$P_0 = P \left( 1 - \frac{a}{T_0} \right)^{-\frac{gM}{R \cdot a}}$$

For simple compensation:
```python
# After recording pressure at known altitude:
reference_altitude = 100.0  # meters
reference_pressure = 99375.0  # Pa (your measured value)

# Solve for sea-level pressure
import math
SEA_LEVEL_PRESSURE = reference_pressure * (1 - reference_altitude / 44330) ** (-5.255)
```

---

### Problem: Barometer altitude is noisy (±0.5m variations)

**Symptoms:**
- `/barometer_data.altitude` oscillates ±0.5m
- Notchy altitude changes
- Sensor reading jerks

**Diagnosis:**
```bash
# Sample altitude for 10 seconds, compute std-dev
ros2 topic echo /barometer_data > baro_10sec.log &
sleep 10
Python3 << 'EOF'
import re
import numpy as np
values = []
with open('baro_10sec.log') as f:
    for line in f:
        match = re.search(r'altitude: ([\d.-]+)', line)
        if match:
            values.append(float(match.group(1)))
print(f"Std-dev: {np.std(values):.3f} m")
print(f"Range: [{np.min(values):.2f}, {np.max(values):.2f}] m")
EOF
```

**Root Cause & Solution:**

**This is normal** - BMP390 has inherent noise ~0.3m due to quantization and atmospheric turbulence. No action needed for typical applications.

**If noise is excessive (> 1m):**
1. Check sensor power supply is clean (5V regulated)
2. Apply Kalman filter (already implemented in inv_kine node)
3. Increase measurement averaging window in driver

---

### Problem: Barometer not responding

**Symptoms:**
- `i2cdetect -y 1` shows no device at 0x76/0x77
- Node crashes reading sensor
- Read timeout errors

**Diagnosis & Solutions:**
Same as [IMU not responding](#problem-imu-not-responding--i2c-address-not-found) - follow those steps for BMP390 at address 0x76 or 0x77 depending on SDO pin configuration.

---

## Motor Control Issues

### Problem: Motors don't spin, no response to commands

**Symptoms:**
- `/esc_input` publishes values, but motors don't move
- No servo/motor whining sound
- PWM pins outputting (can verify with multimeter)

**Diagnosis:**
```bash
# Step 1: Check pigpiod is running
sudo systemctl status pigpiod
# Should show: Active (running)

# Step 2: Verify ESCs are armed
ros2 topic echo /esc_input
# Should show PWM values other than 1000

# Step 3: Check GPIO pins manually
python3 << 'EOF'
import pigpio
pi = pigpio.pi()
# Check if pigpio connected
if pi.connected:
    print("✓ pigpio connected")
    # Test ESC 1 on GPIO 5 (set to 1500 µs neutral)
    pi.hardware_PWM(5, 50, 1500000)  # GPIO 5, 50 Hz, 1500 µs
    print("✓ ESC 1 (GPIO 5) PWM test successful")
else:
    print("✗ pigpio not connected")
pi.stop()
EOF

# Step 4: Use multimeter to check voltage on motor power
# Should see 11.1V on ESC input, 5V on servo power
```

**Root Causes & Solutions:**

1. **pigpiod not running:**
   ```bash
   # Start service
   sudo systemctl start pigpiod
   sudo systemctl enable pigpiod  # Auto-start on boot
   
   # Or run manually
   sudo pigpiod -s 1  # 1 microsecond sample rate
   ```

2. **ESCs not armed:**
   ```bash
   # Run arming sequence
   ros2 launch launch arming.py
   # Sends 1000 µs PWM for ~2 seconds to arm ESCs
   ```

3. **Propeller installed → safety disable:**
   - **Remove propellers immediately!**
   - Most ESCs have safety feature preventing motor start with unknown propeller state

4. **Power not reaching motors:**
   - Check battery voltage: should be 11.1V (3S LiPo full)
   - Check voltage at ESC input: should be ~11.1V
   - Check servo power (5V rail): should be 5V ±0.2V
   - Test with fresh fully-charged battery

5. **Motor/ESC connection issue:**
   - Verify servo motor headers are plugged into ESC
   - Check motor wire colors (red=+, black=-, yellow/white=signal)
   - Test motor on different ESC if available

6. **GPIO pin wrong:**
   - Verify in esc_driver.py: pins should be [17, 22, 23, 24, 27]
   - Cross-check with [GPIO_PIN_MAP.md](GPIO_PIN_MAP.md)

---

### Problem: Motor spins but control is jittery/unstable

**Symptoms:**
- Wide oscillations in thrust output
- Control lag or overshoots
- Motors whine with high-frequency noise

**Diagnosis:**
```bash
# Monitor motor PWM output
ros2 topic echo /esc_input

# Check for oscillations in control
ros2 topic echo /esc_balloon_input | tee motor_cmds.log &
sleep 30
# Analyze log for high-frequency jitter

# Monitor CPU usage (high load → delays)
top -b -n 5  # Show top processes
```

**Root Causes & Solutions:**

1. **PID gains too aggressive (Kp too high):**
   - Reduce Kp by factor of 1.5-2x
   - Edit configs/pid_gains.yaml:
     ```yaml
     pi_controller:
       yaw:
         kp: 0.3  # Was 0.5
     ```

2. **Insufficient sensor filtering:**
   - Kalman filter may need tuning
   - Increase process noise covariance
   - Add RC low-pass filter on PWM signals (resistor-capacitor):
     ```
     τ = R × C (time constant)
     For 5Hz cutoff: R = 100kΩ, C = 0.33µF
     ```

3. **Control loop too fast:**
   - Check mode_switcher loop frequency (should be 50Hz)
   - Reduce if causes CPU overload

4. **Motor resonance:**
   - Servo motors have natural frequency
   - Avoid control frequencies near resonance
   - Increase servo damping or weight

5. **Loose propellers:**
   - Even slight imbalance causes vibration/noise
   - Tighten propeller screws
   - Verify propellers are balanced

---

## Camera Issues

### Problem: Camera not found / image capture fails

**Symptoms:**
- `libcamera-hello` fails with "ERROR: Cannot find camera"
- Python CV2 cannot open device
- ROS 2 camera node won't start

**Diagnosis:**
```bash
# Check if camera interface enabled
grep "start_x" /boot/firmware/config.txt

# List video devices
ls -l /dev/video*
# Should show at least /dev/video0 (camera device)

# Test with libcamera directly
libcamera-hello -t 5s
# Should show preview window or error message

# Test with OpenCV
python3 << 'EOF'
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    ret, frame = cap.read()
    if ret:
        print(f"✓ Camera working! Frame size: {frame.shape}")
    else:
        print("✗ Cannot read frame")
else:
    print("✗ Cannot open camera device")
cap.release()
EOF
```

**Root Causes & Solutions:**

1. **Camera interface not enabled:**
   ```bash
   # Edit boot config
   sudo nano /boot/firmware/config.txt
   
   # Uncomment or add:
   start_x=1
   gpu_mem=256
   
   # Save and reboot
   sudo reboot
   ```

2. **Camera ribbon cable disconnected/seeded:**
   - Power off Raspberry Pi
   - Locate Camera Serial Interface (CSI) connector (between USB and 3.5mm jack)
   - Gently lift retention clips
   - Reseat ribbon cable (blue part toward ethernet)
   - Close clips firmly
   - Power on and test

3. **Camera hardware defective:**
   - Test on different Raspberry Pi if available
   - Check ribbon cable for visible damage
   - Verify camera LED lights up when powered

4. **Permission issues:**
   ```bash
   # Add user to video group
   sudo usermod -aG video ubuntu
   
   # Log out and back in
   logout
   ```

---

### Problem: Camera frames have low quality / blurry detection

**Symptoms:**
- YOLOv5 detections show low confidence (< 0.5)
- Frequent false positives/negatives
- Images appear out of focus

**Diagnosis:**
```bash
# Capture and inspect raw image
python3 << 'EOF'
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cv2.imwrite('/tmp/camera_test.jpg', frame)
print(f"Image saved: {frame.shape}")
cap.release()
EOF

# Analyze quality
file /tmp/camera_test.jpg
# Open in image viewer to check focus/exposure
```

**Solutions:**

1. **Camera needs focus adjustment:**
   ```bash
   # Focus ring on Camera Module v2
   # (Turn small lens adjustment nut carefully)
   
   # Or software focus (if enabled in config.txt)
   # camera_auto_focus=1
   ```

2. **Lighting too dim:**
   - Ensure bright ambient light
   - Add LED ring light if indoors
   - YOLOv5 was trained in well-lit conditions

3. **Image resolution too low:**
   - Current: 640x480 (good balance) - keep as is
   - For faster inference: reduce to 416x416

4. **YOLOv5 model undertrained:**
   - Current model mAP ≈ 96% on balloon dataset
   - If detections poor: retrain model with more diverse images

---

### Problem: Camera node crashes or high CPU load

**Symptoms:**
- YOLOv5 detection node crashes after few minutes
- CPU usage 100%, system becomes unresponsive
- Frame rate drops to < 5 fps

**Diagnosis:**
```bash
# Monitor CPU usage while detection running
top -b -n 1 | head -20

# Check system temperature
vcgencmd measure_temp

# Monitor ROS 2 node CPU
ps aux | grep detect
```

**Solutions:**

1. **YOLOv5 inference too slow on CPU:**
   - Use quantized/lightweight model (already implemented)
   - Reduce input resolution: 640x480 → 416x416
   - Skip frames: detect every 3rd frame instead of every frame

2. **System thermal throttling:**
   ```bash
   # Check CPU frequency (should be 1500 MHz)
   watch -n 0.1 'cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq'
   
   # If drops below 1200 MHz → overheating
   # Add heatsink or cooling fan to Pi
   ```

3. **Memory leak in detection node:**
   - Check for unclosed image buffers in C++ code
   - Ensure OpenCV Mat objects released

4. **Too many ROS 2 subscribers:**
   - Disable unused topics
   - Check if multiple copies of detect_node running

---

## Autonomy Issues

### Problem: Balloon detection not working / detections always zero

**Symptoms:**
- `/cam_data` topic has no detections (empty array)
- Even when balloon clearly visible on screen
- YOLOv5 confidence always < 0.5

**Diagnosis:**
```bash
# Check if detect_node running
ros2 node list | grep detect

# Monitor detections
ros2 topic echo /cam_data --once

# Check detection on raw image
# Capture frame and run YOLOv5 inference locally
python3 << 'EOF'
from ultralytics import YOLO
import cv2

model = YOLO('/path/to/yolov5s_balloon.pt')
cap = cv2.VideoCapture(0)
ret, frame = cap.read()

results = model(frame)
detections = results[0].boxes

for box in detections:
    if box.conf > 0.5:
        print(f"✓ Found: {box.cls} ({box.conf:.2f})")
print(f"Total detections: {len(detections)}")
cap.release()
EOF
```

**Root Causes & Solutions:**

1. **Target balloon color wrong or object not visible:**
   - Model trained on olive-colored balloons
   - Test with correct color balloon
   - Ensure good lighting

2. **YOLOv5 model not loaded:**
   - Check model path in detect_node.cpp
   - Verify .pt file exists and not corrupted
   - Test model with standalone script (above)

3. **Confidence threshold too high:**
   - Default: 0.5
   - Lower to 0.4 or 0.3 if legitimate detections missed
   - Edit in detect_node.cpp: `CONF_THRESHOLD`

4. **Model quantization issue:**
   - If using quantized model, ensure quantization correct
   - Try full-precision model for comparison

5. **Camera resolution mismatch:**
   - Model expects 640x480 input
   - Verify camera configured to capture this resolution

---

### Problem: Autonomous control not activating / mode switch not working

**Symptoms:**
- Mode stick on joystick does nothing
- Always stuck in MANUAL mode
- No autonomous behavior even when mode should activate

**Diagnosis:**
```bash
# Check mode_switcher running
ros2 node list | grep mode_switch

# Monitor mode input
ros2 topic echo /joy | head -30

# Check which branch being taken
ros2 topic echo /esc_input
# Should switch between /esc_manual_input and /esc_balloon_input

# Monitor control computation
ros2 topic hz /esc_balloon_input
# Should be 50 Hz when autonomous active
```

**Solutions:**

1. **Mode button not mapped correctly:**
   - Default mode button: RB (right shoulder)
   - Check joy message to find actual button ID
   - Edit mode_switcher.py:
     ```python
     MODE_SWITCH_BUTTON = 5  # RB button (adjust index if different)
     ```

2. **Mode switcher node not running:**
   ```bash
   ros2 run controls mode_switcher
   ```

3. **Autonomous control nodes not computing:**
   - Check inv_kine and pi_controller running
   - Monitor their output topics
   - Verify they're subscribed to sensor data

---

### Problem: Blimp oscillates wildly or diverges when autonomous

**Symptoms:**
- In autonomous mode, blimp spirals out of control
- Altitude oscillates > 1m
- Yaw overshoots badly

**Diagnosis:**
```bash
# Record flight data
ros2 bag record /imu_data /barometer_data /cam_data /esc_input &
# Perform autonomous test
# Stop recording: Ctrl-C

# Analyze
python3 << 'EOF'
from rosbag2_py import SequentialReader
import matplotlib.pyplot as plt

reader = SequentialReader()
# Load and plot altitude vs time, motor commands vs time
EOF
```

**Root Causes & Solutions:**

1. **PID gains not tuned for this hardware:**
   - Current gains tuned on reference platform
   - Your BLIMP may have different dynamics
   - Reduce Kp and Ki by factor of 2, test incrementally

2. **Control loop not closed properly:**
   - Verify sensor feedback connected to controller
   - Check barometer variance (noisy → filter needed)
   - Monitor state estimation lag

3. **Mode switching instability:**
   - When switching MANUAL → AUTONOMOUS, discontinuity in commands
   - Implement smooth ramping or integral wind-down

---

## Network Issues

### Problem: WiFi drops packets / disconnects

**Symptoms:**
- Frequent "No response to ping" messages
- ROS 2 communication delays spike
- Camera streaming stutters

**Diagnosis:**
```bash
# Check WiFi signal strength
iwconfig

# Ping gateway to test connectivity
ping 192.168.1.1 -c 100 | grep loss

# Monitor ROS 2 communication
ros2 topic hz /imu_data  # Should be steady at 200 Hz
# If variance high → network issue
```

**Solutions:**

1. **WiFi signal weak:**
   - Move closer to router
   - Check for interference (microwaves, cordless phones)
   - Use WiFi analyzer app to find less crowded channel
   - Upgrade to 5GHz if available

2. **WiFi adapter insufficient:**
   - Use USB WiFi dongle instead of onboard (often more powerful)
   - Ensure external antenna perpendicular to Pi

3. **Use Ethernet instead:**
   ```bash
   # If possible, USB Ethernet adapter more reliable
   sudo apt install dwc-otg-driver  # For USB gadget mode
   ```

4. **Reduce camera bandwidth:**
   - Lower resolution: 640x480 → 416x416
   - Lower framerate: 30fps → 15fps
   - Compress JPEG more

---

## System Stability Issues

### Problem: System reboots unexpectedly or freezes

**Symptoms:**
- Raspberry Pi suddenly loses power / reboots
- System becomes unresponsive
- SSH connection drops

**Diagnosis:**
```bash
# Check for power brownout
dmesg | tail -20  # Look for "Undervoltage" messages

# Check CPU temperature
vcgencmd measure_temp
# Should be < 80°C (ideal <60°C)

# Monitor voltage (with multimeter on GPIO pins)
# 5V rail should be 5.0V ± 0.1V
# 3.3V rail should be 3.3V ± 0.1V

# Check system logs for crash
journalctl -xe
```

**Solutions:**

1. **Power supply insufficient:**
   - Raspberry Pi needs 5V, 3A minimum
   - At peak load (WiFi + motors), can draw 4A
   - Upgrade to 5V/5A power supply
   - Use separate USB cable (not hub) if possible

2. **Battery voltage sag:**
   - When motors draw 5A peak, battery voltage drops
   - Check LiPo voltage under load
   - Use fresh, fully-charged battery
   - Consider 5000mAh → 10000mAh battery upgrade

3. **Thermal throttling:**
   - CPU temperature exceeds 80°C
   - Add heatsink to SoC (square chip on Pi)
   - Add 5V cooling fan blowing across heatsink
   - Ensure ventilation around Pi case

4. **Software memory leak:**
   - Monitor RAM usage during flight
   - Check for processes using 100% memory
   - Restart services periodically

---

### Problem: High CPU load / system slow

**Symptoms:**
- `top` command shows CPU > 80% consistently
- ROS 2 nodes responding slowly
- Delays in control loop (> 100ms)

**Diagnosis:**
```bash
# Check CPU usage by process
top -b -n 1 | head -20

# Check system load
uptime
# Load > 2.0 on 4-core system → overloaded

# Monitor ROS 2 node performance
ros2 stats /detect_cpp 2>&1 | head
```

**Solutions:**

1. **YOLOv5 inference consuming too much:**
   - Already using quantized model
   - Process every 3rd frame: `hz_reduce = 3`
   - Use lighter model (yolov5n instead of yolov5s)

2. **Too many ROS 2 nodes:**
   - Check `ros2 node list` for duplicates
   - Kill unnecessary nodes
   - Only launch needed subsystems

3. **I/O excessive (disk/network):**
   - Stop data logging
   - Disable ROS bag recording
   - Disable camera streaming

---

## Emergency Recovery

### Complete system reset (Last resort)

If system is unresponsive/corrupted:

```bash
# Quick reboot
sudo reboot

# Hard reboot (if SSH down)
# (Physically disconnect power for 10 sec, reconnect)

# Restore from backup (if available)
# Flash fresh Raspberry Pi OS and re-setup
```

### Kill hung processes

```bash
# Kill all ROS 2 processes
killall -9 ros2 detect_node esc_driver

# Or kill everything and restart from scratch
killall -9 python3
sudo systemctl restart pigpiod
```

---

## Getting Help

If issue persists:

1. **Check documentation:**
   - [README.md](README.md) - System overview
   - [INSTALLATION.md](INSTALLATION.md) - Setup guide
   - [NODES_REFERENCE.md](NODES_REFERENCE.md) - Node details
   - [GPIO_PIN_MAP.md](GPIO_PIN_MAP.md) - Hardware pinout

2. **Collect debug info:**
   ```bash
   # System info
   uname -a
   ros2 --version
   
   # Build info
   cat ~/blimp_ws/.colcon/profiles/default/metadata.json | grep -i version
   
   # Error logs
   dmesg | tail -50 > system_log.txt
   journalctl -xe > journal_log.txt
   
   # ROS 2 logs
   cat ~/.ros/log/latest/*/*/logger_all.log > ros_log.txt
   ```

3. **Report to team:**
   - Include: error message, steps to reproduce, logs
   - Reference: hardware setup, software version
   - Provide: photos if hardware issue

---

**Last Updated:** February 9, 2026  
**For detailed troubleshooting support, see [Full Technical Report](BLIMP_Handoff_Report_Final.md)**

# GPIO Pin Map - BLIMP Hardware Configuration

Complete pin assignments and interface documentation for the BLIMP platform.

---

## Raspberry Pi 4B GPIO Layout

```
 3V3  [●]  [●]  5V
 SDA  [●]  [●]  5V
 SCL  [●]  [●]  GND
GPIO4 [●]  [●]  GPIO14
GND   [●]  [●]  GPIO15
GPIO17[●]  [●]  GPIO18
GPIO27[●]  [●]  GND
GPIO22[●]  [●]  GPIO23
3V3   [●]  [●]  GPIO24
GPIO10[●]  [●]  GND
GPIO9 [●]  [●]  GPIO25
GPIO11[●]  [●]  GPIO8
GND   [●]  [●]  GPIO7
GPIO5 [●]  [●]  GND
GPIO6 [●]  [●]  GPIO12
GPIO13[●]  [●]  GND
GPIO19[●]  [●]  GPIO16
GPIO26[●]  [●]  GPIO20
GND   [●]  [●]  GPIO21
```

---

## Pin Assignments

### Motor Control (3 ESCs + 1 Unused)

| GPIO Pin | Function | Signal | Frequency | Duty Cycle Range | Motor Connection | Status | Purpose |
|----------|----------|--------|-----------|------------------|------|--------|----------|
| GPIO 5 | ESC 1 PWM | 1-3.3V logic | 50 Hz | 5-10% (1000-2000 µs) | Motor 1 | **ACTIVE** | Left motor |
| GPIO 6 | ESC 2 PWM | 1-3.3V logic | 50 Hz | 5-10% (1000-2000 µs) | Motor 2 | **ACTIVE** | Right motor |
| GPIO 13 | ESC 3 PWM | 1-3.3V logic | 50 Hz | 5-10% (1000-2000 µs) | Motor 3 | **ACTIVE** | Bottom motor (thrust) |
| GPIO 26 | ESC 4 PWM | 1-3.3V logic | 50 Hz | 5-10% (1000-2000 µs) | Motor 4 | **UNUSED** | Reserved for future use |

**Architecture:**
- Each GPIO pin controls one ESC via PWM signal
- Each ESC then controls one brushless/servo motor
- **Motors do NOT connect directly to GPIO** - they connect only to their ESC
- ESCs provide signal, power, and ground to motors

**PWM Specifications:**
- All PWM controlled via `pigpio` daemon
- Frequency: 50 Hz (20 ms period)
- Minimum pulse: 1000 µs (motor idle/off)
- Center: 1500 µs (motor stopped)
- Maximum pulse: 2000 µs (maximum thrust)
- Do NOT use GPIO without pigpiod running!

### I2C Bus (Sensors)

| GPIO Pin | Function | I2C Role | Speed | Voltage | Device |
|----------|----------|----------|-------|---------|--------|
| GPIO 2 | I2C_SDA | Data Line (Master) | 400 kHz | 3.3V* | IMU, Barometer |
| GPIO 3 | I2C_SCL | Clock Line (Master) | 400 kHz | 3.3V* | IMU, Barometer |

**I2C Device Addresses:**

| Device | Address | Alt Address | Configuration | Status |
|--------|---------|-------------|---------------|----- |
| BNO055 IMU | 0x28 | 0x29 | COM3 pin low | ✅ Active |
| BMP390 Barometer | 0x76 | 0x77 | SDO pin low | ✅ Active |

**I2C Wiring:**
- SDA: GPIO 2 → Level shifter (3.3V) → Sensor SDA (pulled up to 5V via 10kΩ)
- SCL: GPIO 3 → Level shifter (3.3V) → Sensor SCL (pulled up to 5V via 10kΩ)
- GND: Common ground between Pi and sensors

**Verify I2C:**
```bash
i2cdetect -y 1

# Expected output:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --  ← BNO055
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 70: -- -- -- -- -- -- 76 --                          ← BMP390
```

*Note: I2C data lines are 3.3V from Pi; level shifter converts to 5V for sensor compatibility

### Optional/Unused Pins (Reserved for Future)

| GPIO Pin | Potential Use | Notes |
|----------|---------------|----- |
| GPIO 25 | Gate Servo | Future cage gate actuation |
| GPIO 12 | PWM Output | Line following sensor (not implemented) |
| GPIO 16 | Emergency Cutoff | Safety relay control (not implemented) |
| GPIO 20 | Status LED | System status indicator (not implemented) |

---

## Power Distribution

### Voltage Rails

| Rail | Voltage | Source | Max Current | Devices |
|------|---------|--------|-------------|---------|
| **Battery (Unregulated)** | 11.1V (3S LiPo) | LiPo battery | 50A peak | ESC input, fuses |
| **USB Voltage Regulator** | 5V | USB buck converter | 3A | Raspberry Pi, sensors |
| **Pi 3.3V (GPIO)** | 3.3V | Pi internal | 500mA | GPIO pull-ups, logic |

### Power Budget by Subsystem

| Subsystem | Idle Current | Active Current | Peak Current | Notes |
|-----------|--------------|----------------|--------------|-------|
| Raspberry Pi | 600mA | 1200mA | 1500mA | Increases with WiFi |
| BNO055 IMU | 10mA | 20mA | 25mA | I2C current |
| BMP390 Barometer | 5mA | 8mA | 10mA | I2C current |
| Raspberry Pi Camera | 100mA | 300mA | 400mA | Video processing |
| 4x Servo Motors | 0mA | 2000mA | 4800mA | All motors at max |
| **TOTAL (Flight)** | **715mA** | **3508mA** | **7735mA** | Sustained: 2-4A |

### Battery Life Estimate

```
5A-hour LiPo battery
÷ 3A average current draw
= ~1.67 hours total capacity

Typical flight: 15-20 minutes per battery
(Rest of capacity used for ground testing)
```

---

## Connector Types & Pin Assignments

### Motor PWM Connectors

**ESCs (4x) to Motors:**
- Each ESC receives PWM signal from GPIO pin: 5, 6, 13, 26
- ESC Power (Red): +11.1V from battery → motor power
- ESC Ground (Black): Common GND → motor power return
- ESC Signal (Yellow/White): GPIO PWM signal from Pi (3.3V logic)

**Motor to ESC Connection:**
- Motor 1: Connects to ESC 1 (GPIO 5 PWM signal input)
- Motor 2: Connects to ESC 2 (GPIO 6 PWM signal input)
- Motor 3: Connects to ESC 3 (GPIO 13 PWM signal input)
- Motor 4: Connects to ESC 4 (GPIO 26 PWM signal input)
- **Motors receive power and signals ONLY from ESCs**, not directly from GPIO

### I2C Connector (Sensors)

**JST PH 4-pin connector (common on Adafruit breakouts):**
- Pin 1: VDD (5V, from USB regulator)
- Pin 2: GND (Common ground)
- Pin 3: SCL (GPIO 3, via level shifter)
- Pin 4: SDA (GPIO 2, via level shifter)

### Power Connectors

**Battery to ESC:**
- JST connector
- Red: +11.1V
- Black: GND

**USB Power to Raspberry Pi:**
- Micro-USB B: 5V, max 3A

---

## Software GPIO Mapping (pigpio)

### PWM Control Example (Python)

```python
import pigpio

# Initialize pigpio
pi = pigpio.pi()

# ESC PWM GPIO assignments (3 active motors, 1 reserved)
ESC_PINS = {
    1: 5,       # ESC 1 controls Motor 1 (LEFT)
    2: 6,       # ESC 2 controls Motor 2 (RIGHT)
    3: 13,      # ESC 3 controls Motor 3 (BOTTOM - vertical thrust)
    # 4: 26,    # ESC 4 GPIO 26 UNUSED - reserved for future use
}

FREQUENCY = 50  # Hz (20ms period for RC servos/ESCs)

# Set all ESCs to 1500 µs (neutral/idle)
for motor_id, gpio_pin in ESC_PINS.items():
    pi.hardware_PWM(gpio_pin, FREQUENCY, 1500000)  # 1500 µs pulse width
    print(f"Motor {motor_id} (GPIO {gpio_pin}): 1500 µs (idle)")

# Ramp Motor 1 from idle to 75% thrust over 2 seconds
for pwm_us in range(1500, 1700, 10):
    pi.hardware_PWM(ESC_PINS[1], FREQUENCY, pwm_us * 1000)
    time.sleep(0.05)  # 50ms delay between steps

print(f"Motor 1 at 75%: {pwm_us} µs")

# Stop all motors (set to 1000 µs - minimum safe level)
for motor_id, gpio_pin in ESC_PINS.items():
    pi.hardware_PWM(gpio_pin, FREQUENCY, 1000000)  # 1000 µs = off
    print(f"Motor {motor_id}: STOP (1000 µs)")

pi.stop()
```

---

## I2C Configuration

### Enable I2C on Raspberry Pi

**Configuration: `/boot/firmware/config.txt`**
```
dtparam=i2c_arm=on     # Enable I2C1 (GPIO 2/3)
dtparam=i2c1=on        # Alternative enable
```

### I2C Bus Details

| Parameter | Value |
|-----------|-------|
| **Bus Number** | 1 (I2C1 or /dev/i2c-1) |
| **Master Clock Speed** | 400 kHz (standard mode) |
| **Voltage** | 3.3V (logic), 5V (sensor power) |
| **Pull-up Resistors** | 10kΩ on SDA, SCL (onboard) |

### I2C Test Commands

```bash
# List I2C buses
i2cdetect -l

# Scan bus 1 for devices
i2cdetect -y 1

# Read single byte from BNO055 (address 0x28, register 0x00)
i2cget -y 1 0x28 0x00

# Write byte to device (example)
i2cset -y 1 0x28 0x00 0xFF
```

---

## Electrical Safety Notes

### ⚠️ Critical Warnings

1. **Motor/ESC Control:**
   - Only GPIO 5, 6, 13, 26 should control ESCs
   - Each ESC receives PWM signal from one GPIO pin
   - Motors connect only to their ESC, NOT to GPIO
   - Never mix 5V and 3.3V signals without level shifter
   - Always verify pigpiod daemon is running before PWM

2. **I2C Bus:**
   - Level shifter is REQUIRED between Pi (3.3V) and 5V sensors
   - Pull-up resistors reduce I2C frequency if too weak (10kΩ typical)
   - Bus conflicts cause I2C lockup; restart I2C service

3. **Power Distribution:**
   - Battery directly powers ESC (no 5V regulator)
   - USB regulator powers Pi and sensors
   - Separate fusing for ESC (30A) and USB branch (5A)
   - Common ground REQUIRED between battery and Pi

4. **GPIO Limits:**
   - Absolute maximum source/sink per pin: 16mA
   - Total GPIO current (all pins): 50mA
   - PWM via pigpio does NOT increase current draw

### Recommended Fusing

```
Battery
    ↓
[Main Fuse: 50A]
    ↓
    ├→ [ESC 50A]
    │
    └→ [USB Voltage Regulator]
        ↓
        [5A Fuse]
        ↓
        Raspberry Pi + Sensors
```

---

## Troubleshooting GPIO Issues

### ESC/Motor PWM Not Working

```bash
# 1. Verify pigpiod is running
sudo systemctl status pigpiod
# Should show: Active (running)

# 2. Restart pigpiod if needed
sudo systemctl restart pigpiod

# 3. Check GPIO permissions
ps aux | grep pigpiod
# Should show process running as root

# 4. Test ESC 1 (GPIO 5) with command-line tool
pigs hwm 5 50 1500000   # Set ESC 1 to neutral (1500 µs)
# If motor 1 moves or changes pitch, GPIO 5 is working

# 5. Test all ESC pins
pigs hwm 5 50 1500000   # ESC 1 (Motor 1)
pigs hwm 6 50 1500000   # ESC 2 (Motor 2)
pigs hwm 13 50 1500000  # ESC 3 (Motor 3)
pigs hwm 26 50 1500000  # ESC 4 (Motor 4)

# 6. Verify ESC connections
# - Yellow signal wire from ESC → GPIO pin
# - Red power wire from ESC → Battery XT90
# - Black ground wire from ESC → Battery GND (also connect to Pi GND)
```

### I2C Bus Issues

```bash
# 1. Check config.txt
grep "dtparam=i2c" /boot/firmware/config.txt

# 2. Scan I2C bus
i2cdetect -y 1

# 3. If no devices found:
# - Check connections (loose wires)
# - Verify level shifter installation
# - Power cycle sensors

# 4. Reset I2C if hung
sudo systemctl restart i2c.service
```

## Pin Assignment Summary Table

| GPIO | Function | Type | Controls | Address | Status |
|------|----------|------|----------|---------|--------|
| 2 | I2C SDA | I2C Master | Sensors | /dev/i2c-1 | ✅ Active |
| 3 | I2C SCL | I2C Master | Sensors | /dev/i2c-1 | ✅ Active |
| 5 | ESC 1 PWM | PWM Output | Motor 1 | N/A | ⏸️ Unused |
| 6 | ESC 2 PWM | PWM Output | Motor 2 | N/A | ✅ Active |
| 13 | ESC 3 PWM | PWM Output | Motor 3 | N/A | ✅ Active |
| 26 | ESC 4 PWM | PWM Output | Motor 4 | N/A | ✅ Active |
| 25 | (Reserved) | - | - | - | ⏸️ Unused |
| 12 | (Reserved) | - | - | - | ⏸️ Unused |

---

**Last Updated:** February 9, 2026  
**Release Tag:** `platform_report_2025_02`

# Hardware Documentation - BLIMP Platform

This directory contains all hardware-related documentation, designs, and manufacturing files for the BLIMP autonomous platform.

## Directory Structure

```
hardware/
├── README.md                          # This file
├── cad/                               # 3D CAD models and mechanical designs
├── electronics/                       # Electrical schematics and PCB layouts
├── wiring/                            # Wiring diagrams and connection documentation
└── assembly/                          # Assembly instructions and build guides
```

---

## CAD Models (`cad/`)

### Contents
- **3D Models:** STEP files (.SLDASM, .SLDPRT), STL files for 3D printing
- **Vector Graphics:** SVG files for 2D component drawings
- **Assembly Designs:** Full system assemblies and component references
- **2D Technical Drawings:** DXF files for manufacturing and reference

### Current Files
```
cad/
├── Assemblies (SLDASM files):
│   ├── 400gElliptic.SLDASM            # 400g balloon envelope assembly
│   ├── Attacker(6-24).SLDASM          # Attacker configuration
│   ├── GondolaSummer2024.SLDASM       # Summer 2024 gondola assembly
│   └── PropulsionSide2024.SLDASM      # 2024 propulsion subsystem
│
├── Parts (SLDPRT files):
│   ├── 400gElliptic.SLDPRT            # Main envelope body
│   ├── 4mmx3mmCFTube.SLDPRT           # Carbon fiber tube stock
│   ├── 6x3 prop.SLDPRT                # Propeller part
│   ├── CFTubeBent.SLDPRT              # Bent CF tube component
│   ├── CrossConnector.SLDPRT          # Cross-shaped connector
│   ├── electronics.SLDPRT             # Electronics component
│   ├── FinnToBlimp.SLDPRT             # Fin attachment bracket
│   ├── FoamFinn.SLDPRT                # Foam fin design
│   ├── holedplate.SLDPRT              # Mounting plate with holes
│   ├── MotorPlaceholder.SLDPRT        # Motor mount placeholder
│   Design Reference
All SLDASM files represent complete assemblies showing how parts fit together. The SLDPRT files are individual components that can be:
- Viewed for reference
- Exported to STEP format for sharing
- Exported to STL format for 3D printing
- Modified using SolidWorks or equivalent CAD software
├── 3D Printable Files (STL):
│   ├── CrossConnector.STL             # 3D printer ready version
│   ├── RodToBlimp.STL                 # Rod assembly for printing
│   ├── RodToBlimp_2mm.STL             # Thin variant for printing
│   ├── RodToMotor.STL                 # Motor rod for printing
│   └── RodToMotor1.STL                # Motor rod alternate version
│
├── Vector Graphics (SVG):
│   ├── FinnToBlimp.svg                # Fin design schematic
│   └── FoamFinn.svg                   # Foam fin vector drawing
│
└── DXF/ (Technical Drawings)
    └── [2D manufacturing and reference drawings]
```

### Software Requirements
- **SLDASM/SLDPRT files:** SolidWorks 2020+ or Fusion 360
- **STL files:** 3D printing software (Cura, PrusaSlicer, IdeaMaker)
- **SVG files:** Inkscape, Adobe Illustrator, or any vector graphics editor
- **DXF files:** AutoCAD, LibreCAD, or CNC software

### Software Requirements
- **STEP/IGES files:** Fusion 360, SolidWorks, FreeCAD, OpenSCAD
- **STL files:** 3D printing software (Cura, PrusaSlicer, IdeaMaker)
- **PDF drawings:** Any PDF viewer

### Fusion 360 Project
If using Fusion 360, all CAD files can be imported from the cloud workspace for collaborative editing.

---

## Datasheets (`Datasheets/`)

### Current Component Documentation
```
Datasheets/
├── Sensors:
│   ├── BNO055 Datasheet.pdf           # 9-DOF IMU (Inertial Measurement Unit)
│   ├── adafruit-bmp388-bmp390-bmp3xx.pdf  # BMP390 Barometer & Altimeter
│   ├── 4Gore_EllipticalBalloon_2m.pdf # Balloon specifications (4-gore design)
│   └── 20240530_400g_Lift.pdf         # Lift characteristics for 400g balloon
│
├── Components:
│   └── 20200920_SB-279-250.pdf        # Component specification document
│
└── [Additional datasheets to be added]
```

### Key Sensor Specifications
- **BNO055 IMU:** 9-axis (accel, gyro, mag), I2C interface, 0x28 address
- **BMP390 Barometer:** Pressure sensing, altitude calculation, I2C/SPI, 0x76 address
- **Balloon:** 4-gore elliptical, 400g lift capacity, ~2 meter diameter

### Files to Extract from Google Drive
- ESC (Electronic Speed Controller) datasheets
- Raspberry Pi 4B detailed specification
- Motor/servo specifications
- Battery characteristics (3S LiPo voltage/discharge curves)
- Voltage regulator technical documentation

---

## Electronics (`electronics/`)

### Status
Currently empty - reserved for future electrical design documentation.

---

## PCB Design (`PCB design/`)

### Current Projects
```
PCB design/
├── Actuated_Fin/                      # Fin actuation PCB project
├── New Blimps PCB/                    # Updated platform PCB design
└── Raspberry Pi 4B PCB/               # Pi 4B integration board
```

### Applications
- **Actuated Fin:** Control PCB for fin servo actuation
- **New Blimps PCB:** Main custom PCB for sensor integration and power distribution
- **Raspberry Pi 4B PCB:** Breakout or expansion board for GPIO and I2C access
---

## ESC Programmer (`ESC programmer/`)

### Current Tools
```
ESC Programmer/
└── BLHeliSuite32_32.10.0.0/           # ESC firmware programming software
    └── [BLHeli Suite application files]
```

### Purpose
BLHeli Suite 32 is used to:
- Program Electronic Speed Controllers (ESCs) with custom firmware
- Configure motor parameters (rotation direction, braking, cutoff voltage, etc.)
- Set up multi-rotor configurations for our 3-motor setup
- Debug ESC performance and monitor telemetry

### Usage
1. Connect ESC programmer cable to ESC
2. Launch BLHeliSuite32_32.10.0.0
3. Select connected ESC (via USB programmer)
4. Load configuration for BLIMP's 3-motor layout:
   - Motor 1 (GPIO 5): Left - Yaw control
   - Motor 2 (GPIO 6): Right - Yaw control
   - Motor 3 (GPIO 13): Bottom - Vertical thrust
   - Motor 4 (GPIO 26): Reserved/unused
5. Apply firmware update or configuration changes
6. Verify motor response

**Configuration Files to Add:**
- Default BLIMP ESC configuration (.hex)
- Motor mapping profiles for 3-motor configuration
- Safe startup parameters
- Power limit settings for 11.1V 3S LiPo

---

## Wiring Diagrams (`wiring/`)

### Status
Currently empty - reserved for wiring and connection documentation.

**Planned Content:**
- Complete system wiring diagram
- Motor-to-ESC connections (3-motor configuration)
- I2C sensor wiring (BNO055, BMP390)
- Camera ribbon routing
- Power distribution wiring
- GPIO pin assignments and connector pinouts
- Testing procedures and multimeter measurement points

**Critical Wiring Notes:**

⚠️ **Safety:**
- Always disconnect battery before working on wiring
- Verify polarity (+ and -) before connecting high-current circuits
- Use appropriate wire gauges (see recommendations below)
- All high-current wires should be insulated and protected from abrasion

**Wire Gauge Recommendations:**
- Battery to 30A fuse: 10 AWG (6mm²) - supports 40A continuous
- Fuse to motor subsystem: 12 AWG (4mm²) - supports 25A
- Fuse to 5V regulator: 14 AWG (2.5mm²) - supports 10A
- I2C signal wires: 22 AWG (0.5mm²)
- PWM signal wires: 22 AWG (0.5mm²)

**Connector Types:**
- Battery: XT90-S Antileak (high current, safety switch)
- Motor power: Tamiya connectors or JST XT60
- I2C sensors: JST PH mini connectors (2.0mm pitch)
- PWM signals: 3-pin servo headers (0.1" pitch)
- Camera: CSI flat ribbon (15-pin specialized connector)

**Files to be added:**
- complete_wiring_diagram.pdf
- motor_wiring_diagram.pdf
- sensor_connections.pdf
- power_distribution.pdf
- troubleshooting_continuity_guide.txt

---

## Assembly Instructions (`assembly/`)

### Status
Currently empty - reserved for assembly guidance and build procedures.

**Planned Content:**
- Complete assembly manual with step-by-step photos
- Bill of Materials (BOM) with part sources and quantities
- Tools and equipment required for assembly
- Mechanical assembly procedures
- Electronics integration guide
- System testing and calibration procedures
- Safety inspection checklists

**Files to be added:**
- complete_assembly_manual.pdf
- bill_of_materials.xlsx
- tools_required.txt
- mechanical_assembly_guide.pdf
- electronics_integration_guide.pdf
- sensor_calibration_procedure.pdf

---

## Using the Hardware Documentation

### Current Available Resources
- **CAD Models:** All mechanical and component designs in SLDASM/SLDPRT format
- **Datasheets:** Key sensor specifications and component documentation
- **PCB Design Projects:** Directory structure for future PCB designs
- **ESC Programming:** BLHeli Suite for ESC configuration

### For New Builders
1. Start with **cad/** folder to understand mechanical design
2. Reference **Datasheets/** for component specifications:
   - BNO055 IMU datasheet (I2C address: 0x28)
   - BMP390 Barometer datasheet (I2C address: 0x76)
   - Balloon lift characteristics
3. Use [**/blimp_src/INSTALLATION.md*](../blimp_src/INSTALLATION.md) in parent directory for software setup
4. Check [**GPIO_PIN_MAP.md**](../blimp_src/GPIO_PIN_MAP.md) for pinout reference
5. Follow **Step 12: Xbox Controller Setup** in INSTALLATION.md for manual mode

### For Electrical Engineers
1. Review **PCB design/** folder structure for custom board projects
2. Consult **ESC programmer/** for BLHeli Suite configuration templates
3. Check datasheets for:
   - I2C sensor interfacing
   - 3-motor ESC configuration
   - Power budget calculations

### For Troubleshooting
1. Cross-reference component connections with [**GPIO_PIN_MAP.md**](../blimp_src/GPIO_PIN_MAP.md)
2. Consult [**TROUBLESHOOTING.md**](../blimp_src/TROUBLESHOOTING.md) for known issues
3. Verify sensor I2C addresses and wiring against datasheets
4. Check power distribution per specifications in datasheets

### For Design Modifications
1. Review current CAD designs in **cad/** folder
2. Consult component datasheets in **Datasheets/** folder
3. Update **PCB design/** projects if modifying electrical layout
4. Update **ESC programmer/** configuration if changing motor assignments
5. Document changes and update this README with version numbers and dates

---

## Source Material & Future Files

### Currently Imported Files
- **CAD Models:** All SolidWorks SLDASM/SLDPRT files with STL and SVG exports
- **Datasheets:** BNO055, BMP390, balloon, and component specifications
- **ESC Programmer:** BLHeli Suite 32 v32.10.0.0

### Files Ready to Be Added
The following documentation should be added to this hardware directory:

**From Google Drive - Electronics:**
- Main power distribution schematic (PDF)
- Sensor breakout board designs (if custom PCBs exist)
- I2C level shifter circuit details
- ESC wiring diagrams
- Power budget analysis spreadsheet

**From Google Drive - Wiring:**
- Complete system wiring diagram
- Motor-to-ESC connection diagram
- I2C sensor wiring guide
- Camera CSI ribbon routing
- Power distribution wiring schematic

**From Google Drive - Assembly:**
- Step-by-step assembly manual with photos
- Bill of Materials (BOM) with part numbers and sources
- Mechanical assembly procedures
- Electronics integration guide
- Sensor calibration procedures

**Generated Files (To Create):**
- 3-motor configuration guide (for updated propulsion system)
- Safety inspection checklists
- Testing procedures and verification checklists
- CAD assembly instructions and exploded views

---

## Contributing & Updates

When updating hardware documentation:
1. Place files in appropriate subdirectory
2. Use descriptive filenames with version: `motor_bracket_v2.step`
3. Include PDF exports for documentation files
4. Update this README with summary of changes
5. Add dates to version numbers: `wiring_diagram_2025_02_09.pdf`

---

**Last Updated:** February 9, 2026  
**Repository:** https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU  
**Related Documentation:** See `blimp_src/` for software, `GPIO_PIN_MAP.md` for hardware pinouts, `TROUBLESHOOTING.md` for hardware issues

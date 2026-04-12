# TrailCurrent Plateau

Vehicle leveling module that monitors trailer tilt angles and computes per-corner height adjustments via a CAN bus interface with OTA firmware update capability. Part of the [TrailCurrent](https://trailcurrent.com) open-source vehicle platform.

## Hardware Overview

- **Microcontroller:** ESP32-S3
- **IMU:** Adafruit BNO055 9-DOF Absolute Orientation
- **Function:** Vehicle leveling with CAN bus communication
- **Key Features:**
  - BNO055 IMU for pitch/roll measurement with auto-calibration
  - Per-corner height adjustment calculation from vehicle dimensions
  - Three mounting orientations (floor, left wall, right wall) with axis remapping
  - CAN bus communication at 500 kbps
  - Vehicle configuration (dimensions, mounting) received via CAN and stored in NVS
  - Over-the-air (OTA) firmware updates via WiFi (credentials provisioned via CAN)
  - mDNS-based device discovery for Headwaters integration
  - Custom flash partition layout with dual OTA slots

## Hardware Requirements

### Components

- **Microcontroller:** [Waveshare ESP32-S3-Zero](https://www.waveshare.com/esp32-s3-zero.htm?aff_id=Trailcurrent) — selected for its extensive documentation, small footprint, pre-soldered programming pins, castellations for direct PCB integration, and low power consumption
- **CAN Transceiver:** SN65HVD230
- **Power:** Buck converter (12V step-down)
- **Connectors:** JST XH 2.54 4-pin

### Pin Connections

| GPIO | Function |
|------|----------|
| 1 | I2C SDA (BNO055) |
| 2 | I2C SCL (BNO055) |
| 7 | CAN TX |
| 8 | CAN RX |

### KiCAD Library Dependencies

This project uses the consolidated [TrailCurrentKiCADLibraries](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries).

**Setup:**

```bash
# Clone the library
git clone git@github.com:trailcurrentoss/TrailCurrentKiCADLibraries.git

# Set environment variables (add to ~/.bashrc or ~/.zshrc)
export TRAILCURRENT_SYMBOL_DIR="/path/to/TrailCurrentKiCADLibraries/symbols"
export TRAILCURRENT_FOOTPRINT_DIR="/path/to/TrailCurrentKiCADLibraries/footprints"
export TRAILCURRENT_3DMODEL_DIR="/path/to/TrailCurrentKiCADLibraries/3d_models"
```

See [KICAD_ENVIRONMENT_SETUP.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/KICAD_ENVIRONMENT_SETUP.md) in the library repository for detailed setup instructions.

## Opening the Project

1. **Set up environment variables** (see Library Dependencies above)
2. **Open KiCAD:**
   ```bash
   kicad EDA/TrailCurrentPlateau/TrailCurrentPlateau.kicad_pro
   ```
3. **Verify libraries load** - All symbol and footprint libraries should resolve without errors
4. **View 3D models** - Open PCB and press `Alt+3` to view the 3D visualization

## Firmware

ESP-IDF native firmware in the `main/` directory.

**Prerequisites:**
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/) v4.1 or later

**Build and flash:**
```bash
# Set ESP-IDF target
idf.py set-target esp32s3

# Build firmware
idf.py build

# Upload to board (serial)
idf.py flash

# Monitor serial output
idf.py monitor

# Build, flash, and monitor in one command
idf.py build flash monitor
```

**OTA update (after initial flash):**
```bash
curl -X POST http://esp32-XXYYZZ.local/ota --data-binary @build/plateau.bin
```

### Firmware Architecture

The firmware uses FreeRTOS tasks for concurrent operation:

| Task | Priority | Stack | Function |
|------|----------|-------|----------|
| twai | 5 | 4096 | CAN bus RX/TX, leveling computation, IMU reads |
| ota | 3 | 8192 | OTA update (spawned on demand, non-blocking) |
| discovery | 3 | 8192 | mDNS discovery (spawned on demand, non-blocking) |

OTA and discovery run as separate tasks to avoid blocking CAN bus communication. Mutual exclusion prevents both from running simultaneously.

### Source Files

| File | Purpose |
|------|---------|
| `main.c` | Application entry, TWAI task, leveling logic, CAN protocol |
| `bno055.c/h` | BNO055 IMU driver (bare I2C, no Arduino dependency) |
| `wifi_config.c/h` | WiFi credentials, connect/disconnect, CAN provisioning |
| `ota.c/h` | Task-based OTA with HTTP server and mDNS |
| `discovery.c/h` | Task-based mDNS discovery for Headwaters |

### CAN Bus Protocol

- **Receive ID `0x00`:** OTA update trigger — initiates firmware update when device MAC matches
- **Receive ID `0x01`:** WiFi credential provisioning — multi-frame protocol to store SSID/password in NVS
- **Receive ID `0x02`:** Discovery trigger — joins WiFi, advertises via mDNS, waits for Headwaters confirmation
- **Transmit ID `0x04`:** Firmware version report — sent once on boot: `[mac3, mac4, mac5, major, minor, patch]`
- **Receive ID `0x20`:** Leveling configuration — set mounting surface, vehicle length/width, persist to NVS
- **Transmit ID `0x30`:** Tilt data — pitch/roll angles (×100) and front-back/left-right height diffs in mm
- **Transmit ID `0x31`:** Corner data — per-corner height adjustments in mm (normalized, lowest = 0)
- **Transmit ID `0x32`:** Status data — IMU connection, calibration levels, mounting orientation
- CAN speed: 500 kbps

### WiFi Credentials

WiFi credentials are provisioned over the CAN bus (ID `0x01`) and stored in NVS. No hardcoded credentials or secrets files needed.

## Manufacturing

- **PCB Files:** Ready for fabrication via standard PCB services (JLCPCB, OSH Park, etc.)
- **BOM Generation:** Export BOM from KiCAD schematic (Tools > Generate BOM)
- **JLCPCB Assembly:** See [BOM_ASSEMBLY_WORKFLOW.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/BOM_ASSEMBLY_WORKFLOW.md) for detailed assembly workflow

## Project Structure

```
├── EDA/                          # KiCAD hardware design files
│   └── TrailCurrentPlateau/
│       ├── *.kicad_pro           # KiCAD project
│       ├── *.kicad_sch           # Schematic
│       └── *.kicad_pcb           # PCB layout
├── CAD/                          # FreeCAD PCB enclosure design
├── main/                         # ESP-IDF firmware source
│   ├── main.c                    # Application entry, TWAI task, leveling logic
│   ├── bno055.c/h                # BNO055 IMU I2C driver
│   ├── wifi_config.c/h           # WiFi credentials and CAN provisioning
│   ├── ota.c/h                   # OTA firmware update (task-based)
│   ├── discovery.c/h             # mDNS device discovery (task-based)
│   ├── CMakeLists.txt            # Component build configuration
│   └── idf_component.yml         # ESP-IDF component dependencies
├── CMakeLists.txt                # Root ESP-IDF project file
├── sdkconfig.defaults            # ESP-IDF build defaults
├── partitions.csv                # ESP32 flash partition layout (dual OTA)
├── LICENSE                       # MIT License
└── README.md                     # This file
```

## License

MIT License - See LICENSE file for details.

This is open source hardware. You are free to use, modify, and distribute these designs for personal or commercial purposes.

## Contributing

Improvements and contributions are welcome! Please submit issues or pull requests.

## Support

For questions about:
- **KiCAD setup:** See [KICAD_ENVIRONMENT_SETUP.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/KICAD_ENVIRONMENT_SETUP.md)
- **Assembly workflow:** See [BOM_ASSEMBLY_WORKFLOW.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/BOM_ASSEMBLY_WORKFLOW.md)

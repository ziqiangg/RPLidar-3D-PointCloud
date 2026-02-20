# RPLidar 3D Point Cloud

Cross-platform Python toolkit for capturing and processing RPLidar scans into 2D/3D point cloud data.

## Quick Start

### 1. Set Up Virtual Environment (Recommended)
```bash
# Create virtual environment with Python 3.12
python3.12 -m venv venv

# if you have multiple python versions installed use
py -3.12 -m venv venv

.\venv\Scripts\Activate.ps1; python --version

# Activate the virtual environment
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate
```

### 2. Install Dependencies
```bash
# Upgrade pip
python.exe -m pip install --upgrade pip

pip install -r requirements.txt
```

### 3. Find Your RPLidar Port
```bash
python examples/select_port.py
```
This auto-detects your RPLidar and shows available serial ports.

### 4. Capture a Scan
```bash
# Auto-detect port
python dump_one_scan.py

# Or specify port manually
python dump_one_scan.py COM3        # Windows
python dump_one_scan.py /dev/ttyUSB0  # Linux
```

Output: `data/scan.csv` and `data/scan.ply`

### 5. Visualize Scans with GUI 
```bash
# Install viewer dependencies (includes open3d)
pip install -r requirements.txt

# Launch file viewer (browse and load any PLY/CSV)
python viewer/app.py

# Load specific file on startup
python viewer/app.py data/scan.ply
```

Features:
- Interactive 3D visualization with Open3D
- File browser - load any PLY or CSV point cloud
- Control RPLidar via USB (CP210 controller)
- Real-time scan execution and monitoring
- Adjustable point size and rendering options
- Cross-platform (Windows/Linux/macOS)




## Project Structure

```
RPLidar-3D-PointCloud/
├── dump_one_scan.py      # Main: Capture single 2D scan → CSV/PLY
├── xyzscan_servoless.py  # 3D scan with servo control
├── viewer/               # GUI Application
│   ├── app.py           # Main GUI - file browser + 3D view
│   ├── point_cloud_loader.py # PLY/CSV file loading
│   ├── scan_controller.py    # Scan script execution
│   ├── config.py         # Configuration settings
├── utils/
│   └── port_config.py    # Cross-platform port detection
├── examples/
│   ├── rplidarTest.py    # Test connection & view live scans
│   └── select_port.py    # Interactive port selector
├── data/                 # Generated scan files (git-ignored)
└── requirements.txt
```

## Script Reference

### Main Scripts

**`dump_one_scan.py`**  
Captures one complete 360° scan and saves to CSV/PLY format.
- Auto-detects serial port (Windows/Linux/macOS)
- Outputs: X/Y coordinates in meters + quality/angle/distance
- Files: `data/scan.csv`, `data/scan.ply`

**`xyzscan_servoless.py`**  
*(Work in progress)* 3D point cloud generation with servo tilt control.

### Examples

**`examples/rplidarTest.py`**  
Connection test script - displays 6 scans with distance statistics.
```bash
python examples/rplidarTest.py [PORT]
```

**`examples/select_port.py`**  
Interactive port selection tool with auto-detection.
```bash
python examples/select_port.py
```

### Utilities

**`utils/port_config.py`**  
Cross-platform serial port detection module.
- `get_default_port()` - Auto-detect or return OS default
- `find_rplidar_port()` - Search for USB-serial devices
- `get_available_ports()` - List all serial ports
- `select_port_interactive()` - User selection prompt

## Platform Notes

### Windows
- Ports: `COM3`, `COM4`, `COM5`, etc.
- Check Device Manager → Ports (COM & LPT)
- Example: `python dump_one_scan.py COM3`

### Linux
- Ports: `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.
- May need permissions: `sudo usermod -a -G dialout $USER`
- Example: `python dump_one_scan.py /dev/ttyUSB0`

### macOS
- Ports: `/dev/cu.usbserial-*` or `/dev/cu.usbmodem-*`
- Example: `python dump_one_scan.py /dev/cu.usbserial-0001`

## Output Formats

**CSV Format** (`data/scan.csv`)
```
quality,angle_deg,distance_mm,x_m,y_m,z_m
15,0.50,1234.0,1.234,0.010,0.0
```

**PLY Format** (`data/scan.ply`)  
ASCII point cloud - compatible with MeshLab, CloudCompare, etc.

## Troubleshooting

**Port not found?**  
Run `python examples/select_port.py` to see available ports.

If using Windows, check Device Manager, if CP2102 Driver not found in Ports, check Other devices and update driver. For detailed instructions, refer to [DRIVER_INSTALL.md](DRIVER_INSTALL.md).

**Permission denied (Linux)?**  
```bash
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group (requires logout)
sudo usermod -a -G dialout $USER
```

**No data received?**  
- Check USB connection
- Verify correct port
- Ensure no other program is using the port
- Check RPLidar power/motor spin-up

## Requirements

- Python 3.12
- rplidar-roboticia (RPLidar communication)
- pyserial (serial port detection)
- open3d (3D visualization)
- numpy (numerical operations)
- paho-mqtt (MQTT communication for viewer)

All dependencies can be installed via:
```bash
pip install -r requirements.txt
```

## License

MIT License

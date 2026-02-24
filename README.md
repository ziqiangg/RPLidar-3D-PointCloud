# RPLidar 3D Point Cloud - Distributed Scanning System

**Distributed RPLidar scanning system with Raspberry Pi scanner service and laptop GUI client communicating via MQTT.**

## 🎯 System Architecture

```
┌──────────────────────────────────┐        ┌───────────────────────────────┐
│ LAPTOP (Windows)                 │        │ RASPBERRY PI 5                │
│ IP: 10.197.211.141              │◄──────►│                               │
│                                  │  MQTT  │                               │
│ ├─ Mosquitto Broker (1883)      │        │ ├─ RPLidar Scanner Service    │
│ ├─ MQTT Viewer Client            │        │ ├─ RPLidar A1/A2 (USB)       │
│ └─ Open3D GUI (Visualization)    │        │ └─ dump_one_scan.py           │
└──────────────────────────────────┘        └───────────────────────────────┘
```

**How It Works:**
1. **Laptop GUI** publishes scan command to MQTT broker → `rplidar/commands/scan`
2. **Raspberry Pi** subscribed to commands, receives request and executes `dump_one_scan.py`
3. **RPi** scans environment with RPLidar (360° 2D scan), generates CSV/PLY files
4. **RPi** publishes scan data in 256KB chunks to MQTT → `rplidar/data/<scan_id>`
5. **Laptop** receives chunks, reassembles files, loads into Open3D for visualization

**Key Features:**
- ✅ **Distributed Architecture**: RPLidar on RPi, visualization on laptop
- ✅ **MQTT Communication**: Reliable QoS 1 message delivery
- ✅ **Network Scanning**: No USB connection to laptop needed
- ✅ **Chunked Transfer**: Large files sent in 256KB chunks
- ✅ **Auto-reassembly**: Files automatically saved to `data/` folder
- ✅ **GUI Integration**: Seamless Open3D visualization

---

## 📋 Requirements

### Hardware
- **Raspberry Pi 5** (or RPi 4/3B+) with WiFi/Ethernet
- **RPLidar A1 or A2** with USB connection
- **Laptop** (Windows/Linux/macOS) on same network

### Software
- **Python 3.9+** (both laptop and RPi)
- **Mosquitto MQTT Broker** (laptop only)
- **MQTT Explorer** (optional, for debugging)

### Network
- Both devices on same network (WiFi/LAN)
- Laptop must have static IP or known hostname
- Port 1883 (MQTT) open on laptop firewall

---

## 🚀 Quick Start

**For complete setup instructions, see [README_SETUP.md](README_SETUP.md)**

### Laptop Setup (5 minutes)
1. Install Mosquitto MQTT broker
2. Configure firewall (port 1883)
3. Install Python dependencies
4. Edit `config_laptop.yaml`

### Raspberry Pi Setup (10 minutes)
1. Clone repository
2. Install Python dependencies (minimal, no GUI)
3. Edit `config_rpi.yaml` with laptop IP
4. Run scanner service

### First Scan Test
```bash
# On RPi terminal:
python rpi_scanner_service.py

# On laptop:
python viewer/app.py
# Click "Start Scan" button in GUI
```

---

## 📁 Project Structure
```
RPLidar-3D-PointCloud/
├── mqtt_protocol/              # Shared MQTT protocol layer
│   ├── __init__.py
│   ├── topics.py              # Topic definitions
│   ├── messages.py            # Message schemas (JSON)
│   └── client_base.py         # Base MQTT client
│
├── rpi_scanner_service.py     # Raspberry Pi: Main scanner service
├── laptop_viewer_client.py    # Laptop: MQTT client for GUI
│
├── config_rpi.yaml            # RPi configuration (broker IP, port)
├── config_laptop.yaml         # Laptop configuration (data dir)
│
├── requirements_rpi.txt       # RPi dependencies (minimal, no GUI)
├── requirements_laptop.txt    # Laptop dependencies (full, with Open3D)
│
├── viewer/                    # GUI Application (laptop only)
│   ├── app.py                # Main GUI - file browser + 3D view
│   ├── scan_controller.py    # MQTT-based scan control
│   ├── point_cloud_loader.py # PLY/CSV file loading
│   └── config.py             # GUI configuration
│
├── dump_one_scan.py          # Core scanning script (RPi)
├── xyzscan_servoless.py      # [DEPRECATED] 3D scanning
│
├── utils/
│   └── port_config.py        # Serial port detection
│
├── examples/
│   ├── rplidarTest.py        # RPLidar connection test
│   └── select_port.py        # Port selection utility
│
└── data/                     # Scan output directory
    ├── scan.csv              # Latest 2D scan (CSV)
    └── scan.ply              # Latest 2D scan (PLY)
```

---

## 📡 MQTT Communication Protocol

### Topics

| Topic | Publisher | Subscriber | Purpose |
|-------|-----------|------------|---------|
| `rplidar/commands/scan` | Laptop | RPi | Scan command requests |
| `rplidar/status/<scan_id>` | RPi | Laptop | Scan status updates |
| `rplidar/data/<scan_id>` | RPi | Laptop | Chunked file transfer |

### Message Flow

**1. Scan Request (Laptop → RPi)**
```json
{
  "scan_id": "scan_20260224_143022",
  "scan_type": "2d",
  "port": "/dev/ttyUSB0",
  "timestamp": 1708783822.5
}
```

**2. Status Update (RPi → Laptop)**
```json
{
  "scan_id": "scan_20260224_143022",
  "status": "completed",
  "message": "Scan completed successfully: 1234 points",
  "timestamp": 1708783835.2,
  "point_count": 1234,
  "files": ["scan.ply", "scan.csv"]
}
```

**3. Data Transfer (RPi → Laptop)**
```json
{
  "scan_id": "scan_20260224_143022",
  "filename": "scan.ply",
  "chunk_index": 0,
  "total_chunks": 3,
  "data": "<base64_encoded_chunk>",
  "timestamp": 1708783835.5
}
```

### QoS & Reliability
- **QoS 1** (At least once delivery) for all messages
- **Chunked transfer**: 256KB chunks with base64 encoding
- **Auto-reassembly**: Files reconstructed on laptop
- **Error handling**: Status messages report failures

---

## 🔧 Configuration Files

### `config_rpi.yaml` (Raspberry Pi)
```yaml
mqtt:
  broker_host: "10.197.211.141"  # Laptop IP address
  broker_port: 1883
  client_id: "rplidar_scanner_rpi"
  qos: 1

scanner:
  default_port: "/dev/ttyUSB0"
  scan_2d_script: "./dump_one_scan.py"

logging:
  level: "INFO"
```

### `config_laptop.yaml` (Laptop)
```yaml
mqtt:
  broker_host: "localhost"
  broker_port: 1883
  client_id: "rplidar_viewer_laptop"
  qos: 1

viewer:
  receive_dir: "./data"
  auto_load: true
  window_width: 1024
  window_height: 768
```

---

## 🎮 Usage

### Start Scanner Service (Raspberry Pi)
```bash
cd ~/rplidar-scanner
source venv/bin/activate
python rpi_scanner_service.py
```

**Expected Output:**
```
======================================================================
RPLidar Scanner Service (Raspberry Pi)
======================================================================

INFO - RPi Scanner Service initialized
INFO - Connecting to MQTT broker at 10.197.211.141:1883
INFO - Connected to MQTT broker successfully
INFO - Ready to receive scan commands
Scanner service running - press Ctrl+C to exit
```

### Launch GUI Viewer (Laptop)
```powershell
cd C:\Users\robii\Documents\github_repos\rplidar\RPLidar-3D-PointCloud
.\venv\Scripts\Activate.ps1
python viewer/app.py
```

1. Go to **"Scan Control"** tab
2. Select **"dump_one_scan.py"** (default)
3. Click **"Start Scan"**
4. Wait for scan completion (~10-30 seconds)
5. Go to **"Visualization"** tab
6. Click **"Visualize in 3D Window"**

---

## 🐛 Troubleshooting

### Issue: "MQTT client not initialized"
**Cause**: Mosquitto broker not running on laptop  
**Fix**:
```powershell
net start mosquitto
# or
sc query mosquitto  # Check status
```

### Issue: RPi can't connect to broker
**Cause**: Firewall blocking port 1883 or wrong IP in `config_rpi.yaml`  
**Fix**:
```powershell
# On laptop, check firewall
Get-NetFirewallRule -DisplayName "Mosquitto MQTT"

# Add rule if missing
New-NetFirewallRule -DisplayName "Mosquitto MQTT" `
    -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow
```

**Test connectivity from RPi**:
```bash
ping 10.197.211.141
telnet 10.197.211.141 1883
```

### Issue: No data received after scan
**Check MQTT messages**: Use MQTT Explorer (GUI) or mosquitto_sub (CLI):
```powershell
# Option 1: MQTT Explorer (GUI)
# Launch from Start Menu, connect to localhost:1883
# Subscribe to: rplidar/#

# Option 2: CLI
cd "C:\Program Files\mosquitto"
.\mosquitto_sub.exe -h localhost -t "rplidar/#" -v
```

**Check RPi logs**:
```bash
tail -f ~/rplidar-scanner/rpi_scanner.log
```

### Issue: RPLidar not detected on RPi
```bash
# Check if device is connected
lsusb | grep CP210

# Check serial ports
ls -l /dev/ttyUSB*

# Add user to dialout group (if permission denied)
sudo usermod -a -G dialout raspberrypi
# Log out and log back in
```

---

## 📖 Documentation

- **[README_SETUP.md](README_SETUP.md)** - Complete setup guide for laptop and Raspberry Pi
- **[DRIVER_INSTALL.md](DRIVER_INSTALL.md)** - CP210x USB driver installation (Windows)

---

## 📦 Dependencies

### Laptop (Full)
```
rplidar-roboticia==1.0.1  # RPLidar A1/A2 driver
numpy>=1.24.0             # Numerical operations
open3d>=0.18.0            # 3D visualization
paho-mqtt==2.1.0          # MQTT client
pyyaml>=6.0               # YAML config parsing
pyserial>=3.5             # Serial port detection
```

### Raspberry Pi (Minimal)
```
rplidar-roboticia==1.0.1  # RPLidar A1/A2 driver
numpy>=1.24.0             # Numerical operations
paho-mqtt==2.1.0          # MQTT client
pyyaml>=6.0               # YAML config parsing
pyserial>=3.5             # Serial port detection
# Note: open3d NOT required on RPi
```

---

## 🎯 Design Decisions

### Why MQTT?
- **Decoupling**: Scanner and viewer run independently
- **Reliability**: QoS 1 ensures message delivery
- **Scalability**: Easy to add multiple clients/scanners
- **Debugging**: MQTT Explorer provides visual monitoring

### Why Chunked Transfer?
- MQTT has message size limits (~256MB depending on broker)
- 256KB chunks ensure reliable delivery over WiFi
- Base64 encoding for binary-safe transmission
- Progressive reassembly reduces memory usage

### Why Deprecate 3D Scanning?
- Requires user input during scan (servo angle prompts)
- Not compatible with automated MQTT workflow
- Can be re-implemented with automated servo control later

---

## 📝 License

MIT License

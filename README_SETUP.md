# Setup Guide - RPLidar MQTT System

Complete setup instructions for distributed RPLidar scanning system.

**System Components:**
- **Laptop** (10.197.211.141): MQTT broker + GUI viewer
- **Raspberry Pi 5**: RPLidar scanner service

---

## Prerequisites

### Hardware Requirements
- ✅ **Laptop** (Windows/Linux/macOS) with WiFi/Ethernet
- ✅ **Raspberry Pi 5** (or RPi 4/3B+) with WiFi/Ethernet  
- ✅ **RPLidar A1 or A2** with USB cable
- ✅ **Both devices on same network** (WiFi or LAN)

### Network Requirements
- Laptop must have **static IP** or **known IP address**
- Port **1883** (MQTT) must be accessible on laptop
- RPi must be able to reach laptop IP

### Software Requirements
- **Python 3.9+** (both laptop and RPi)
- **Git** (for cloning repository)

---

# Part 1: Laptop Setup

## 1.1 Install Mosquitto MQTT Broker

Mosquitto is the MQTT message broker that both devices communicate through.

### Windows

**Option A: Using Chocolatey (Recommended)**
```powershell
# Install Chocolatey if not installed
# Visit: https://chocolatey.org/install

# Install Mosquitto
choco install mosquitto -y
```

**Option B: Manual Installation**
1. Download from: https://mosquitto.org/download/
2. Run the Windows installer (.exe)
3. Default install path: `C:\Program Files\mosquitto`

**Start Mosquitto Service:**
```powershell
net start mosquitto
```

**Verify it's running:**
```powershell
sc query mosquitto
# Should show: STATE: 4 RUNNING
```

### Linux (Ubuntu/Debian)

```bash
# Install Mosquitto
sudo apt update
sudo apt install -y mosquitto mosquitto-clients

# Enable and start service
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

# Verify it's running
sudo systemctl status mosquitto
```

### macOS

```bash
# Install via Homebrew
brew install mosquitto

# Start Mosquitto
brew services start mosquitto

# Verify it's running
brew services list | grep mosquitto
```

---

## 1.2 Configure Mosquitto

### Windows

Create or edit `C:\Program Files\mosquitto\mosquitto.conf`:
```conf
# Basic MQTT broker configuration
listener 1883
allow_anonymous true

# Logging
log_dest file C:\Program Files\mosquitto\mosquitto.log
log_type all
```

**Restart Mosquitto:**
```powershell
net stop mosquitto
net start mosquitto
```

### Linux/macOS

Edit `/etc/mosquitto/mosquitto.conf` (or `/usr/local/etc/mosquitto/mosquitto.conf` on macOS):
```conf
# Basic MQTT broker configuration
listener 1883
allow_anonymous true

# Logging
log_dest file /var/log/mosquitto/mosquitto.log
log_type all
```

**Restart Mosquitto:**
```bash
# Linux
sudo systemctl restart mosquitto

# macOS
brew services restart mosquitto
```

---

## 1.3 Configure Firewall

The laptop firewall must allow incoming connections on port 1883 for RPi to connect.

### Windows

```powershell
# Add firewall rule (run as Administrator)
New-NetFirewallRule -DisplayName "Mosquitto MQTT" `
    -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow

# Verify rule was added
Get-NetFirewallRule -DisplayName "Mosquitto MQTT"
```

### Linux (Ubuntu/Debian)

```bash
# Add firewall rule
sudo ufw allow 1883/tcp

# Verify

 rule
sudo ufw status
```

### macOS

macOS firewall typically allows incoming connections by default. If you have firewall enabled:
1. System Preferences → Security & Privacy → Firewall → Firewall Options
2. Ensure "Block all incoming connections" is NOT checked

---

## 1.4 Find Your Laptop IP Address

The RPi needs to know your laptop's IP address to connect to the MQTT broker.

### Windows

```powershell
ipconfig
# Look for "IPv4 Address" under your active network adapter (WiFi or Ethernet)
# Example: 10.197.211.141
```

### Linux/macOS

```bash
ip addr show
# or
ifconfig
# Look for "inet" address under your active interface (wlan0, eth0, en0, etc.)
```

**Important:** Write down this IP address - you'll need it for RPi configuration.

---

## 1.5 Clone Repository and Install Dependencies

### Clone/Navigate to Repository

```powershell
# If cloning fresh:
git clone https://github.com/ziqiangg/RPLidar-3D-PointCloud.git
cd RPLidar-3D-PointCloud

# If already cloned:
cd C:\Users\robii\Documents\github_repos\rplidar\RPLidar-3D-PointCloud
```

### Create Virtual Environment

```powershell
# Create venv
python -m venv venv

# Activate venv (Windows)
.\venv\Scripts\Activate.ps1

# Activate venv (Linux/macOS)
source venv/bin/activate
```

### Install Laptop Dependencies

```powershell
# Upgrade pip
python -m pip install --upgrade pip

# Install all laptop dependencies (includes Open3D, paho-mqtt, etc.)
pip install -r requirements_laptop.txt
```

**Verify installation:**
```powershell
python -c "import paho.mqtt.client as mqtt; print('paho-mqtt:', mqtt.__version__)"
python -c "import open3d as o3d; print('open3d:', o3d.__version__)"
python -c "import yaml; print('pyyaml: installed')"
```

---

## 1.6 Configure Laptop Settings

Edit `config_laptop.yaml`:

```yaml
mqtt:
  broker_host: "localhost"      # Connect to local broker
  broker_port: 1883
  client_id: "rplidar_viewer_laptop"
  qos: 1

viewer:
  receive_dir: "./data"         # Where to save received scans
  auto_load: true               # Auto-load scans in GUI
  window_width: 1024
  window_height: 768

logging:
  level: "INFO"
  file: "laptop_viewer.log"
```

**No changes needed unless:**
- You want to change the data directory
- You want different window size
- You want different logging level (DEBUG, INFO, WARNING, ERROR)

---

## 1.7 Test MQTT Broker

Use MQTT Explorer (GUI) or mosquitto CLI tools to verify broker is working.

### Option A: MQTT Explorer (Recommended for Visual Debugging)

1. Launch MQTT Explorer from Start Menu
2. Create new connection:
   - **Name**: `localhost`
   - **Protocol**: `mqtt://`
   - **Host**: `localhost`
   - **Port**: `1883`
   - Leave Username/Password empty
3. Click **CONNECT**
4. You should see `$SYS/broker` topics appear

### Option B: mosquitto CLI Tools

**Terminal 1 (Subscribe):**
```powershell
cd "C:\Program Files\mosquitto"
.\mosquitto_sub.exe -h localhost -t test -v
```

**Terminal 2 (Publish):**
```powershell
cd "C:\Program Files\mosquitto"
.\mosquitto_pub.exe -h localhost -t test -m "Hello from laptop"
```

**Expected:** Terminal 1 should display: `test Hello from laptop`

---

## 1.8 Test GUI Viewer

```powershell
# Activate venv
.\venv\Scripts\Activate.ps1

# Launch GUI
python viewer/app.py
```

**Expected:** Open3D GUI window opens with two tabs:
- **Scan Control** - For initiating scans
- **Visualization** - For viewing point clouds

**Note:** Scanner won't work yet until RPi is set up. But GUI should load without errors.

---

# Part 2: Raspberry Pi Setup

## 2.1 Initial RPi Preparation

### SSH into Raspberry Pi

```bash
# From laptop/computer:
ssh raspberrypi@<your-rpi-ip>
# Password: (your RPi password)
```

**Don't know RPi IP?** Check your router's DHCP client list or use:
```bash
# From laptop (Linux/macOS with nmap installed):
nmap -sn 192.168.1.0/24  # Adjust subnet for your network

# Look for "Raspberry Pi Foundation" in results
```

### Update System

```bash
sudo apt update
sudo apt upgrade -y
```

### Install System Dependencies

```bash
# Install Python 3 and pip (usually pre-installed)
sudo apt install -y python3 python3-pip python3-venv git

# Verify Python version (should be 3.9+)
python3 --version
```

---

## 2.2 Clone Repository

```bash
# Navigate to home directory
cd ~

# Clone repository
git clone https://github.com/ziqiangg/RPLidar-3D-PointCloud.git rplidar-scanner

# Navigate to project
cd rplidar-scanner
```

**Alternative:** If you already have files on laptop, you can use SCP:
```powershell
# From laptop (PowerShell):
scp -r C:\Users\robii\Documents\github_repos\rplidar\RPLidar-3D-PointCloud raspberrypi@<rpi-ip>:~/rplidar-scanner
```

---

## 2.3 Create Virtual Environment

```bash
cd ~/rplidar-scanner

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Verify activation (prompt should show (venv))
which python
# Should show: /home/raspberrypi/rplidar-scanner/venv/bin/python
```

---

## 2.4 Install RPi Dependencies

```bash
# Activate venv (if not already)
source venv/bin/activate

# Upgrade pip
python -m pip install --upgrade pip

# Install RPi dependencies (minimal - no Open3D)
pip install -r requirements_rpi.txt
```

**Verify installation:**
```bash
python -c "import paho.mqtt.client as mqtt; print('paho-mqtt:', mqtt.__version__)"
python -c "import yaml; print('pyyaml: installed')"
python -c "from rplidar import RPLidar; print('rplidar: installed')"
```

---

## 2.5 Configure RPi Settings

Edit `config_rpi.yaml`:

```bash
nano config_rpi.yaml
```

**Update the following:**
```yaml
mqtt:
  broker_host: "10.197.211.141"  # ⚠️ CHANGE to your laptop IP!
  broker_port: 1883
  client_id: "rplidar_scanner_rpi"
  qos: 1

scanner:
  default_port: "/dev/ttyUSB0"   # RPLidar USB port (usually correct)
  scan_2d_script: "./dump_one_scan.py"

logging:
  level: "INFO"
  file: "rpi_scanner.log"
```

**Critical:** Replace `10.197.211.141` with your actual laptop IP address (from Step 1.4).

**Save and exit:** Press `Ctrl+O`, `Enter`, `Ctrl+X`

---

## 2.6 Connect RPLidar

1. **Plug RPLidar USB cable into Raspberry Pi**
2. **Verify device is detected:**

```bash
# Check USB devices
lsusb | grep CP210
# Should show: "Silicon Labs CP210x UART Bridge"

# Check serial ports
ls -l /dev/ttyUSB*
# Should show: /dev/ttyUSB0 (or ttyUSB1, ttyUSB2, etc.)
```

3. **Add user to dialout group (for serial port access):**

```bash
sudo usermod -a -G dialout raspberrypi

# Log out and log back in for changes to take effect
exit
# SSH back in
ssh raspberrypi@<rpi-ip>
```

---

## 2.7 Test Network Connectivity

Verify RPi can reach laptop broker:

```bash
# Ping laptop
ping -c 4 10.197.211.141
# Should show: 4 packets transmitted, 4 received

# Test MQTT port (requires netcat)
sudo apt install -y netcat
nc -zv 10.197.211.141 1883
# Should show: "Connection to 10.197.211.141 1883 port [tcp/*] succeeded!"
```

**If ping fails:**
- Verify laptop IP is correct
- Check both devices are on same network
- Check laptop firewall (Step 1.3)

---

## 2.8 Test MQTT Connection

Test publishing to laptop broker:

```bash
# Install mosquitto clients (optional, for testing)
sudo apt install -y mosquitto-clients

# Subscribe to test topic (from laptop, using MQTT Explorer or mosquitto_sub)

# Publish test message from RPi
mosquitto_pub -h 10.197.211.141 -t test -m "Hello from RPi"
```

**Expected:** Message appears in MQTT Explorer or mosquitto_sub on laptop.

---

## 2.9 Test Scanner Service

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
INFO - Subscribed to: rplidar/commands/scan
INFO - Ready to receive scan commands
Scanner service running - press Ctrl+C to exit
```

**If connection fails:**
- Check `config_rpi.yaml` has correct laptop IP
- Verify laptop Mosquitto is running: `net start mosquitto` (on laptop)
- Check firewall on laptop (Step 1.3)
- Test network connectivity (Step 2.7)

---

# Part 3: End-to-End Testing

## 3.1 Start Services

### On Raspberry Pi:
```bash
cd ~/rplidar-scanner
source venv/bin/activate
python rpi_scanner_service.py
```

**Leave this terminal open** - this is your scanner service.

### On Laptop:
```powershell
cd C:\Users\robii\Documents\github_repos\rplidar\RPLidar-3D-PointCloud
.\venv\Scripts\Activate.ps1
python viewer/app.py
```

**Leave this window open** - this is your GUI viewer.

---

## 3.2 Monitor MQTT Traffic

### Option A: MQTT Explorer (Recommended)

1. Launch MQTT Explorer
2. Connect to `localhost:1883`
3. Expand tree to see all topics
4. You should see `rplidar` topic appear when scan starts

### Option B: CLI Monitoring

In a **new terminal on laptop**:
```powershell
cd "C:\Program Files\mosquitto"
.\mosquitto_sub.exe -h localhost -t "rplidar/#" -v
```

This will show all MQTT messages in real-time.

---

## 3.3 Initiate First Scan

In the GUI (laptop):

1. Go to **"Scan Control"** tab
2. **Script**: `dump_one_scan.py` (should be default)
3. **Port**: Leave empty (auto-detect)
4. Click **"Start Scan"** button

**Expected Sequence:**

**GUI Status:**
```
Scan request sent to Raspberry Pi
Scan ID: scan_20260224_143022
Waiting for response...
Scan completed successfully
```

**RPi Terminal Output:**
```
INFO - Received scan command: scan_20260224_143022, type: 2d
INFO - Starting scan scan_20260224_143022
INFO - Using port: /dev/ttyUSB0
INFO - Running: ./dump_one_scan.py /dev/ttyUSB0
INFO - Scan completed successfully: 1234 points
INFO - Sending ply file: ./data/scan.ply (123456 bytes)
INFO - Completed sending ply file (3 chunks)
INFO - Sending csv file: ./data/scan.csv (45678 bytes)
INFO - Completed sending csv file (1 chunk)
```

**MQTT Explorer/CLI:**
```
rplidar/commands/scan {"scan_id": "scan_20260224_143022", ...}
rplidar/status/scan_20260224_143022 {"status": "running", ...}
rplidar/data/scan_20260224_143022 {"filename": "scan.ply", "chunk_index": 0, ...}
rplidar/status/scan_20260224_143022 {"status": "completed", ...}
```

---

## 3.4 Visualize Scan

In the GUI (laptop):

1. Go to **"Visualization"** tab
2. The scan should be **auto-loaded** (if `auto_load: true` in config)
3. Or click **"Load File..."** and select `data/scan.ply`
4. Click **"Visualize in 3D Window"**

**Expected:** New Open3D window opens showing your scanned point cloud!

**Controls:**
- **Mouse left drag**: Rotate view
- **Mouse scroll**: Zoom in/out
- **Mouse right drag**: Pan view
- **R**: Reset view

---

## 3.5 Verify Files

Check that files were saved to laptop:

```powershell
# On laptop:
dir data\
# Should show:
# scan.ply
# scan.csv
```

Check RPi also has the files:

```bash
# On RPi:
ls -lh data/
# Should show:
# scan.ply
# scan.csv
```

---

# Part 4: Advanced Configuration

## 4.1 Static IP for Laptop (Recommended)

To avoid updating `config_rpi.yaml` every time laptop IP changes:

### Windows

1. Settings → Network & Internet → Ethernet/WiFi
2. Click your connection → IP settings → Edit
3. Change to **Manual**
4. Set:
   - IP: `10.197.211.141` (or your preferred static IP)
   - Subnet mask: `255.255.255.0`
   - Gateway: (your router IP, usually `10.197.211.1`)
   - DNS: `8.8.8.8` (or your router IP)

### Linux

Edit `/etc/netplan/01-netcfg.yaml` (Ubuntu):
```yaml
network:
  version: 2
  ethernets:
    eth0:  # or your interface name
      dhcp4: no
      addresses: [10.197.211.141/24]
      gateway4: 10.197.211.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

Apply: `sudo netplan apply`

---

## 4.2 Autostart Scanner Service on RPi (Optional)

Create systemd service to start scanner automatically on boot.

```bash
sudo nano /etc/systemd/system/rplidar-scanner.service
```

**Content:**
```ini
[Unit]
Description=RPLidar Scanner Service
After=network.target

[Service]
Type=simple
User=raspberrypi
WorkingDirectory=/home/raspberrypi/rplidar-scanner
ExecStart=/home/raspberrypi/rplidar-scanner/venv/bin/python /home/raspberrypi/rplidar-scanner/rpi_scanner_service.py
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
```

**Enable and start:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable rplidar-scanner.service
sudo systemctl start rplidar-scanner.service

# Check status
sudo systemctl status rplidar-scanner.service

# View logs
journalctl -u rplidar-scanner.service -f
```

---

## 4.3 Multiple Scanners (Advanced)

To use multiple RPi scanners with one laptop:

1. **On each RPi**, edit `config_rpi.yaml`:
   ```yaml
   mqtt:
     client_id: "rplidar_scanner_rpi_1"  # Unique ID for each RPi!
   ```

2. **On laptop**, GUI will show scan requests from all RPis

3. **MQTT Explorer** will show which RPi sent which data

---

# Troubleshooting

## Issue: "Module 'paho.mqtt.client' not found"

**Cause**: Dependencies not installed or wrong Python environment

**Fix:**
```bash
# Activate venv
source venv/bin/activate  # Linux/macOS
.\venv\Scripts\Activate.ps1  # Windows

# Reinstall dependencies
pip install -r requirements_rpi.txt  # On RPi
pip install -r requirements_laptop.txt  # On laptop
```

---

## Issue: "Permission denied: '/dev/ttyUSB0'"

**Cause**: User not in `dialout` group

**Fix:**
```bash
sudo usermod -a -G dialout raspberrypi
# Log out and back in
exit
ssh raspberrypi@<rpi-ip>
```

---

## Issue: "Connection refused: 10.197.211.141:1883"

**Cause**: Mosquitto not running on laptop OR firewall blocking

**Fix:**
```powershell
# On laptop:
net start mosquitto
sc query mosquitto  # Verify it's running

# Check firewall
Get-NetFirewallRule -DisplayName "Mosquitto MQTT"
```

---

## Issue: Scan starts but no files received

**Check MQTT messages:**
```powershell
# On laptop:
cd "C:\Program Files\mosquitto"
.\mosquitto_sub.exe -h localhost -t "rplidar/#" -v
```

**Check RPi logs:**
```bash
# On RPi:
tail -f ~/rplidar-scanner/rpi_scanner.log
```

**Look for**:
- "Sending ply file" - means RPi is sending
- If you see `rplidar/data/...` in MQTT but no files appear → check laptop client logs
- If you don't see `rplidar/data/...` → check RPi logs for errors

---

## Issue: GUI shows "MQTT client not initialized"

**Cause**: `laptop_viewer_client.py` not started or MQTT broker not running

**Fix:**
1. Verify Mosquitto is running: `sc query mosquitto` (Windows)
2. Restart GUI: `python viewer/app.py`
3. Check `laptop_viewer.log` for errors

---

## Issue: RPLidar not spinning

**Possible causes:**
- USB not connected properly
- Insufficient power (use powered USB hub if needed)
- Wrong serial port
- Another program using the port

**Fix:**
```bash
# On RPi: Check if device is detected
lsusb | grep CP210

# Check if port is in use
sudo lsof | grep ttyUSB

# Try different port
ls -l /dev/ttyUSB*
# Edit config_rpi.yaml if needed
```

---

# Summary Checklist

## Laptop Setup ✅
- [ ] Mosquitto installed and running
- [ ] Firewall configured (port 1883 open)
- [ ] Repository cloned
- [ ] Virtual environment created and activated
- [ ] Dependencies installed (`requirements_laptop.txt`)
- [ ] `config_laptop.yaml` configured
- [ ] MQTT broker tested (MQTT Explorer or mosquitto_sub/pub)
- [ ] GUI viewer launches without errors

## Raspberry Pi Setup ✅
- [ ] SSH access working
- [ ] System updated
- [ ] Repository cloned
- [ ] Virtual environment created and activated
- [ ] Dependencies installed (`requirements_rpi.txt`)
- [ ] `config_rpi.yaml` configured with **correct laptop IP**
- [ ] RPLidar connected and detected (`lsusb`, `/dev/ttyUSB0`)
- [ ] User added to `dialout` group
- [ ] Network connectivity verified (ping laptop, test MQTT)
- [ ] Scanner service starts without errors

## End-to-End Test ✅
- [ ] Scanner service running on RPi
- [ ] GUI viewer running on laptop
- [ ] MQTT monitoring active (MQTT Explorer or mosquitto_sub)
- [ ] Scan initiated from GUI
- [ ] MQTT messages visible in explorer/sub
- [ ] Files received in `data/` folder on laptop
- [ ] Point cloud visualizes in Open3D

---

**🎉 If all checks pass, your system is fully operational!**

For questions or issues, check:
- [README.md](README.md) - System architecture and overview
- [DRIVER_INSTALL.md](DRIVER_INSTALL.md) - USB driver installation (Windows)
- RPi logs: `~/rplidar-scanner/rpi_scanner.log`
- Laptop logs: `laptop_viewer.log`

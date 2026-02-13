# CP2102 Driver Installation Guide

## Problem
RPLidar detected in Device Manager as "CP2102 USB to UART Bridge Controller" under "Other devices" with Code 28 error (no driver installed).

## Solution: Install CP2102 Driver

### Method 1: Automatic Driver Install (Recommended)

1. **Right-click** on "CP2102 USB to UART Bridge Controller" in Device Manager
2. Select **"Update driver"**
3. Choose **"Search automatically for drivers"**
4. Windows will attempt to find and install the driver
5. **Restart your computer** if prompted

### Method 2: Manual Driver Download

If automatic install fails:

1. **Download the driver** from Silicon Labs:
   - Direct link: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
   - Click **"Downloads"** tab
   - Download **"CP210x Universal Windows Driver"**

2. **Extract the ZIP file** to a folder (e.g., Downloads folder)

3. **Install the driver**:
   - Right-click "CP2102 USB to UART Bridge Controller" in Device Manager
   - Select **"Update driver"**
   - Choose **"Browse my computer for drivers"**
   - Click **"Browse"** and select the folder where you extracted the files
   - Click **"Next"** and follow the prompts

4. **Restart if prompted**

### Method 3: Windows Update

1. Press `Win + I` to open Settings
2. Go to **Windows Update**
3. Click **"Check for updates"**
4. Windows may find and install the driver automatically

## After Installation

### What You Should See:
- CP2102 should **disappear** from "Other devices"
- A **new COM port** appears under "Ports (COM & LPT)"
- Example: "Silicon Labs CP210x USB to UART Bridge (COM7)"

### Verify the Connection:

Run the diagnostic tool:
```powershell
python examples/diagnose.py
```

You should now see a USB-serial port (e.g., COM6, COM7, COM8)

### Test the RPLidar:

Once you see the COM port number, test it:
```powershell
python dump_one_scan.py COMX  # Replace X with your actual port number
```

Example:
```powershell
python dump_one_scan.py COM7
```

## Quick Start After Driver Install

1. Run diagnostic: `python examples/diagnose.py`
2. Note your COM port (e.g., COM7)
3. Capture scan: `python dump_one_scan.py COM7`
4. View output: Check `data/scan.csv` and `data/scan.ply`

## Troubleshooting

**Driver install fails?**
- Download manual driver (Method 2)
- Run installer as Administrator (right-click → Run as administrator)

**COM port still not showing?**
- Unplug and replug RPLidar USB cable
- Try a different USB port
- Restart computer

**"Windows cannot verify digital signature"?**
- Disable Driver Signature Enforcement (advanced - Google for instructions)
- Or download latest driver from Silicon Labs (they're signed)

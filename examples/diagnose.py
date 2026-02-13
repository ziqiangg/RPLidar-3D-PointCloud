"""
RPLidar Connection Diagnostic Tool

This script helps troubleshoot RPLidar connection issues on Windows.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def check_serial_module():
    """Check if pyserial is installed."""
    try:
        import serial
        import serial.tools.list_ports
        print("✓ pyserial is installed")
        return True
    except ImportError:
        print("✗ pyserial is NOT installed")
        print("  Install with: pip install pyserial")
        return False


def list_all_ports():
    """List all COM ports with detailed information."""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            print("\n⚠ NO serial ports detected on this system")
            return []
        
        print(f"\n{'='*80}")
        print(f"Found {len(ports)} serial port(s):")
        print(f"{'='*80}")
        
        for i, port in enumerate(ports, 1):
            print(f"\n[{i}] {port.device}")
            print(f"    Description: {port.description}")
            print(f"    Manufacturer: {port.manufacturer or 'Unknown'}")
            print(f"    Hardware ID: {port.hwid}")
            print(f"    VID:PID: {port.vid}:{port.pid}" if port.vid else "    VID:PID: N/A")
            print(f"    Serial Number: {port.serial_number or 'N/A'}")
            
            # Check if it could be RPLidar
            desc_lower = port.description.lower()
            hwid_lower = port.hwid.lower()
            
            is_bluetooth = 'bluetooth' in desc_lower or 'bthenum' in hwid_lower
            is_usb_serial = any(chip in desc_lower or chip in hwid_lower 
                              for chip in ['cp210', 'ch340', 'ftdi', 'usb serial', 'uart', 'usb-serial'])
            
            if is_bluetooth:
                print(f"    ⚠ This is a BLUETOOTH device (not RPLidar)")
            elif is_usb_serial:
                print(f"    ✓ This appears to be a USB-SERIAL device (likely RPLidar!)")
            else:
                print(f"    ? Unknown device type")
        
        print(f"\n{'='*80}\n")
        return ports
    except ImportError:
        return []


def check_device_manager_instructions():
    """Provide instructions for checking Device Manager."""
    print("\n" + "="*80)
    print("HOW TO CHECK IF RPLIDAR IS CONNECTED:")
    print("="*80)
    print("""
1. Press Win + X and select 'Device Manager'
   (or search for 'Device Manager' in Start menu)

2. Look for one of these categories:
   • Ports (COM & LPT)
   • Universal Serial Bus devices
   • Other devices (if driver not installed)

3. What to look for:
   ✓ "USB Serial Port (COMx)" - RPLidar is connected and working
   ✓ "Silicon Labs CP210x USB to UART Bridge" - RPLidar chip detected
   ✓ "CH340" or "CH341" - Alternative USB-serial chip
   ⚠ Yellow warning icon - Driver issue
   ✗ Nothing USB-related - RPLidar not connected

4. If you see "Unknown device" or yellow warning:
   • Right-click → Update driver
   • Or download CP210x drivers from Silicon Labs website
""")


def provide_troubleshooting_steps():
    """Provide troubleshooting steps."""
    print("\n" + "="*80)
    print("TROUBLESHOOTING STEPS:")
    print("="*80)
    print("""
1. CHECK PHYSICAL CONNECTION
   • Is the RPLidar USB cable plugged into your computer?
   • Try a different USB port
   • Try a different USB cable if available

2. CHECK POWER
   • Is the RPLidar motor spinning?
   • Some RPLidar models need external power (5V/12V)
   • USB port may not provide enough power

3. INSTALL DRIVERS (if needed)
   • RPLidar uses CP210x USB-to-UART bridge chip
   • Download from: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
   • Install the driver and reconnect the device

4. CHECK WINDOWS DEVICE MANAGER
   • Win + X → Device Manager
   • Expand "Ports (COM & LPT)"
   • Look for new COM port when plugging/unplugging RPLidar

5. VERIFY IN DEVICE MANAGER
   • Note the COM port number (e.g., COM7, COM8)
   • Then run: python dump_one_scan.py COMX (replace X with your port)

6. STILL NOT WORKING?
   • Restart your computer
   • Check if another program is using the port
   • Try the RPLidar on another computer to verify it works
""")


def main():
    print("="*80)
    print("RPLIDAR CONNECTION DIAGNOSTIC")
    print("="*80)
    
    # Check if pyserial is installed
    if not check_serial_module():
        return
    
    # List all available ports
    ports = list_all_ports()
    
    # Analyze results
    if not ports:
        print("⚠ No serial ports found on this system.")
        print("⚠ The RPLidar is NOT connected or drivers are not installed.")
    else:
        # Check if any port looks like RPLidar
        import serial.tools.list_ports
        all_ports = serial.tools.list_ports.comports()
        
        usb_serial_ports = []
        bluetooth_ports = []
        
        for port in all_ports:
            desc_lower = port.description.lower()
            hwid_lower = port.hwid.lower()
            
            if 'bluetooth' in desc_lower or 'bthenum' in hwid_lower:
                bluetooth_ports.append(port.device)
            elif any(chip in desc_lower or chip in hwid_lower 
                    for chip in ['cp210', 'ch340', 'ftdi', 'usb serial', 'uart']):
                usb_serial_ports.append(port.device)
        
        if usb_serial_ports:
            print(f"✓ FOUND {len(usb_serial_ports)} USB-SERIAL PORT(S): {', '.join(usb_serial_ports)}")
            print(f"\nTry running:")
            for port in usb_serial_ports:
                print(f"  python dump_one_scan.py {port}")
        else:
            print("⚠ NO USB-SERIAL DEVICES FOUND")
            print(f"  Only Bluetooth ports detected: {', '.join(bluetooth_ports)}")
            print("\n⚠ RPLidar is NOT connected or drivers are missing!")
    
    # Provide instructions
    check_device_manager_instructions()
    provide_troubleshooting_steps()
    
    print("\n" + "="*80)
    print("After connecting the RPLidar, run this diagnostic again to verify.")
    print("="*80)


if __name__ == "__main__":
    main()

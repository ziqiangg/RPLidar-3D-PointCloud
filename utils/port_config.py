"""
Cross-platform serial port configuration for RPLidar.

This module automatically detects the operating system and finds available
serial ports, helping identify the correct port for the RPLidar device.
"""

import sys
import os
import platform

def get_available_ports():
    """
    List all available serial ports on the system.
    
    Returns:
        list: List of available port names (e.g., ['COM3', 'COM4'] on Windows
              or ['/dev/ttyUSB0', '/dev/ttyUSB1'] on Linux)
    """
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    except ImportError:
        print("Warning: pyserial not installed. Install with: pip install pyserial")
        return []


def get_port_info():
    """
    Get detailed information about all available serial ports.
    
    Returns:
        list: List of tuples containing (device, description, hwid)
    """
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [(p.device, p.description, p.hwid) for p in ports]
    except ImportError:
        return []


def find_rplidar_port():
    """
    Attempt to automatically detect the RPLidar port.
    
    On Linux, this looks for /dev/ttyUSB* devices.
    On Windows, this attempts to identify USB serial devices.
    
    Returns:
        str or None: The detected port name, or None if not found
    """
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        
        os_type = platform.system()
        
        if os_type == "Linux":
            # Prefer /dev/ttyUSB0 on Linux (standard for USB-to-serial adapters)
            for port in ports:
                if "ttyUSB" in port.device:
                    return port.device
        
        elif os_type == "Windows":
            # On Windows, look for USB serial devices
            # RPLidar typically shows up as a CP210x or similar USB-UART bridge
            for port in ports:
                desc_lower = port.description.lower()
                hwid_lower = port.hwid.lower()
                
                # Common USB-serial chip identifiers
                if any(keyword in desc_lower or keyword in hwid_lower 
                       for keyword in ["cp210", "ch340", "ftdi", "usb serial", "uart"]):
                    return port.device
        
        elif os_type == "Darwin":  # macOS
            # On macOS, USB serial devices appear as /dev/cu.usbserial-* or similar
            for port in ports:
                if "usbserial" in port.device or "usbmodem" in port.device:
                    return port.device
        
    except ImportError:
        pass
    
    return None


def find_pico_port():
    """
    Attempt to detect a Raspberry Pi Pico (or compatible USB CDC serial) port.

    Returns:
        str or None: Detected servo-controller serial port, or None if not found
    """
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        os_type = platform.system()

        scored = []
        for p in ports:
            dev = (p.device or "")
            desc = (p.description or "")
            hwid = (p.hwid or "")
            text = f"{desc} {hwid}".lower()

            score = 0
            if "pico" in text or "rp2040" in text:
                score += 5
            if "usb serial" in text or "cdc" in text:
                score += 2

            if os_type == "Linux" and "ttyACM" in dev:
                score += 3
            elif os_type == "Windows" and dev.upper().startswith("COM"):
                score += 1
            elif os_type == "Darwin" and ("usbmodem" in dev or "usbserial" in dev):
                score += 1

            if score > 0:
                scored.append((score, dev))

        if scored:
            scored.sort(key=lambda x: x[0], reverse=True)
            return scored[0][1]

        # Fallback by common device naming when descriptors are unhelpful.
        if os_type == "Linux":
            for p in ports:
                if "ttyACM" in (p.device or ""):
                    return p.device
        elif os_type == "Darwin":
            for p in ports:
                d = (p.device or "")
                if "usbmodem" in d or "usbserial" in d:
                    return d
        elif os_type == "Windows":
            for p in ports:
                d = (p.device or "")
                if d.upper().startswith("COM"):
                    return d
    except ImportError:
        pass

    return None


def get_default_port():
    """
    Get the default port based on OS, or auto-detect if possible.
    
    Returns:
        str: The default port name for the current OS
    """
    # Try auto-detection first
    detected_port = find_rplidar_port()
    if detected_port:
        return detected_port
    
    # Fall back to OS-specific defaults
    os_type = platform.system()
    
    if os_type == "Linux":
        return "/dev/ttyUSB0"
    elif os_type == "Windows":
        return "COM3"  # Common default, but may need adjustment
    elif os_type == "Darwin":  # macOS
        return "/dev/cu.usbserial-0001"
    else:
        return "COM3"  # Generic fallback


def get_default_servo_port(preferred: str = "auto"):
    """
    Resolve servo controller serial port with optional auto-detection.

    If a Linux /dev path is explicitly configured but missing, this function
    falls back to auto-detection to tolerate enumeration changes (e.g., ACM0->ACM1).

    Args:
        preferred: configured port value or "auto"

    Returns:
        str: resolved servo serial port
    """
    preferred = (preferred or "auto").strip()

    if preferred.lower() != "auto":
        if preferred.startswith("/dev/"):
            if os.path.exists(preferred):
                return preferred
        else:
            return preferred

    detected = find_pico_port()
    if detected:
        return detected

    os_type = platform.system()
    if os_type == "Linux":
        return "/dev/ttyACM0"
    elif os_type == "Windows":
        return "COM4"
    elif os_type == "Darwin":
        return "/dev/cu.usbmodem0001"
    else:
        return "COM4"


def select_port_interactive():
    """
    Interactively let the user select a port from available options.
    
    Returns:
        str: The selected port name
    """
    ports_info = get_port_info()
    
    if not ports_info:
        print("No serial ports detected.")
        print("Please enter the port name manually.")
        return input("Port name: ").strip()
    
    print("\nAvailable serial ports:")
    print("-" * 70)
    for i, (device, description, hwid) in enumerate(ports_info, 1):
        print(f"{i}. {device}")
        print(f"   Description: {description}")
        print(f"   Hardware ID: {hwid}")
        print()
    
    while True:
        try:
            choice = input(f"Select port [1-{len(ports_info)}] or enter custom port name: ").strip()
            
            # Check if it's a number (selection from list)
            if choice.isdigit():
                idx = int(choice) - 1
                if 0 <= idx < len(ports_info):
                    return ports_info[idx][0]
                else:
                    print(f"Please enter a number between 1 and {len(ports_info)}")
            else:
                # Assume it's a custom port name
                return choice
        except KeyboardInterrupt:
            print("\nCancelled.")
            sys.exit(0)


def print_port_info():
    """Print information about the detected OS and available ports."""
    os_type = platform.system()
    print(f"Operating System: {os_type}")
    print(f"Default port: {get_default_port()}")
    print()
    
    ports_info = get_port_info()
    if ports_info:
        print("Available serial ports:")
        for device, description, hwid in ports_info:
            print(f"  {device}: {description}")
    else:
        print("No serial ports detected (or pyserial not installed)")


if __name__ == "__main__":
    """Demo: Show port information and auto-detection."""
    print("=== RPLidar Port Configuration ===\n")
    print_port_info()
    
    print("\n" + "="*50)
    detected = find_rplidar_port()
    if detected:
        print(f"Auto-detected RPLidar port: {detected}")
    else:
        print("Could not auto-detect RPLidar port.")
        print("You may need to select manually or check your connection.")

"""
Interactive port selection tool for RPLidar.

Run this script to see all available serial ports and test the auto-detection.
"""

import os
import sys

# Add parent directory to path to import utils
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.port_config import (
    print_port_info, 
    select_port_interactive, 
    find_rplidar_port,
    get_default_port
)

def main():
    print("="*70)
    print("RPLidar Port Selection Tool")
    print("="*70)
    print()
    
    print_port_info()
    print()
    
    print("="*70)
    detected = find_rplidar_port()
    if detected:
        print(f"✓ Auto-detected RPLidar at: {detected}")
        print()
        use_detected = input("Use this port? [Y/n]: ").strip().lower()
        if use_detected in ['', 'y', 'yes']:
            print(f"\nSelected port: {detected}")
            print(f"\nYou can now run:")
            print(f"  python rplidarTest.py {detected}")
            print(f"  python dump_one_scan.py {detected}")
            return
    else:
        print("⚠ Could not auto-detect RPLidar port")
        print(f"  Default will be: {get_default_port()}")
    
    print()
    print("="*70)
    print()
    
    selected = select_port_interactive()
    print(f"\nSelected port: {selected}")
    print(f"\nTo use this port, run:")
    print(f"  python rplidarTest.py {selected}")
    print(f"  python dump_one_scan.py {selected}")

if __name__ == "__main__":
    main()

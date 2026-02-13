from rplidar import RPLidar
import time
import sys
import os
import sys

# Add parent directory to path to import utils
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.port_config import get_default_port, print_port_info

BAUDRATE = 115200  # A1 default

def main():
    # Auto-detect port or use OS-appropriate default
    port_name = get_default_port()
    
    print(f"Using port: {port_name}")
    print("(You can override by setting PORT_NAME environment variable or passing as argument)\n")
    
    # Allow command-line override: python rplidarTest.py COM4
    if len(sys.argv) > 1:
        port_name = sys.argv[1]
        print(f"Port overridden to: {port_name}\n")
    
    lidar = RPLidar(port_name, baudrate=BAUDRATE)

    try:
        print("Info:", lidar.get_info())
        print("Health:", lidar.get_health())

        lidar.start_motor()
        time.sleep(2)  # let it spin up

        # max_buf_meas limits how much the lib buffers before yielding
        for i, scan in enumerate(lidar.iter_scans(max_buf_meas=800)):
            # scan is a list of tuples: (quality, angle_deg, distance_mm)
            distances = [d for (_, _, d) in scan if d > 0]
            if distances:
                print(
                    f"scan {i}: points={len(scan)} "
                    f"min={min(distances):.1f}mm max={max(distances):.1f}mm"
                )

            # print a few sample points (optional)
            for q, a, d in scan[:5]:
                print(f"  q={q:3d} angle={a:7.2f}° dist={d:8.2f} mm")

            if i >= 5:  # stop after 6 scans
                break

    finally:
        # Proper shutdown order
        try:
            lidar.stop()        # stop scanning
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
        lidar.disconnect()

if __name__ == "__main__":
    main()

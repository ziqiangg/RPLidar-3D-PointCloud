from rplidar import RPLidar
import time

PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 115200  # A1 default

def main():
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)

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

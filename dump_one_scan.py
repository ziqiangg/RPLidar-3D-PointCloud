import csv, math, time
from rplidar import RPLidar

PORT = "/dev/ttyUSB0"
BAUD = 115200

def main():
    lidar = RPLidar(PORT, baudrate=BAUD, timeout=3)

    try:
        print("Info:", lidar.get_info(), flush=True)
        print("Health:", lidar.get_health(), flush=True)

        print("Starting motor...", flush=True)
        lidar.start_motor()
        time.sleep(2)

        print("Starting scan loop (waiting for first scan)...", flush=True)
        t0 = time.time()

        # Don’t require a dense scan first — grab the first scan we receive.
        scan = None
        for i, s in enumerate(lidar.iter_scans(max_buf_meas=1200, min_len=5)):
            scan = s
            print(f"Got scan batch #{i} with {len(scan)} points after {time.time()-t0:.2f}s", flush=True)
            # keep grabbing until we have a reasonable number of points OR time out
            if len(scan) >= 200:
                break
            if time.time() - t0 > 5:
                print("Still getting tiny batches; continuing anyway.", flush=True)
                break

        if not scan:
            raise RuntimeError("No scan data received at all.")

        # Convert to XY (meters)
        pts = []
        for q, ang, dist in scan:
            if dist <= 0:
                continue
            r = dist / 1000.0
            th = math.radians(ang)
            x = r * math.cos(th)
            y = r * math.sin(th)
            pts.append((q, ang, dist, x, y, 0.0))

        print(f"Writing {len(pts)} points...", flush=True)

        with open("scan.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["quality", "angle_deg", "distance_mm", "x_m", "y_m", "z_m"])
            w.writerows(pts)

        with open("scan.ply", "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(pts)}\n")
            f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
            for _, _, _, x, y, z in pts:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        print("Done: scan.csv + scan.ply", flush=True)

    finally:
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
        lidar.disconnect()

if __name__ == "__main__":
    main()

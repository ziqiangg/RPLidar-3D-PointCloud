import csv, math, time
from rplidar import RPLidar

PORT = "COM4"  # Windows COM port (try COM5 if this doesn't work)
BAUD = 115200
SCANS_PER_ANGLE = 3  # Multi-scan averaging
MOTOR_PWM = 500      # Slower motor for denser points (default ~600)

def main():
    lidar = RPLidar(PORT, baudrate=BAUD, timeout=3)
    all_points = []  # Store all points from all servo angles

    try:
        print("Info:", lidar.get_info(), flush=True)
        print("Health:", lidar.get_health(), flush=True)

        # Start motor with custom PWM for slower rotation
        print(f"Starting motor at PWM={MOTOR_PWM} (slower = denser points)...", flush=True)
        lidar.start_motor()
        lidar.set_pwm(MOTOR_PWM)
        time.sleep(2)

        print("\n=== Ready to capture scans ===", flush=True)
        print(f"Taking {SCANS_PER_ANGLE} scans per angle for better coverage", flush=True)
        print("Enter servo angle in degrees (or 'q'/'quit'/'exit' to finish)\n", flush=True)

        while True:
            # Get servo angle from user
            user_input = input("Servo angle (θ): ").strip().lower()
            
            if user_input in ['q', 'quit', 'exit']:
                print("Finishing capture...", flush=True)
                break
            
            try:
                theta_deg = float(user_input)
            except ValueError:
                print("Invalid input. Please enter a number or 'q' to quit.", flush=True)
                continue
            
            theta_rad = math.radians(theta_deg)
            print(f"\nCapturing at θ={theta_deg}° (taking {SCANS_PER_ANGLE} scans)...", flush=True)
            
            # Collect multiple scans at this angle
            scan_points = []
            for scan_num in range(SCANS_PER_ANGLE):
                print(f"  Scan {scan_num + 1}/{SCANS_PER_ANGLE}...", flush=True)
                t0 = time.time()
                
                # Get one full 360° scan
                scan = None
                for i, s in enumerate(lidar.iter_scans(max_buf_meas=1200, min_len=5)):
                    scan = s
                    if len(scan) >= 150:  # Lower threshold since we're merging multiple
                        break
                    if time.time() - t0 > 5:
                        break
                
                if scan:
                    scan_points.extend(scan)
                    print(f"    Got {len(scan)} points", flush=True)
            
            print(f"  Total: {len(scan_points)} points from {SCANS_PER_ANGLE} scans", flush=True)
            
            # Convert to 3D XYZ coordinates
            for q, phi_deg, dist in scan_points:
                if dist <= 0:
                    continue
                
                r = dist / 1000.0  # mm to meters
                phi_rad = math.radians(phi_deg)
                
                # 3D spherical to Cartesian conversion
                # phi = horizontal angle (from lidar), theta = vertical angle (from servo)
                x = r * math.cos(phi_rad) * math.cos(theta_rad)
                y = r * math.sin(phi_rad)
                z = r * math.cos(phi_rad) * math.sin(theta_rad)
                
                all_points.append((q, phi_deg, theta_deg, dist, x, y, z))
            
            print(f"✓ Captured θ={theta_deg}° → Total points so far: {len(all_points)}\n", flush=True)

        if not all_points:
            print("No data captured. Exiting.", flush=True)
            return

        # Write CSV
        print(f"\nWriting {len(all_points)} points to scan_3d.csv...", flush=True)
        with open("scan_3d.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["quality", "phi_deg", "theta_deg", "distance_mm", "x_m", "y_m", "z_m"])
            w.writerows(all_points)

        # Write PLY
        print(f"Writing {len(all_points)} points to scan_3d.ply...", flush=True)
        with open("scan_3d.ply", "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(all_points)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("end_header\n")
            for _, _, _, _, x, y, z in all_points:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        print("\n=== DONE ===", flush=True)
        print(f"Captured {len(all_points)} total points", flush=True)
        print("Files: scan_3d.csv + scan_3d.ply", flush=True)

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

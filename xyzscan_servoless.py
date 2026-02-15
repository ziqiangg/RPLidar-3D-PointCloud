"""
RPLidar 3D Scanner (Servoless - Manual Positioning)

Collects multiple 2D vertical scans at different servo angles to build a 3D point cloud.
User manually adjusts servo position between scans.
"""

import csv, math, time, sys, os
import numpy as np
from rplidar import RPLidar
from utils.port_config import get_default_port

BAUD = 115200
MIN_ANGLE_RESOLUTION = 2.0  # Degrees - must have at least one point every 2 degrees
MIN_COVERAGE = 0.80  # Require 80% angle coverage (288° out of 360°) - relaxed for low-density lidars
MAX_SCANS_TO_MERGE = 20  # Merge up to 20 scans to fill coverage gaps (increased patience)
RELAXED_MAX_GAP = 10.0  # Accept max gaps up to 10° for sparse scans
PLATEAU_PATIENCE = 5  # Number of scans with no improvement before accepting plateau
SCAN_TIMEOUT = 5  # Timeout in seconds for scan collection
BUFFER_SIZE = 5000  # Buffer size for lidar measurements (increased to prevent overflow)


def merge_scans(scans):
    """
    Merge multiple scans, keeping the best measurement for each angle bin.
    
    Args:
        scans: List of scan lists, where each scan is [(quality, angle, distance), ...]
    
    Returns:
        list: Merged scan with best measurements per angle bin
    """
    # Create dict to store best measurement per angle bin (2-degree bins)
    angle_bins = {}
    
    for scan in scans:
        for quality, angle, dist in scan:
            if dist <= 0:
                continue
            
            # Determine 2-degree bin
            bin_idx = int(angle / MIN_ANGLE_RESOLUTION)
            
            # Keep measurement with best quality, or first if quality same
            if bin_idx not in angle_bins or quality > angle_bins[bin_idx][0]:
                angle_bins[bin_idx] = (quality, angle, dist)
    
    # Convert back to list
    merged = list(angle_bins.values())
    return merged


def validate_scan_quality(scan, min_resolution=MIN_ANGLE_RESOLUTION, min_coverage=MIN_COVERAGE):
    """
    Validate that a scan meets minimum quality requirements.
    
    Args:
        scan: List of (quality, angle, distance) tuples from rplidar
        min_resolution: Maximum angle gap allowed (degrees)
        min_coverage: Minimum fraction of 360° that must have data
    
    Returns:
        tuple: (is_valid, coverage_pct, max_gap, point_count, message)
    """
    if not scan or len(scan) < 10:
        return False, 0.0, 360.0, len(scan), "Too few points"
    
    # Extract and sort angles
    angles = sorted([angle for _, angle, dist in scan if dist > 0])
    
    if len(angles) < 10:
        return False, 0.0, 360.0, len(angles), "Too few valid measurements"
    
    # Calculate angle coverage by checking which 2-degree bins have data
    angle_bins = set()
    for angle in angles:
        bin_idx = int(angle / min_resolution)
        angle_bins.add(bin_idx)
    
    # Total bins needed for 360° at min_resolution
    total_bins = int(360 / min_resolution)
    coverage = len(angle_bins) / total_bins
    
    # Find maximum gap between consecutive points
    max_gap = 0
    for i in range(len(angles)):
        next_i = (i + 1) % len(angles)
        gap = angles[next_i] - angles[i]
        if gap < 0:  # Wrapped around 360°
            gap = (360 - angles[i]) + angles[next_i]
        max_gap = max(max_gap, gap)
    
    # Validate with relaxed criteria for sparse scans
    is_valid = coverage >= min_coverage and max_gap <= RELAXED_MAX_GAP
    
    message = f"Coverage: {coverage*100:.1f}%, Max gap: {max_gap:.1f} deg, Points: {len(angles)}"
    
    return is_valid, coverage, max_gap, len(angles), message


def perform_scan_at_angle(lidar, servo_angle):
    """
    Perform a 2D scan at a specific servo angle.
    
    Args:
        lidar: RPLidar instance
        servo_angle: Current servo position in degrees (0-359)
    
    Returns:
        list: Merged scan data [(quality, angle, distance), ...]
    """
    print(f"\n{'='*60}", flush=True)
    print(f"SCANNING AT SERVO ANGLE: {servo_angle} deg", flush=True)
    print(f"{'='*60}", flush=True)
    print(f"Requirement: {MIN_COVERAGE*100:.0f}% coverage, max gap {RELAXED_MAX_GAP:.0f} deg", flush=True)
    
    t0 = time.time()
    
    # Collect and merge multiple scans to achieve required coverage
    collected_scans = []
    scan = None
    last_coverage = 0.0
    plateau_count = 0
    
    for i, s in enumerate(lidar.iter_scans(max_buf_meas=BUFFER_SIZE, min_len=50)):
        elapsed = time.time() - t0
        
        # Skip empty scans
        if not s or len(s) < 50:
            continue
        
        # Add to collection
        collected_scans.append(s)
        
        # Merge all scans collected so far
        merged_scan = merge_scans(collected_scans)
        
        # Validate merged scan quality
        is_valid, coverage, max_gap, point_count, message = validate_scan_quality(merged_scan)
        
        scan_info = f"[Single: {len(s)} pts]" if len(collected_scans) == 1 else f"[Merged: {len(collected_scans)} scans]"
        print(f"  Scan #{i}: {message} {scan_info} [{elapsed:.2f}s]", flush=True)
        
        # Check if coverage is plateauing
        if abs(coverage - last_coverage) < 0.01:
            plateau_count += 1
        else:
            plateau_count = 0
        last_coverage = coverage
        
        # Accept if we meet quality requirements
        if is_valid:
            scan = merged_scan
            print(f"  [OK] Quality achieved! Merged {len(collected_scans)} scan(s)", flush=True)
            break
        
        # Early exit if coverage plateaued (increased patience)
        if plateau_count >= PLATEAU_PATIENCE and len(collected_scans) >= 5:
            scan = merged_scan
            print(f"  [!] Coverage plateaued at {coverage*100:.1f}%", flush=True)
            break
        
        # Stop collecting if max scans reached
        if len(collected_scans) >= MAX_SCANS_TO_MERGE:
            scan = merged_scan
            print(f"  [!] Max scans reached ({MAX_SCANS_TO_MERGE})", flush=True)
            break
        
        # Timeout (increased to 30s for more patience)
        if elapsed > SCAN_TIMEOUT:
            scan = merged_scan if collected_scans else s
            print(f"  [!] Timeout ({len(collected_scans)} scans merged)", flush=True)
            break
    
    if not scan:
        raise RuntimeError(f"No scan data received at servo angle {servo_angle} deg")
    
    # Final report
    is_valid, coverage, max_gap, point_count, message = validate_scan_quality(scan)
    print(f"\n  SCAN COMPLETE:", flush=True)
    print(f"    {message}", flush=True)
    print(f"    Valid: {'YES [OK]' if is_valid else 'NO [X]'}", flush=True)
    
    return scan


def compute_3d_coordinates(scan, servo_angle_deg):
    """
    Convert 2D scan data to 3D coordinates.
    
    Args:
        scan: List of (quality, angle, distance) tuples
        servo_angle_deg: Servo position in degrees
    
    Returns:
        list: [(quality, lidar_angle, distance, servo_angle, x, y, z), ...]
    """
    points_3d = []
    servo_rad = math.radians(servo_angle_deg)
    
    for quality, lidar_angle, distance in scan:
        if distance <= 0:
            continue
        
        # Convert to meters
        r = distance / 1000.0
        
        # Lidar angle in radians
        lidar_rad = math.radians(lidar_angle)
        
        # 3D Coordinate Transformation:
        # - Lidar scans vertically (measures elevation angle)
        # - Servo rotates horizontally (azimuth angle)
        # 
        # The lidar's vertical scan gives us the elevation,
        # and the servo's horizontal position determines the azimuth.
        # 
        # Spherical to Cartesian conversion:
        # - Horizontal projection: r_horizontal = r * cos(lidar_angle)
        # - Vertical component: z = r * sin(lidar_angle)
        # - X and Y from horizontal projection rotated by servo angle
        
        r_horizontal = r * math.cos(lidar_rad)  # Project onto horizontal plane
        
        x = r_horizontal * math.cos(servo_rad)  # Horizontal component in X
        y = r_horizontal * math.sin(servo_rad)  # Horizontal component in Y
        z = r * math.sin(lidar_rad)              # Vertical component (elevation)
        
        points_3d.append((quality, lidar_angle, distance, servo_angle_deg, x, y, z))
    
    return points_3d


def get_user_input(prompt, valid_range=None):
    """
    Get validated user input.
    
    Args:
        prompt: Prompt to display
        valid_range: Optional tuple (min, max) for numeric validation
    
    Returns:
        str or int: User input
    """
    while True:
        try:
            response = input(prompt).strip()
            
            if response.lower() == 'q':
                return 'q'
            
            if response.lower() in ['y', 'n']:
                return response.lower()
            
            if valid_range:
                value = int(response)
                if valid_range[0] <= value <= valid_range[1]:
                    return value
                else:
                    print(f"  [X] Please enter a number between {valid_range[0]} and {valid_range[1]}")
            else:
                return response
                
        except ValueError:
            print(f"  [X] Invalid input. Please try again.")
        except KeyboardInterrupt:
            print("\n\nScan cancelled by user.")
            return 'q'


def main():
    # Auto-detect port or use OS-appropriate default
    port = get_default_port()
    
    # Allow command-line override
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print("="*70)
    print("RPLidar 3D Scanner (Manual Servo Positioning)")
    print("="*70)
    print(f"Port: {port}")
    print("\nInteractive mode: Type 'q' at any prompt to quit, 'n' to skip a scan.\n")
    
    lidar = RPLidar(port, baudrate=BAUD, timeout=3)
    
    # Storage for all 3D points
    all_points_3d = []
    scan_count = 0
    
    try:
        # Initialize lidar
        info = lidar.get_info()
        health = lidar.get_health()
        print("Lidar Info:", info)
        print("Lidar Health:", health)
        print()
        
        print("Starting motor...")
        lidar.start_motor()
        time.sleep(2)
        print("Motor started.\n")
        
        # Interactive scanning loop
        current_servo_angle = 0  # Start at 0 degrees
        
        while True:
            print("\n" + "="*70)
            if scan_count == 0:
                print(f"FIRST SCAN (Servo Angle: {current_servo_angle} deg)")
                print("="*70)
                ready = get_user_input(f"\nStart first scan at {current_servo_angle} deg? (y/n): ")
            else:
                print(f"SCAN #{scan_count + 1} (Servo Angle: {current_servo_angle} deg)")
                print("="*70)
                print(f"\n-> Please manually position servo to {current_servo_angle} deg")
                ready = get_user_input(f"Ready to scan at {current_servo_angle} deg? (y/n): ")
            
            if ready == 'q' or ready == 'n':
                print("\nScan cancelled.")
                break
            
            # Perform scan at current angle
            scan_2d = perform_scan_at_angle(lidar, current_servo_angle)
            
            # Convert to 3D coordinates
            points_3d = compute_3d_coordinates(scan_2d, current_servo_angle)
            all_points_3d.extend(points_3d)
            
            scan_count += 1
            print(f"\n  [OK] Scan #{scan_count} complete: {len(points_3d)} points added")
            print(f"  Total points collected: {len(all_points_3d)}")
            
            # Ask for next angle
            print("\n" + "-"*70)
            next_input = get_user_input("\nNext servo angle (0-359) or 'q' to finish: ", valid_range=(0, 359))
            
            if next_input == 'q':
                print("\nScanning complete!")
                break
            
            current_servo_angle = next_input
        
        # Save results
        if len(all_points_3d) > 0:
            print("\n" + "="*70)
            print("SAVING 3D POINT CLOUD")
            print("="*70)
            print(f"  Total scans: {scan_count}")
            print(f"  Total points: {len(all_points_3d)}")
            
            # Ensure data directory exists
            os.makedirs("data", exist_ok=True)
            
            # Save CSV
            csv_path = "data/xyz_scan.csv"
            print(f"\n  Writing CSV: {csv_path}")
            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["quality", "angle_deg", "distance_mm", "servo_angle_deg", "x_m", "y_m", "z_m"])
                writer.writerows(all_points_3d)
            
            # Save PLY
            ply_path = "data/xyz_scan.ply"
            print(f"  Writing PLY: {ply_path}")
            with open(ply_path, "w") as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(all_points_3d)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("end_header\n")
                for _, _, _, _, x, y, z in all_points_3d:
                    f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
            
            print(f"\n  [OK] Done!")
            print(f"  Files saved:")
            print(f"    - {csv_path}")
            print(f"    - {ply_path}")
        else:
            print("\n[!] No data collected. No files saved.")
    
    except KeyboardInterrupt:
        print("\n\nScan interrupted by user (Ctrl+C)")
    
    except Exception as e:
        print(f"\n[X] Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\nShutting down...")
        try:
            lidar.stop()
        except:
            pass
        try:
            lidar.stop_motor()
        except:
            pass
        lidar.disconnect()
        print("Lidar disconnected.\n")


if __name__ == "__main__":
    main()

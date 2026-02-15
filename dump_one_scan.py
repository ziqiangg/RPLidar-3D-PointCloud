import csv, math, time, sys, os
import numpy as np
from rplidar import RPLidar
from utils.port_config import get_default_port

BAUD = 115200
MIN_ANGLE_RESOLUTION = 2.0  # Degrees - must have at least one point every 2 degrees
MIN_COVERAGE = 0.80  # Require 80% angle coverage (288° out of 360°) - relaxed for low-density lidars
MAX_SCANS_TO_MERGE = 10  # Merge up to 10 scans to fill coverage gaps
RELAXED_MAX_GAP = 10.0  # Accept max gaps up to 10° for sparse scans


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


def main():
    # Auto-detect port or use OS-appropriate default
    port = get_default_port()
    
    # Allow command-line override: python dump_one_scan.py COM4
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"Using port: {port}", flush=True)
    
    lidar = RPLidar(port, baudrate=BAUD, timeout=3)

    try:
        info = lidar.get_info()
        health = lidar.get_health()
        print("Info:", info, flush=True)
        print("Health:", health, flush=True)
        
        # Display scan mode info
        print("\nNOTE: RPLidar scan density varies by model:", flush=True)
        print("  - A1: ~360 points/scan in standard mode (0.5-1° resolution)", flush=True)
        print("  - A2: ~4000 points/scan (vary high density)", flush=True)
        print("  - If density is low, multiple scans will be merged automatically\n", flush=True)

        print("Starting motor...", flush=True)
        lidar.start_motor()
        time.sleep(2)

        print("Starting scan loop (waiting for first scan)...", flush=True)
        print(f"Requirement: {MIN_COVERAGE*100:.0f}% angle coverage, max gap {RELAXED_MAX_GAP:.0f}°", flush=True)
        print(f"Strategy: Merge up to {MAX_SCANS_TO_MERGE} scans if needed to achieve coverage", flush=True)
        t0 = time.time()

        # Collect and merge multiple scans to achieve required coverage
        collected_scans = []
        scan = None
        last_coverage = 0.0
        plateau_count = 0  # Track if coverage stops improving
        
        for i, s in enumerate(lidar.iter_scans(max_buf_meas=1500, min_len=50)):
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
            print(f"Scan #{i}: {message} {scan_info} [{elapsed:.2f}s]", flush=True)
            
            # Check if coverage is plateauing
            if abs(coverage - last_coverage) < 0.01:  # Less than 1% improvement
                plateau_count += 1
            else:
                plateau_count = 0
            last_coverage = coverage
            
            # Accept if we meet quality requirements
            if is_valid:
                scan = merged_scan
                print(f"[OK] Quality achieved! Merged {len(collected_scans)} scan(s)", flush=True)
                break
            
            # Early exit if coverage plateaued for 3 consecutive scans
            if plateau_count >= 3 and len(collected_scans) >= 3:
                scan = merged_scan
                print(f"[!] Coverage plateaued at {coverage*100:.1f}% after {len(collected_scans)} scans.", flush=True)
                print(f"    Likely hardware limitation or obstruction. Using current result.", flush=True)
                break
            
            # Stop collecting if we've gathered enough scans
            if len(collected_scans) >= MAX_SCANS_TO_MERGE:
                scan = merged_scan
                print(f"[!] Max scans reached ({MAX_SCANS_TO_MERGE}). Using merged result.", flush=True)
                break
            
            # Timeout after 15 seconds
            if elapsed > 15:
                scan = merged_scan if collected_scans else s
                print(f"[!] Timeout reached. Using best available ({len(collected_scans)} scans merged)", flush=True)
                break

        if not scan:
            raise RuntimeError("No scan data received at all.")
        
        # Final quality report
        is_valid, coverage, max_gap, point_count, message = validate_scan_quality(scan)
        print(f"\n{'='*60}", flush=True)
        print(f"FINAL SCAN QUALITY REPORT", flush=True)
        print(f"{'='*60}", flush=True)
        print(f"  Scans merged: {len(collected_scans)}", flush=True)
        print(f"  {message}", flush=True)
        print(f"  Requirements: >={MIN_COVERAGE*100:.0f}% coverage, <={RELAXED_MAX_GAP:.0f} deg max gap", flush=True)
        print(f"  Valid quality: {'YES [OK]' if is_valid else 'NO [X]'}", flush=True)
        if not is_valid:
            print(f"  ", flush=True)
            if coverage < MIN_COVERAGE:
                shortfall = (MIN_COVERAGE - coverage) * 360
                print(f"  [!] Coverage {shortfall:.0f} deg short of target", flush=True)
            if max_gap > RELAXED_MAX_GAP:
                print(f"  [!] Max gap {max_gap:.1f} deg exceeds {RELAXED_MAX_GAP:.0f} deg limit", flush=True)
            print(f"  ", flush=True)
            print(f"  Possible solutions:", flush=True)
            print(f"     - Increase MAX_SCANS_TO_MERGE (currently {MAX_SCANS_TO_MERGE})", flush=True)
            print(f"     - Check for physical obstructions blocking lidar", flush=True)
            print(f"     - Verify lidar is in express/boost mode if available", flush=True)
            print(f"  ", flush=True)
            print(f"  Data will still be saved but may have gaps in coverage.", flush=True)
        print(f"{'='*60}\n", flush=True)

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

        print(f"Writing {len(pts)} valid points (filtered from {len(scan)} total)...", flush=True)
        
        # Ensure data directory exists
        os.makedirs("data", exist_ok=True)

        with open("data/scan.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["quality", "angle_deg", "distance_mm", "x_m", "y_m", "z_m"])
            w.writerows(pts)

        with open("data/scan.ply", "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(pts)}\n")
            f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
            for _, _, _, x, y, z in pts:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        print("Done: data/scan.csv + data/scan.ply", flush=True)

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

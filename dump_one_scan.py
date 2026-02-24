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


def run_scan(port="auto", output_dir="data"):
    """
    Execute a 2D RPLidar scan and save results to files.
    
    This function performs the scan without printing to stdout/stderr,
    returning all status information in a structured dict.
    
    Args:
        port: Serial port path (e.g., "/dev/ttyUSB0", "COM3") or "auto" for auto-detection
        output_dir: Directory to save scan files (default: "data")
    
    Returns:
        dict: {
            'success': bool,          # True if scan completed successfully
            'point_count': int,       # Number of valid points captured
            'files': list,            # List of generated file paths
            'error': str or None,     # Error message if failed, None if success
            'message': str,           # Human-readable status message
            'scan_quality': dict      # Quality metrics (coverage, max_gap, etc.)
        }
    """
    try:
        # Auto-detect port if needed
        if port == "auto":
            port = get_default_port()
        
        # Connect to LiDAR
        lidar = RPLidar(port, baudrate=BAUD, timeout=3)

        try:
            # Get device info (no print, just verify connection)
            info = lidar.get_info()
            health = lidar.get_health()
            
            # Start motor
            lidar.start_motor()
            time.sleep(2)

            # Scan collection parameters
            t0 = time.time()
            collected_scans = []
            scan = None
            last_coverage = 0.0
            plateau_count = 0
            
            # Collect and merge scans
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
                
                # Check if coverage is plateauing
                if abs(coverage - last_coverage) < 0.01:
                    plateau_count += 1
                else:
                    plateau_count = 0
                last_coverage = coverage
                
                # Accept if we meet quality requirements
                if is_valid:
                    scan = merged_scan
                    break
                
                # Early exit if coverage plateaued
                if plateau_count >= 3 and len(collected_scans) >= 3:
                    scan = merged_scan
                    break
                
                # Stop if max scans reached
                if len(collected_scans) >= MAX_SCANS_TO_MERGE:
                    scan = merged_scan
                    break
                
                # Timeout after 15 seconds
                if elapsed > 15:
                    scan = merged_scan if collected_scans else s
                    break

            if not scan:
                return {
                    'success': False,
                    'point_count': 0,
                    'files': [],
                    'error': 'No scan data received',
                    'message': 'RPLidar did not return any scan data',
                    'scan_quality': {}
                }
            
            # Final quality assessment
            is_valid, coverage, max_gap, point_count, message = validate_scan_quality(scan)
            
            # Convert to XY coordinates (meters)
            pts = []
            for q, ang, dist in scan:
                if dist <= 0:
                    continue
                r = dist / 1000.0
                th = math.radians(ang)
                x = r * math.cos(th)
                y = r * math.sin(th)
                pts.append((q, ang, dist, x, y, 0.0))
            
            # Ensure output directory exists
            os.makedirs(output_dir, exist_ok=True)
            
            # Write CSV file
            csv_file = os.path.join(output_dir, "scan.csv")
            with open(csv_file, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["quality", "angle_deg", "distance_mm", "x_m", "y_m", "z_m"])
                w.writerows(pts)
            
            # Write PLY file
            ply_file = os.path.join(output_dir, "scan.ply")
            with open(ply_file, "w") as f:
                f.write("ply\nformat ascii 1.0\n")
                f.write(f"element vertex {len(pts)}\n")
                f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
                for _, _, _, x, y, z in pts:
                    f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
            
            # Build success message
            status_msg = f"Scan completed: {len(pts)} points"
            if len(collected_scans) > 1:
                status_msg += f" (merged {len(collected_scans)} scans)"
            
            return {
                'success': True,
                'point_count': len(pts),
                'files': [csv_file, ply_file],
                'error': None,
                'message': status_msg,
                'scan_quality': {
                    'is_valid': is_valid,
                    'coverage_percent': coverage * 100,
                    'max_gap_degrees': max_gap,
                    'scans_merged': len(collected_scans),
                    'quality_message': message
                }
            }

        finally:
            # Always cleanup LiDAR connection
            try:
                lidar.stop()
            except Exception:
                pass
            try:
                lidar.stop_motor()
            except Exception:
                pass
            lidar.disconnect()
    
    except Exception as e:
        # Return error result
        return {
            'success': False,
            'point_count': 0,
            'files': [],
            'error': str(e),
            'message': f"Scan failed: {str(e)}",
            'scan_quality': {}
        }


def main():
    """
    CLI wrapper for run_scan().
    Maintains backward compatibility for command-line usage.
    """
    # Auto-detect port or use command-line argument
    port = get_default_port()
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"Using port: {port}", flush=True)
    print("Starting scan...", flush=True)
    
    # Execute scan
    result = run_scan(port=port, output_dir="data")
    
    # Display results
    if result['success']:
        print(f"\n{'='*60}", flush=True)
        print(f"SCAN COMPLETED SUCCESSFULLY", flush=True)
        print(f"{'='*60}", flush=True)
        print(f"  Points captured: {result['point_count']}", flush=True)
        
        quality = result['scan_quality']
        if quality:
            print(f"  Scans merged: {quality.get('scans_merged', 1)}", flush=True)
            print(f"  Coverage: {quality.get('coverage_percent', 0):.1f}%", flush=True)
            print(f"  Max gap: {quality.get('max_gap_degrees', 0):.1f}°", flush=True)
            print(f"  Quality: {quality.get('quality_message', 'N/A')}", flush=True)
        
        print(f"\n  Files saved:", flush=True)
        for file in result['files']:
            print(f"    - {file}", flush=True)
        print(f"{'='*60}\n", flush=True)
    else:
        print(f"\n{'='*60}", flush=True)
        print(f"SCAN FAILED", flush=True)
        print(f"{'='*60}", flush=True)
        print(f"  Error: {result['error']}", flush=True)
        print(f"  Message: {result['message']}", flush=True)
        print(f"{'='*60}\n", flush=True)
        sys.exit(1)

if __name__ == "__main__":
    main()

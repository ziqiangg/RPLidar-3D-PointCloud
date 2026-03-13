"""
Robust 3D Scanning Module

Adapted from testrobust_3d_scan.py for MQTT integration.
"""

import sys
import os
import time
import math
import csv
import statistics
import traceback
from typing import List, Tuple, Optional, Callable
import numpy as np
import serial
from rplidar import RPLidar

# ==============================================================================
# DEFAULT CONFIGURATION
# ==============================================================================

DEFAULTS = {
    'lidar_port': '/dev/ttyUSB0',
    'lidar_baudrate': 115200,
    'lidar_pwm': 400,
    'servo_serial_port': '/dev/ttyACM0',
    'servo_baudrate': 115200,
    'servo_timeout': 5.0,
    'servo_settle_time': 0.5,
    'sweep_start': 0,
    'sweep_end': 180,
    'num_steps': 20,
    'max_scans': 40,
    'min_scans': 10,
    'plateau_iters': 5,
    'plateau_tol': 0.01,
    'bin_deg': 1.0,
    'min_dist': 150,
    'max_dist': 12000,
    'save_slice_files': False,
    'sor_neighbors': 12,
    'sor_std_ratio': 1.0,
    'sor_radius_m': 0.08,
    'voxel_size_m': 0.01,
}

# ==============================================================================
# HARDWARE CLASSES
# ==============================================================================

class PicoServoController:
    """
    Control servo via serial commands to Pico W.
    Format: ANGLE:<angle> -> DONE:<angle>
    """
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2.0) # Allow Pico to reset/initialize if needed
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        # Best-effort handshake to absorb startup/re-enumeration delay.
        self._wait_ready(max_wait=max(2.0, timeout + 2.0))
        self.current_angle = 0

    def _wait_ready(self, max_wait: float = 7.0):
        """Wait for Pico startup banner if present; continue if not seen."""
        start = time.time()
        while (time.time() - start) < max_wait:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            print(f"[Pico] {line}")
            if line == "PICO_SERVO_READY":
                return

    def _reconnect(self):
        """Reconnect serial link to recover from transient USB CDC issues."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2.0)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._wait_ready(max_wait=max(2.0, self.timeout + 2.0))
        
    def set_angle(self, angle: float):
        """
        Send angle command and wait for completion.
        """
        # Ensure angle is a float and formatted correctly
        cmd = f"ANGLE:{float(angle):.2f}\n"

        # First timeout can be too short right after USB reconnect/reset.
        attempts = 2
        for attempt in range(1, attempts + 1):
            try:
                # Flush input to clear any old messages
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass

                # Send command
                self.ser.write(cmd.encode('utf-8'))

                # Wait for DONE response
                wait_timeout = max(self.ser.timeout, 8.0) if attempt == 1 else max(self.ser.timeout, 5.0)
                start_time = time.time()
                while True:
                    if time.time() - start_time > wait_timeout:
                        raise TimeoutError(f"Timeout waiting for servo move to {angle}")

                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    print(f"[Pico] {line}")

                    if line.startswith("DONE:"):
                        self.current_angle = angle
                        return

                    if line.startswith("ERR:"):
                        raise RuntimeError(f"Pico servo error: {line}")

            except TimeoutError:
                if attempt >= attempts:
                    raise
                print("[Pico] Servo command timeout, reconnecting serial and retrying...")
                self._reconnect()
                continue
                        

    def detach(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

# ==============================================================================
# ROBUST SCANNING LOGIC
# ==============================================================================

def _bin_index(angle, bin_deg):
    """Convert angle to bin index."""
    return int((angle % 360.0) / bin_deg)


def _voxel_downsample_points(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Downsample points by averaging points that fall in the same voxel."""
    if points.size == 0 or voxel_size <= 0:
        return points

    xyz = points[:, :3]
    voxel_coords = np.floor(xyz / voxel_size).astype(np.int64)

    buckets = {}
    for i, key in enumerate(map(tuple, voxel_coords)):
        if key not in buckets:
            buckets[key] = []
        buckets[key].append(points[i])

    downsampled = []
    for pts in buckets.values():
        arr = np.asarray(pts)
        centroid_xyz = arr[:, :3].mean(axis=0)
        avg_dist = arr[:, 3].mean()
        avg_quality = int(round(arr[:, 4].mean()))
        downsampled.append([centroid_xyz[0], centroid_xyz[1], centroid_xyz[2], avg_dist, avg_quality])

    return np.asarray(downsampled, dtype=np.float64)


def _statistical_outlier_filter(
    points: np.ndarray,
    nb_neighbors: int,
    std_ratio: float,
    radius_m: float,
) -> np.ndarray:
    """
    Remove sparse outliers using local-neighborhood mean distance thresholding.

    This approximates statistical outlier removal without external heavy deps.
    """
    if points.size == 0:
        return points
    if len(points) <= max(3, nb_neighbors):
        return points

    xyz = points[:, :3]
    if radius_m <= 0:
        radius_m = DEFAULTS['sor_radius_m']

    # Spatial hash grid to avoid O(N^2) neighborhood search.
    cell_size = radius_m
    cell_index = np.floor(xyz / cell_size).astype(np.int64)
    grid = {}
    for i, key in enumerate(map(tuple, cell_index)):
        if key not in grid:
            grid[key] = []
        grid[key].append(i)

    neighbor_offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
    ]

    mean_neighbor_dists = np.full(len(points), np.inf, dtype=np.float64)

    for i in range(len(points)):
        cx, cy, cz = tuple(cell_index[i])
        candidates = []
        for dx, dy, dz in neighbor_offsets:
            key = (cx + dx, cy + dy, cz + dz)
            if key in grid:
                candidates.extend(grid[key])

        # Remove self and compute local distances.
        candidates = [j for j in candidates if j != i]
        if not candidates:
            continue

        diffs = xyz[candidates] - xyz[i]
        dists = np.linalg.norm(diffs, axis=1)
        dists = dists[dists <= radius_m]
        if len(dists) < 3:
            continue

        dists.sort()
        k = min(nb_neighbors, len(dists))
        mean_neighbor_dists[i] = float(np.mean(dists[:k]))

    finite = np.isfinite(mean_neighbor_dists)
    if not np.any(finite):
        return points

    mu = float(np.mean(mean_neighbor_dists[finite]))
    sigma = float(np.std(mean_neighbor_dists[finite]))
    threshold = mu + (std_ratio * sigma)
    keep = finite & (mean_neighbor_dists <= threshold)

    filtered = points[keep]
    return filtered if len(filtered) > 0 else points

def capture_robust_slice(lidar: RPLidar, slice_name: str, config: dict) -> List[Tuple[float, float, float]]:
    """
    Captures a single 360-degree slice using robust median filtering.
    Stops when coverage plateaus (stabilizes) or max scans reached.
    
    Returns: List of (quality, angle, distance)
    """
    print(f"  > Starting capture for {slice_name}...")
    
    # Extract config with defaults
    MAX_SCANS = config.get('max_scans', DEFAULTS['max_scans'])
    MIN_SCANS = config.get('min_scans', DEFAULTS['min_scans'])
    PLATEAU_ITERS = config.get('plateau_iters', DEFAULTS['plateau_iters'])
    PLATEAU_TOL = config.get('plateau_tol', DEFAULTS['plateau_tol'])
    BIN_DEG = config.get('bin_deg', DEFAULTS['bin_deg'])
    MIN_DIST = config.get('min_dist', DEFAULTS['min_dist'])
    MAX_DIST = config.get('max_dist', DEFAULTS['max_dist'])

    # Storage for binning
    bin_dists = {}
    bin_quals = {} # Keep max quality per bin for reference

    # State for plateau detection
    history_coverage = []
    
    scan_count = 0
    
    # Clear any stale buffer
    try:
        if hasattr(lidar, 'clean_input'):
            lidar.clean_input()
        elif hasattr(lidar, '_serial_port'):
            lidar._serial_port.reset_input_buffer()
    except:
        pass

    try:
        for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
            scan_count += 1
            
            # Process current scan
            for qual, ang, dist in scan:
                if dist < MIN_DIST or dist > MAX_DIST:
                    continue
                
                idx = _bin_index(ang, BIN_DEG)
                
                if idx not in bin_dists:
                    bin_dists[idx] = []
                bin_dists[idx].append(dist)
                
                # Track max quality
                if idx not in bin_quals or qual > bin_quals[idx]:
                    bin_quals[idx] = qual

            # --- Check Stop Conditions ---
            
            # 1. Calculate Coverage
            total_possible_bins = int(360.0 / BIN_DEG)
            filled_bins = len(bin_dists)
            coverage = filled_bins / total_possible_bins
            
            history_coverage.append(coverage)
            if len(history_coverage) > PLATEAU_ITERS:
                history_coverage.pop(0)

            # 2. Check Plateau (Stability)
            is_stable = False
            if len(history_coverage) == PLATEAU_ITERS and scan_count >= MIN_SCANS:
                # Check if all recent coverage values are close to the current one
                # If the variation is small, we have plateaued
                min_c = min(history_coverage)
                max_c = max(history_coverage)
                if (max_c - min_c) < PLATEAU_TOL:
                    is_stable = True
                    print(f"    - Plateau reached at {coverage*100:.1f}% coverage.")

            # Print status
            if scan_count % 5 == 0:
                print(f"    - Scan {scan_count}/{MAX_SCANS} | Cov: {coverage*100:.1f}%")

            # Exit triggers
            if is_stable:
                break
            if scan_count >= MAX_SCANS:
                print("    - Max scans reached.")
                break
                
    except Exception as e:
        print(f"Error during slice capture: {e}")

    # Compute Median Scan
    merged_points = []
    for idx, dists in bin_dists.items():
        if not dists:
            continue
        
        final_dist = statistics.median(dists)
        final_qual = bin_quals.get(idx, 0)
        final_ang = (idx * BIN_DEG) + (BIN_DEG / 2.0) # Center of bin
        
        merged_points.append((final_qual, final_ang, final_dist))
    
    print(f"  > Slice complete. {len(merged_points)} valid points.")
    return merged_points

# ==============================================================================
# MAIN 3D SCAN ROUTINE (Wrapped for MQTT)
# ==============================================================================

def run_scan(
    port: str = "auto",
    output_dir: str = "data",
    servo_config: Optional[dict] = None,
    scan_config: Optional[dict] = None,
    should_stop: Optional[Callable] = None,
    progress_callback: Optional[Callable] = None,
) -> dict:
    """
    Execute robust 3D scan with MQTT integration support.
    
    Args:
        port: Serial port
        output_dir: Output directory
        servo_config: Servo settings
        scan_config: Scan settings
        should_stop: Callback to check for stop signal
        progress_callback: Callback to report progress
    """
    if should_stop is None: should_stop = lambda: False
    if progress_callback is None: progress_callback = lambda x: None
    
    if servo_config is None: servo_config = {}
    if scan_config is None: scan_config = {}
    
    # Resolve parameters
    lidar_port = port if port != "auto" else DEFAULTS['lidar_port']
    
    # New Serial Servo Config
    servo_port = servo_config.get('serial_port', DEFAULTS['servo_serial_port'])
    if servo_port == "auto" or servo_port is None: 
        servo_port = DEFAULTS['servo_serial_port']
        
    servo_baud = int(servo_config.get('baudrate', DEFAULTS['servo_baudrate']))
    servo_timeout = float(servo_config.get('timeout', DEFAULTS['servo_timeout']))
    servo_settle = float(servo_config.get('settle_time', DEFAULTS['servo_settle_time']))

    # Use scan_config if available, else DEFAULTS
    sweep_start = float(scan_config.get('sweep_start', DEFAULTS['sweep_start']))
    sweep_end = float(scan_config.get('sweep_end', DEFAULTS['sweep_end']))
    num_steps = int(scan_config.get('num_steps', DEFAULTS['num_steps']))
    
    # Robust Capture Config
    capture_config = {
        'max_scans': scan_config.get('max_scans', DEFAULTS['max_scans']),
        'min_scans': scan_config.get('min_scans', DEFAULTS['min_scans']),
        'plateau_iters': scan_config.get('plateau_iters', DEFAULTS['plateau_iters']),
        'plateau_tol': scan_config.get('plateau_tol', DEFAULTS['plateau_tol']),
        'bin_deg': scan_config.get('bin_deg', DEFAULTS['bin_deg']),
        'min_dist': scan_config.get('min_dist', DEFAULTS['min_dist']),
        'max_dist': scan_config.get('max_dist', DEFAULTS['max_dist']),
    }
    
    # Initialize
    servo = None
    lidar = None
    all_points_3d = []
    generated_files = []
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    try:
        progress_callback({'stage': 'init', 'message': 'Initializing robust scan hardware...'})
        
        # 1. Setup Servo
        print(f"Connecting to servo on {servo_port} @ {servo_baud}...")
        servo = PicoServoController(servo_port, baudrate=servo_baud, timeout=servo_timeout)
        
        # 2. Setup Lidar
        lidar = RPLidar(lidar_port, baudrate=DEFAULTS['lidar_baudrate'])
        lidar.stop_motor()
        time.sleep(0.5)
        lidar.start_motor()
        lidar.motor_speed = DEFAULTS['lidar_pwm']
        
        time.sleep(3) # Spin up
        
        save_slice_files = bool(scan_config.get('save_slice_files', DEFAULTS['save_slice_files']))
        sor_neighbors = int(scan_config.get('sor_neighbors', DEFAULTS['sor_neighbors']))
        sor_std_ratio = float(scan_config.get('sor_std_ratio', DEFAULTS['sor_std_ratio']))
        sor_radius_m = float(scan_config.get('sor_radius_m', DEFAULTS['sor_radius_m']))
        voxel_size_m = float(scan_config.get('voxel_size_m', DEFAULTS['voxel_size_m']))

        steps = np.linspace(sweep_start, sweep_end, num_steps)
        
        for i, servo_angle in enumerate(steps):
            if should_stop():
                break
                
            progress_msg = f"Scanning slice {i+1}/{len(steps)} at {servo_angle:.1f}°"
            progress_callback({
                'stage': 'scanning', 
                'message': progress_msg, 
                'slice_index': i, 
                'total_slices': len(steps)
            })
            
            # Move Servo (will wait for DONE response)
            servo.set_angle(servo_angle)
            
            # Settle time after movement
            time.sleep(servo_settle)
            
            # Reset Lidar to clear buffer (crucial for accurate slices)
            try:
                lidar.stop()
                time.sleep(0.5)
                if hasattr(lidar, "clean_input"):
                    lidar.clean_input()
                elif hasattr(lidar, "_serial_port"):
                    lidar._serial_port.reset_input_buffer()
                time.sleep(0.5)
            except Exception as e:
                print(f"Lidar reset warning: {e}")
            
            # Capture Slice
            slice_points_2d = capture_robust_slice(lidar, f"Slice_{i}", capture_config)
            
            # Process to 3D and Save Slice
            slice_points_3d = []
            # Negate angle to fix left/right mirroring
            beta = math.radians(-servo_angle)
            
            for qual, lidar_angle_deg, r_mm in slice_points_2d:
                r_m = r_mm / 1000.0
                alpha = math.radians(lidar_angle_deg)
                
                # Projection Logic
                x = r_m * math.cos(alpha)
                z = r_m * math.sin(alpha)
                y = 0.0
                
                # Rotation
                y2 = y * math.cos(beta) - z * math.sin(beta)
                z2 = y * math.sin(beta) + z * math.cos(beta)
                
                # Point: (x, y2, z2, intensity)
                # Note: "x, y, z" in PLY usually maps to specific visualization axes.
                # Here we use x, y2, z2.
                
                pt = (x, y2, z2, r_m, qual)
                slice_points_3d.append(pt)
                all_points_3d.append(pt)
            
            if save_slice_files:
                timestamp = time.strftime("%H%M%S")
                slice_filename = f"robust_slice_{i}_{timestamp}.ply"
                slice_path = os.path.join(output_dir, slice_filename)

                with open(slice_path, 'w') as f:
                    f.write("ply\n")
                    f.write("format ascii 1.0\n")
                    f.write(f"element vertex {len(slice_points_3d)}\n")
                    f.write("property float x\n")
                    f.write("property float y\n")
                    f.write("property float z\n")
                    f.write("property float intensity\n")
                    f.write("end_header\n")
                    for p in slice_points_3d:
                        f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[4]}\n")
        
        # --- Automatic Stitching ---
        if all_points_3d:
            timestamp = time.strftime("%H%M%S")
            raw_points = np.asarray(all_points_3d, dtype=np.float64)

            progress_callback({'stage': 'processing', 'message': 'Applying SOR outlier filtering...'})
            sor_points = _statistical_outlier_filter(
                raw_points,
                nb_neighbors=max(3, sor_neighbors),
                std_ratio=max(0.1, sor_std_ratio),
                radius_m=max(0.01, sor_radius_m),
            )

            progress_callback({'stage': 'processing', 'message': 'Applying voxel grid downsampling...'})
            final_points = _voxel_downsample_points(sor_points, voxel_size=max(0.001, voxel_size_m))
            
            # 1. Save PLY
            stitched_filename = f"robust_scan_full_filtered_{timestamp}.ply"
            stitched_path = os.path.join(output_dir, stitched_filename)
            
            progress_callback({'stage': 'saving', 'message': 'Saving stitched point cloud...'})
            
            with open(stitched_path, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(final_points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("property float intensity\n")
                f.write("end_header\n")
                for p in final_points:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[4]}\n")
            
            generated_files.append(stitched_path)

            # 2. Save CSV
            stitched_csv_filename = f"robust_scan_full_filtered_{timestamp}.csv"
            stitched_csv_path = os.path.join(output_dir, stitched_csv_filename)

            with open(stitched_csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "z", "distance", "quality"])
                for p in final_points:
                    # p = (x, y, z, distance, quality)
                    writer.writerow([f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}", f"{p[3]:.4f}", int(p[4])])

            generated_files.append(stitched_csv_path)

            progress_callback({
                'stage': 'done',
                'message': (
                    f"Finalized point cloud: raw={len(raw_points)}, "
                    f"after_sor={len(sor_points)}, after_voxel={len(final_points)}"
                ),
                'point_count': int(len(final_points)),
            })
            
    except Exception as e:
        return {
            'success': False,
            'stopped': False,
            'point_count': len(all_points_3d),
            'files': generated_files,
            'error': str(e),
            'message': f"Robust scan failed: {e}",
            'scan_quality': {}
        }
    finally:
        if lidar:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        if servo:
            servo.detach()
            
    return {
        'success': True,
        'stopped': False,
        'point_count': len(final_points) if all_points_3d else 0,
        'files': generated_files,
        'error': None,
        'message': "Robust scan completed successfully (merged + SOR + voxel)",
        'scan_quality': {}
    }

if __name__ == "__main__":
    # Test run
    print("Running standalone robust scan...")
    res = run_scan(output_dir="data")
    print(res['message'])

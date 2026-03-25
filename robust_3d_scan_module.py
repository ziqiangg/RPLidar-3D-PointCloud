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
from utils.port_config import get_default_port, get_default_servo_port

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
    'num_steps': 91,
    'park_after_scan': True,
    'park_target_deg': None,
    'park_backoff_deg': 5.0,
    'park_settle_time': 0.25,
    'max_scans': 40,
    'min_scans': 10,
    'plateau_iters': 5,
    'plateau_tol': 0.01,
    'bin_deg': 1.0,
    'min_dist': 150,
    'max_dist': 12000,
}


def _is_io_error(exc: Exception) -> bool:
    """Return True for common serial I/O failures (Errno 5)."""
    text = str(exc)
    return isinstance(exc, OSError) or "Errno 5" in text or "Input/output error" in text


def _safe_lidar_teardown(lidar: Optional[RPLidar]):
    """Best-effort lidar shutdown; never raises."""
    if not lidar:
        return
    try:
        lidar.stop()
    except Exception:
        pass
    try:
        lidar.stop_motor()
    except Exception:
        pass
    try:
        lidar.disconnect()
    except Exception:
        pass


def _init_lidar(lidar_port: str, baudrate: int, motor_pwm: int, spinup_s: float = 3.0) -> RPLidar:
    """Initialize lidar and set motor speed."""
    lidar = RPLidar(lidar_port, baudrate=baudrate)
    try:
        lidar.stop_motor()
    except Exception:
        pass
    time.sleep(0.5)
    lidar.start_motor()
    lidar.motor_speed = motor_pwm
    time.sleep(spinup_s)
    return lidar

# ==============================================================================
# HARDWARE CLASSES
# ==============================================================================

class PicoServoController:
    """
    Control servo via serial commands to Pico W.
    Format: ANGLE:<angle> -> DONE:<angle>
    """
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2.0) # Allow Pico to reset/initialize if needed
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self.current_angle = 0
        
    def set_angle(self, angle: float):
        """
        Send angle command and wait for completion.
        """
        # Ensure angle is a float and formatted correctly
        cmd = f"ANGLE:{float(angle):.2f}\n"
        
        # Flush input to clear any old messages
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        
        # Send command
        self.ser.write(cmd.encode('utf-8'))
        
        # Wait for DONE response
        # We expect "DONE:<angle>"
        start_time = time.time()
        while True:
            if time.time() - start_time > self.ser.timeout:
                raise TimeoutError(f"Timeout waiting for servo move to {angle}")
                
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            print(f"[Pico] {line}")
            
            if line.startswith("DONE:"):
                try:
                    self.current_angle = float(line.split(":", 1)[1].strip())
                except Exception:
                    # Fall back to requested value if firmware returns non-numeric payload.
                    self.current_angle = float(angle)
                break

            if line.startswith("ERR:"):
                raise RuntimeError(f"Pico servo error: {line}")

    def set_policy(self, policy_name: str):
        """Select active motion policy on Pico firmware."""
        policy = str(policy_name).strip().upper()
        if not policy:
            raise ValueError("policy_name must be non-empty")

        cmd = f"POLICY:{policy}\n"

        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        self.ser.write(cmd.encode('utf-8'))

        start_time = time.time()
        while True:
            if time.time() - start_time > self.ser.timeout:
                raise TimeoutError(f"Timeout waiting for policy ack: {policy}")

            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            print(f"[Pico] {line}")
            if line.startswith("POLICY_SET:"):
                return
            if line.startswith("ERR:"):
                raise RuntimeError(f"Pico servo policy error: {line}")
                        

    def detach(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

# ==============================================================================
# ROBUST SCANNING LOGIC
# ==============================================================================

def _bin_index(angle, bin_deg):
    """Convert angle to bin index."""
    return int((angle % 360.0) / bin_deg)

def capture_robust_slice(lidar: RPLidar, slice_name: str, config: dict) -> Tuple[List[Tuple[float, float, float]], bool]:
    """
    Captures a single 360-degree slice using robust median filtering.
    Stops when coverage plateaus (stabilizes) or max scans reached.
    
    Returns: (List of (quality, angle, distance), had_io_error)
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
    had_io_error = False
    
    # Clear any stale buffer
    try:
        if hasattr(lidar, 'clean_input'):
            lidar.clean_input()
        elif hasattr(lidar, '_serial_port'):
            lidar._serial_port.reset_input_buffer()
    except:
        pass

    try:
        for i, scan in enumerate(lidar.iter_scans(max_buf_meas=3000)):
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
        had_io_error = _is_io_error(e)
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
    return merged_points, had_io_error


def _voxel_downsample_points(
    points: List[Tuple[float, float, float, float, float]],
    voxel_size_m: float,
) -> List[Tuple[float, float, float, float, float]]:
    """Downsample cloud by voxel grid while preserving average geometry and max quality."""
    if voxel_size_m <= 0.0 or len(points) <= 1:
        return points

    pts = np.asarray(points, dtype=np.float64)
    xyz = pts[:, :3]
    voxel_idx = np.floor(xyz / voxel_size_m).astype(np.int64)

    _, _, inverse = np.unique(voxel_idx, axis=0, return_index=True, return_inverse=True)
    voxel_count = int(inverse.max()) + 1 if inverse.size else 0
    if voxel_count == 0:
        return points

    counts = np.bincount(inverse, minlength=voxel_count).astype(np.float64)
    sum_x = np.bincount(inverse, weights=pts[:, 0], minlength=voxel_count)
    sum_y = np.bincount(inverse, weights=pts[:, 1], minlength=voxel_count)
    sum_z = np.bincount(inverse, weights=pts[:, 2], minlength=voxel_count)
    sum_d = np.bincount(inverse, weights=pts[:, 3], minlength=voxel_count)

    max_q = np.full(voxel_count, -np.inf, dtype=np.float64)
    for i, v_idx in enumerate(inverse):
        q = pts[i, 4]
        if q > max_q[v_idx]:
            max_q[v_idx] = q

    reduced = []
    for v_idx in range(voxel_count):
        c = counts[v_idx]
        if c <= 0:
            continue
        reduced.append(
            (
                float(sum_x[v_idx] / c),
                float(sum_y[v_idx] / c),
                float(sum_z[v_idx] / c),
                float(sum_d[v_idx] / c),
                float(max_q[v_idx]),
            )
        )

    return reduced


def _sor_filter_points(
    points: List[Tuple[float, float, float, float, float]],
    neighbors: int,
    std_ratio: float,
    radius_m: float,
    max_points: int,
    max_runtime_s: float,
) -> Tuple[List[Tuple[float, float, float, float, float]], str]:
    """Apply a guarded SOR pass using local radius neighborhoods via spatial hashing."""
    n_points = len(points)
    if n_points <= 4:
        return points, "sor_skipped_too_few_points"

    if n_points > max_points:
        return points, f"sor_skipped_point_cutoff_{n_points}>{max_points}"

    k = max(1, int(neighbors))
    radius = max(1e-6, float(radius_m))
    radius2 = radius * radius
    t0 = time.time()

    pts = np.asarray(points, dtype=np.float64)
    xyz = pts[:, :3]
    grid = np.floor(xyz / radius).astype(np.int64)

    buckets = {}
    for idx, cell in enumerate(grid):
        key = (int(cell[0]), int(cell[1]), int(cell[2]))
        buckets.setdefault(key, []).append(idx)

    offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
    ]

    mean_neighbor_dist = np.full(n_points, np.nan, dtype=np.float64)

    for i, cell in enumerate(grid):
        if max_runtime_s > 0 and (time.time() - t0) > max_runtime_s:
            return points, f"sor_skipped_timeout_{max_runtime_s:.1f}s"

        cx, cy, cz = int(cell[0]), int(cell[1]), int(cell[2])
        candidates = []
        for dx, dy, dz in offsets:
            candidates.extend(buckets.get((cx + dx, cy + dy, cz + dz), []))

        if len(candidates) <= 1:
            continue

        cand = np.asarray([j for j in candidates if j != i], dtype=np.int64)
        if cand.size == 0:
            continue

        diff = xyz[cand] - xyz[i]
        d2 = np.einsum('ij,ij->i', diff, diff)
        within = d2 <= radius2
        if np.any(within):
            d2 = d2[within]
        if d2.size == 0:
            continue

        k_eff = min(k, d2.size)
        nearest = np.partition(d2, k_eff - 1)[:k_eff]
        mean_neighbor_dist[i] = float(np.mean(np.sqrt(nearest)))

    valid = ~np.isnan(mean_neighbor_dist)
    valid_count = int(np.sum(valid))
    if valid_count < max(10, int(0.25 * n_points)):
        return points, "sor_skipped_insufficient_neighbors"

    valid_vals = mean_neighbor_dist[valid]
    mu = float(np.mean(valid_vals))
    sigma = float(np.std(valid_vals))
    threshold = mu + (float(std_ratio) * sigma)

    keep_mask = valid & (mean_neighbor_dist <= threshold)
    kept = int(np.sum(keep_mask))
    if kept < max(10, int(0.35 * n_points)):
        return points, "sor_skipped_over_prune_guard"

    keep_idx = np.where(keep_mask)[0]
    filtered = [points[i] for i in keep_idx]
    removed = n_points - kept
    return filtered, f"sor_applied_removed_{removed}"

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
    wait_for_step: Optional[Callable] = None,
    file_callback: Optional[Callable] = None
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
        wait_for_step: (Ignored in this robust mode, usually)
        file_callback: Callback to report generated file paths (for streaming)
    """
    if should_stop is None: should_stop = lambda: False
    if progress_callback is None: progress_callback = lambda x: None
    if file_callback is None: file_callback = lambda x: None
    
    if servo_config is None: servo_config = {}
    if scan_config is None: scan_config = {}
    
    # Resolve parameters
    lidar_port = port if port != "auto" else get_default_port()
    
    # New Serial Servo Config
    servo_port_cfg = servo_config.get('serial_port', DEFAULTS['servo_serial_port'])
    servo_port = get_default_servo_port(servo_port_cfg)
        
    servo_baud = int(servo_config.get('baudrate', DEFAULTS['servo_baudrate']))
    servo_timeout = float(servo_config.get('timeout', DEFAULTS['servo_timeout']))
    servo_settle = float(servo_config.get('settle_time', DEFAULTS['servo_settle_time']))
    base_io_retries = int(scan_config.get('slice_io_retries', 1))
    first_slice_extra_retries = int(scan_config.get('first_slice_extra_io_retries', 1))
    reinit_spinup_s = float(scan_config.get('io_reinit_spinup_s', 2.0))
    motor_pwm = int(scan_config.get('motor_pwm', DEFAULTS['lidar_pwm']))

    # Use scan_config if available, else DEFAULTS
    sweep_start = float(scan_config.get('sweep_start', DEFAULTS['sweep_start']))
    sweep_end = float(scan_config.get('sweep_end', DEFAULTS['sweep_end']))
    num_steps = int(scan_config.get('num_steps', DEFAULTS['num_steps']))
    park_after_scan = bool(scan_config.get('park_after_scan', DEFAULTS['park_after_scan']))
    park_target_raw = scan_config.get('park_target_deg', DEFAULTS['park_target_deg'])
    park_target_deg = None if park_target_raw in (None, "") else float(park_target_raw)
    park_backoff_deg = abs(float(scan_config.get('park_backoff_deg', DEFAULTS['park_backoff_deg'])))
    park_settle_time = float(scan_config.get('park_settle_time', DEFAULTS['park_settle_time']))
    
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
    save_slice_files = bool(scan_config.get('save_slice_files', False))
    last_servo_angle: Optional[float] = None
    steps = np.linspace(sweep_start, sweep_end, num_steps)
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    try:
        progress_callback({'stage': 'init', 'message': 'Initializing robust scan hardware...'})
        
        # 1. Setup Servo
        print(f"Connecting to servo on {servo_port} @ {servo_baud}...")
        servo = PicoServoController(servo_port, baudrate=servo_baud, timeout=servo_timeout)
        servo.set_policy("ROBUST_3D")
        
        # 2. Setup Lidar
        lidar = _init_lidar(
            lidar_port=lidar_port,
            baudrate=DEFAULTS['lidar_baudrate'],
            motor_pwm=motor_pwm,
            spinup_s=3.0,
        )
        
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
            last_servo_angle = float(servo_angle)
            
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
            
            # Capture slice with retry/reinit on serial I/O faults.
            retry_budget = base_io_retries + (first_slice_extra_retries if i == 0 else 0)
            attempt = 0
            slice_points_2d = []
            while True:
                slice_points_2d, had_io_error = capture_robust_slice(lidar, f"Slice_{i}", capture_config)

                if not had_io_error:
                    break

                if attempt >= retry_budget:
                    raise RuntimeError(
                        f"LiDAR I/O error persisted in slice {i + 1} after {attempt} retries"
                    )

                attempt += 1
                progress_callback({
                    'stage': 'recovering',
                    'message': (
                        f'LiDAR I/O error during slice {i + 1}; '
                        f'reinitializing and retrying ({attempt}/{retry_budget})...'
                    ),
                    'slice_index': i,
                    'total_slices': len(steps)
                })

                _safe_lidar_teardown(lidar)
                lidar = _init_lidar(
                    lidar_port=lidar_port,
                    baudrate=DEFAULTS['lidar_baudrate'],
                    motor_pwm=motor_pwm,
                    spinup_s=reinit_spinup_s,
                )
            
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
            
            # Optional local-only per-slice debug artifact.
            if save_slice_files:
                slice_filename = f"robust_slice_{i}.ply"
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
            raw_points = all_points_3d
            processed_points = raw_points

            # 1) Voxel downsample for payload reduction and local denoising.
            voxel_size_m = float(scan_config.get('voxel_size_m', 0.0))
            if voxel_size_m > 0.0:
                before = len(processed_points)
                processed_points = _voxel_downsample_points(processed_points, voxel_size_m)
                progress_callback({
                    'stage': 'filtering',
                    'message': (
                        f'Voxel downsample ({voxel_size_m:.4f}m): '
                        f'{before} -> {len(processed_points)} points'
                    ),
                    'point_count': len(processed_points),
                })

            # 2) Optional SOR pass with automatic safety cutoffs for RP5 latency.
            sor_neighbors = int(scan_config.get('sor_neighbors', 0))
            sor_std_ratio = float(scan_config.get('sor_std_ratio', 0.0))
            sor_radius_m = float(scan_config.get('sor_radius_m', 0.0))
            sor_max_points = int(scan_config.get('sor_max_points', 12000))
            sor_max_runtime_s = float(scan_config.get('sor_max_runtime_s', 2.5))
            sor_note = 'sor_disabled'
            if sor_neighbors > 0 and sor_std_ratio > 0.0 and sor_radius_m > 0.0:
                before = len(processed_points)
                processed_points, sor_note = _sor_filter_points(
                    processed_points,
                    neighbors=sor_neighbors,
                    std_ratio=sor_std_ratio,
                    radius_m=sor_radius_m,
                    max_points=sor_max_points,
                    max_runtime_s=sor_max_runtime_s,
                )
                progress_callback({
                    'stage': 'filtering',
                    'message': (
                        f'SOR ({sor_note}): {before} -> {len(processed_points)} points '
                        f'(k={sor_neighbors}, std={sor_std_ratio:.2f}, r={sor_radius_m:.3f}m)'
                    ),
                    'point_count': len(processed_points),
                })

            # 1. Save PLY
            stitched_filename = "robust_scan_full.ply"
            stitched_path = os.path.join(output_dir, stitched_filename)
            
            progress_callback({'stage': 'saving', 'message': 'Saving stitched point cloud...'})
            
            with open(stitched_path, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(processed_points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("property float intensity\n")
                f.write("end_header\n")
                for p in processed_points:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[4]}\n")
            
            generated_files.append(stitched_path)
            file_callback(stitched_path)

            # 2. Save CSV
            stitched_csv_filename = "robust_scan_full.csv"
            stitched_csv_path = os.path.join(output_dir, stitched_csv_filename)

            with open(stitched_csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "z", "distance", "quality"])
                for p in processed_points:
                    # p = (x, y, z, distance, quality)
                    writer.writerow([f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}", f"{p[3]:.4f}", int(p[4])])

            generated_files.append(stitched_csv_path)
            file_callback(stitched_csv_path)
            
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
        if (
            servo
            and park_after_scan
            and last_servo_angle is not None
            and (park_target_deg is not None or park_backoff_deg > 0.0)
            and len(steps) >= 1
        ):
            try:
                lower = min(sweep_start, sweep_end)
                upper = max(sweep_start, sweep_end)
                if park_target_deg is not None:
                    park_target = max(lower, min(upper, park_target_deg))
                    park_reason = f"absolute park target {park_target_deg:.1f}°"
                else:
                    direction = 0.0
                    if len(steps) >= 2:
                        direction = float(steps[-1] - steps[0])

                    if direction > 0.0:
                        park_target = max(lower, min(upper, last_servo_angle - park_backoff_deg))
                    elif direction < 0.0:
                        park_target = max(lower, min(upper, last_servo_angle + park_backoff_deg))
                    else:
                        park_target = last_servo_angle
                    park_reason = f"{park_backoff_deg:.1f}° backoff from scan endpoint"

                if abs(park_target - last_servo_angle) > 1e-6:
                    progress_callback({
                        'stage': 'parking',
                        'message': (
                            f'Parking servo at {park_target:.1f}° '
                            f'({park_reason})'
                        ),
                    })
                    servo.set_angle(park_target)
                    if park_settle_time > 0.0:
                        time.sleep(park_settle_time)
            except Exception as e:
                print(f"Servo park warning: {e}")

        _safe_lidar_teardown(lidar)
        if servo:
            servo.detach()
            
    return {
        'success': True,
        'stopped': False,
        'point_count': len(processed_points) if all_points_3d else 0,
        'files': generated_files,
        'error': None,
        'message': "Robust scan completed successfully",
        'scan_quality': {
            'raw_points': len(all_points_3d),
            'final_points': len(processed_points) if all_points_3d else 0,
            'voxel_size_m': float(scan_config.get('voxel_size_m', 0.0)),
            'sor_neighbors': int(scan_config.get('sor_neighbors', 0)),
            'sor_std_ratio': float(scan_config.get('sor_std_ratio', 0.0)),
            'sor_radius_m': float(scan_config.get('sor_radius_m', 0.0)),
        }
    }

if __name__ == "__main__":
    # Test run
    print("Running standalone robust scan...")
    res = run_scan(output_dir="data")
    print(res['message'])

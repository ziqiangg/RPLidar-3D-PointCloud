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
from rplidar import RPLidar
from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory

# ==============================================================================
# DEFAULT CONFIGURATION
# ==============================================================================

DEFAULTS = {
    'lidar_port': '/dev/ttyUSB0',
    'lidar_baudrate': 115200,
    'lidar_pwm': 400,
    'servo_pin': 18,
    'servo_min_angle': -90,
    'servo_max_angle': 90,
    'servo_min_pulse': 0.0005,
    'servo_max_pulse': 0.0025,
    'sweep_start': -75,
    'sweep_end': 87,
    'num_steps': 20,
    'max_scans': 40,
    'min_scans': 10,
    'plateau_iters': 5,
    'plateau_tol': 0.01,
    'bin_deg': 1.0,
    'min_dist': 150,
    'max_dist': 12000,
}

# ==============================================================================
# HARDWARE CLASSES
# ==============================================================================

class ServoTD8120MG:
    """
    TD-8120MG servo controller using gpiozero.
    Calibrated for precise angular positioning with custom pulse widths.
    """
    def __init__(self, pin: int, min_angle: float = -90, max_angle: float = 90,
                 min_pulse_width: float = 0.5, max_pulse_width: float = 2.5):
        factory = LGPIOFactory()
        self.min_angle = min_angle
        self.max_angle = max_angle
        
        self.servo = AngularServo(
            pin,
            min_angle=min_angle,
            max_angle=max_angle,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
            frame_width=0.02, # 20ms (50Hz)
            pin_factory=factory
        )
        self.current_angle = 0

    def set_angle(self, angle: float, wait_time: float = 0.5):
        if angle < self.min_angle: angle = self.min_angle
        if angle > self.max_angle: angle = self.max_angle
        self.servo.angle = angle
        self.current_angle = angle
        time.sleep(wait_time)

    def detach(self):
        self.servo.detach()

# ==============================================================================
# ROBUST SCANNING LOGIC
# ==============================================================================

def _bin_index(angle, bin_deg):
    """Convert angle to bin index."""
    return int((angle % 360.0) / bin_deg)

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
    lidar_port = port if port != "auto" else DEFAULTS['lidar_port']
    
    servo_pin = servo_config.get('pin', DEFAULTS['servo_pin'])
    servo_min = servo_config.get('physical_min', DEFAULTS['servo_min_angle'])
    servo_max = servo_config.get('physical_max', DEFAULTS['servo_max_angle'])
    servo_min_pulse = servo_config.get('min_pulse_width', DEFAULTS['servo_min_pulse'])
    servo_max_pulse = servo_config.get('max_pulse_width', DEFAULTS['servo_max_pulse'])
    
    # Use scan_config if available, else DEFAULTS
    sweep_start = scan_config.get('sweep_start', DEFAULTS['sweep_start'])
    sweep_end = scan_config.get('sweep_end', DEFAULTS['sweep_end'])
    num_steps = scan_config.get('num_steps', DEFAULTS['num_steps'])
    
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
        servo = ServoTD8120MG(
            servo_pin, 
            min_angle=servo_min, 
            max_angle=servo_max,
            min_pulse_width=servo_min_pulse,
            max_pulse_width=servo_max_pulse
        )
        
        # 2. Setup Lidar
        lidar = RPLidar(lidar_port, baudrate=DEFAULTS['lidar_baudrate'])
        lidar.stop_motor()
        time.sleep(0.5)
        lidar.start_motor()
        lidar.motor_speed = DEFAULTS['lidar_pwm']
        
        time.sleep(3) # Spin up
        
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
            
            # Move Servo
            servo.set_angle(servo_angle)
            
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
            beta = math.radians(servo_angle)
            
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
            
            # Save Slice PLY
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
            
            generated_files.append(slice_path)
            
            # STREAMING: Send this file immediately
            file_callback(slice_path)
            
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
        'point_count': len(all_points_3d),
        'files': generated_files, # Return all slice files
        'error': None,
        'message': "Robust scan completed successfully",
        'scan_quality': {}
    }

if __name__ == "__main__":
    # Test run
    print("Running standalone robust scan...")
    res = run_scan(output_dir="data")
    print(res['message'])

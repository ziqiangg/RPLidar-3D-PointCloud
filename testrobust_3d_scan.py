"""
Robust 3D Scanning Script
Combines the mechanical scanning logic of 'xyzscan_servo_auto.py' with the 
robust noise-filtering and plateau-detection algorithms from 'dump_one_scan.py'.

Key Features:
- Servo: TD-8120MG on GPIO 18 (Configured for -75° to 87°)
- Lidar: RPLidar A1 @ 400 PWM (Slow speed for high density)
- Algorithm: Median filtering over multiple revolutions per slice
- Stability: Plateau detection (5 iterations) prevents early exit
- Output: Points saved to 'data/robust_3d.ply' and 'data/robust_3d.csv'
"""

import sys
import os
import time
import math
import csv
import statistics
from typing import List, Tuple, Optional
import numpy as np
from rplidar import RPLidar
from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory

# ==============================================================================
# CONFIGURATION
# ==============================================================================

# Hardware Settings
LIDAR_PORT = '/dev/ttyUSB0'        # Change this if needed, or use 'auto' logic if preferred
LIDAR_BAUDRATE = 115200
LIDAR_PWM = 400            # Slower speed = higher point density (Default is ~660)
                           # 400 is roughly 4-5Hz scanning frequency

# Servo Settings
SERVO_PIN = 18
SERVO_MIN_ANGLE = -90      # Physical limit
SERVO_MAX_ANGLE = 90       # Physical limit
SERVO_MIN_PULSE = 0.0005   # 0.5ms
SERVO_MAX_PULSE = 0.0025   # 2.5ms

# Scanning Sweep Settings
SWEEP_START_ANGLE = -75    # Start angle for 3D scan
SWEEP_END_ANGLE = 87       # End angle for 3D scan
NUM_STEPS = 20	              # Number of vertical slices to take

# Robust Scanning Parameters (from dump_one_scan.py)
MAX_SCANS_PER_SLICE = 40   # Maximum revolutions to accumulate per slice
MIN_SCANS_PER_SLICE = 10   # Minimum revolutions before allowing exit
PLATEAU_ITERS = 5          # Number of iterations coverage must be stable to exit
PLATEAU_TOLERANCE = 0.01   # Coverage change < 1% considered stable
ANGLE_BIN_DEG = 1.0        # Resolution for binning points (1 degree)
MIN_DIST_MM = 150          # Minimum reliable distance
MAX_DIST_MM = 12000        # Maximum reliable distance

# Output
OUTPUT_DIR = "data"
OUTPUT_FILENAME = "robust_3d"

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

def capture_robust_slice(lidar: RPLidar, slice_name: str) -> List[Tuple[float, float, float]]:
    """
    Captures a single 360-degree slice using robust median filtering.
    Stops when coverage plateaus (stabilizes) or max scans reached.
    
    Returns: List of (quality, angle, distance)
    """
    print(f"  > Starting capture for {slice_name}...")
    
    # Storage for binning
    # bin_dists: Dict[bin_index, List[distances]]
    bin_dists = {}
    bin_quals = {} # Keep max quality per bin for reference

    # State for plateau detection
    history_coverage = []
    
    scan_count = 0
    start_time = time.time()

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
                if dist < MIN_DIST_MM or dist > MAX_DIST_MM:
                    continue
                
                idx = _bin_index(ang, ANGLE_BIN_DEG)
                
                if idx not in bin_dists:
                    bin_dists[idx] = []
                bin_dists[idx].append(dist)
                
                # Track max quality
                if idx not in bin_quals or qual > bin_quals[idx]:
                    bin_quals[idx] = qual

            # --- Check Stop Conditions ---
            
            # 1. Calculate Coverage
            total_possible_bins = int(360.0 / ANGLE_BIN_DEG)
            filled_bins = len(bin_dists)
            coverage = filled_bins / total_possible_bins
            
            history_coverage.append(coverage)
            if len(history_coverage) > PLATEAU_ITERS:
                history_coverage.pop(0)

            # 2. Check Plateau (Stability)
            is_stable = False
            if len(history_coverage) == PLATEAU_ITERS and scan_count >= MIN_SCANS_PER_SLICE:
                # Check if all recent coverage values are close to the current one
                # If the variation is small, we have plateaued
                min_c = min(history_coverage)
                max_c = max(history_coverage)
                if (max_c - min_c) < PLATEAU_TOLERANCE:
                    is_stable = True
                    print(f"    - Plateau reached at {coverage*100:.1f}% coverage.")

            # Print status
            if scan_count % 5 == 0:
                print(f"    - Scan {scan_count}/{MAX_SCANS_PER_SLICE} | Cov: {coverage*100:.1f}%")

            # Exit triggers
            if is_stable:
                break
            if scan_count >= MAX_SCANS_PER_SLICE:
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
        final_ang = (idx * ANGLE_BIN_DEG) + (ANGLE_BIN_DEG / 2.0) # Center of bin
        
        merged_points.append((final_qual, final_ang, final_dist))
    
    print(f"  > Slice complete. {len(merged_points)} valid points.")
    return merged_points

# ==============================================================================
# MAIN 3D SCAN ROUTINE
# ==============================================================================

def run_3d_scan():
    print("=== Robust 3D Lidar Scanner ===")
    print(f"Servo Pin: {SERVO_PIN}, Range: {SWEEP_START_ANGLE} to {SWEEP_END_ANGLE}, Steps: {NUM_STEPS}")
    print(f"Lidar PWM: {LIDAR_PWM}, Plateau Iters: {PLATEAU_ITERS}")
    
    # 1. Setup Servo
    try:
        servo = ServoTD8120MG(
            SERVO_PIN, 
            min_angle=SERVO_MIN_ANGLE, 
            max_angle=SERVO_MAX_ANGLE,
            min_pulse_width=SERVO_MIN_PULSE,
            max_pulse_width=SERVO_MAX_PULSE
        )
        print("Servo initialized.")
    except Exception as e:
        print(f"Failed to init servo: {e}")
        return

    # 2. Setup Lidar
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
    
    # Ensure motor is running at correct speed
    lidar.stop_motor()
    time.sleep(0.5)
    lidar.start_motor()
    lidar.motor_speed = LIDAR_PWM
    
    # Variable to store all 3D points: (x, y, z, dist, intensity)
    point_cloud = []
    
    steps = np.linspace(SWEEP_START_ANGLE, SWEEP_END_ANGLE, NUM_STEPS)
    
    try:
        print("Spinning up lidar... (waiting 3s)")
        time.sleep(3) # Let motor stabilize
        
        for i, servo_angle in enumerate(steps):
            print(f"\n--- Step {i+1}/{NUM_STEPS}: Servo Angle {servo_angle:.1f}° ---")
           
            # Move Servo
            servo.set_angle(servo_angle)
           
            print("Resetting lidar before slice")
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
           
            # Capture robust slice (2D data)
            current_slice = capture_robust_slice(lidar, f"Slice_{i}")

            # Convert to 3D and store
            for qual, lidar_angle_deg, r_mm in current_slice:
                r_m = r_mm / 1000.0

                # Lidar spins in its own vertical scan plane
                alpha = math.radians(lidar_angle_deg)
                beta = math.radians(servo_angle)

                # Point in the lidar's local vertical plane
                x = r_m * math.cos(alpha)
                z = r_m * math.sin(alpha)
                y = 0.0

                # Rotate the scan plane by the servo tilt
                y2 = y * math.cos(beta) - z * math.sin(beta)
                z2 = y * math.sin(beta) + z * math.cos(beta)

                point_cloud.append((x, y2, z2, r_m, qual))
							
						# --- PROJECTION LOGIC END ---

    except KeyboardInterrupt:
        print("\nAborted by user.")
    finally:
        print("\nShutting down...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        servo.detach()
    
    # 3. Save Data
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        
    # Generate timestamped filename
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename_base = f"{OUTPUT_FILENAME}_{timestamp}"
        
    # Save CSV
    csv_path = os.path.join(OUTPUT_DIR, f"{filename_base}.csv")
    print(f"Saving {len(point_cloud)} points to {csv_path}...")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z', 'r', 'quality'])
        for p in point_cloud:
            writer.writerow(p)
            
    # Save PLY
    ply_path = os.path.join(OUTPUT_DIR, f"{filename_base}.ply")
    print(f"Saving PLY to {ply_path}...")
    with open(ply_path, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(point_cloud)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property float intensity\n") # Use quality as intensity
        f.write("end_header\n")
        for p in point_cloud:
            # x, y, z, r, qual
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[4]}\n")
            
    print("Done!")

if __name__ == "__main__":
    run_3d_scan()

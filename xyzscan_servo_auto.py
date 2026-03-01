"""
Automated 3D scanning with RPLidar and TD-8120MG servo.

Performs synchronized scanning with FULL 360° horizontal × 360° vertical coverage:
- Servo steps through 9 positions (command angles -75° to +87° in 18° steps)
- Each position produces 20° physical rotation (180° servo sweep: 0° to 160°)
- RPLidar captures 360° vertical slice at each servo position
- Each vertical slice covers TWO horizontal directions (front + back)
- Result: 9 servo positions × 2 directions = 18 azimuth slices = 360° coverage

Horizontal coverage (18 azimuths at 20° intervals):
- Front hemisphere (elev 0-180°): 0°, 20°, 40°, 60°, 80°, 100°, 120°, 140°, 160°
- Back hemisphere (elev 180-360°): 180°, 200°, 220°, 240°, 260°, 280°, 300°, 320°, 340°

Coordinate system:
- Servo rotation = Base azimuth (horizontal angle, 0° to 160°)
- Lidar rotation = Elevation (vertical angle, 0° to 360°)
  - For elevation 0-180°: front hemisphere, azimuth = servo_angle
  - For elevation 180-360°: back hemisphere, azimuth = servo_angle + 180°
- Distance = Radial distance from lidar (meters)
- 3D Cartesian: (azimuth, elevation, radius) → (x, y, z)
- Minimum distance filter: 50mm (removes points too close to sensor)

Motor speed control:
- motor_pwm: 0-1023 (default=660)
- Lower PWM = slower rotation = more samples per angle = better coverage
- Recommended: 300-500 for high quality, 400 for best balance
"""

import csv
import math
import time
import os
from typing import Callable, Optional, Dict, List, Tuple
import numpy as np
from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory
from rplidar import RPLidar
from utils.port_config import get_default_port


class ServoTD8120MG:
    """
    TD-8120MG servo controller using gpiozero.
    
    Calibrated for precise angular positioning with custom pulse widths.
    """
    
    def __init__(self, pin: int, min_angle: float = -90, max_angle: float = 90,
                 min_pulse_width: float = 0.5, max_pulse_width: float = 2.5):
        """
        Initialize servo with calibrated parameters.
        
        Args:
            pin: GPIO pin number (BCM numbering)
            min_angle: Physical minimum angle (degrees)
            max_angle: Physical maximum angle (degrees)
            min_pulse_width: Minimum pulse width (milliseconds)
            max_pulse_width: Maximum pulse width (milliseconds)
        """
        factory = LGPIOFactory()
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 0
        
        # Create angular servo with custom pulse widths
        self.servo = AngularServo(
            pin,
            min_angle=min_angle,
            max_angle=max_angle,
            min_pulse_width=min_pulse_width / 1000,  # Convert ms to seconds
            max_pulse_width=max_pulse_width / 1000,
            frame_width=20 / 1000,  # 50Hz PWM (20ms period)
            pin_factory=factory
        )
    
    def set_angle(self, angle: float, wait_time: float = 0.05):
        """
        Set servo to specified angle with bounds checking.
        
        Args:
            angle: Target angle (degrees)
            wait_time: Settling time after movement (seconds)
        """
        # Clamp to physical limits
        if angle < self.min_angle:
            angle = self.min_angle
        elif angle > self.max_angle:
            angle = self.max_angle
        
        self.servo.angle = angle
        self.current_angle = angle
        time.sleep(wait_time)
    
    def get_current_angle(self) -> float:
        """Get current servo angle."""
        return self.current_angle
    
    def center(self):
        """Move servo to center position (0°)."""
        self.servo.angle = 0
        self.current_angle = 0
    
    def detach(self):
        """Detach servo (stop PWM signal)."""
        self.servo.angle = None
    
    def cleanup(self):
        """Cleanup servo resources."""
        self.servo.close()


def merge_scans(scans: List[List[Tuple]]) -> List[Tuple]:
    """
    Merge multiple RPLidar scans, keeping best measurement per angle bin.
    
    Args:
        scans: List of scan lists, where each scan is [(quality, angle, distance), ...]
    
    Returns:
        Merged scan with best measurements per 1-degree angle bin
    """
    angle_bins = {}
    
    for scan in scans:
        for quality, angle, dist in scan:
            # Filter: minimum distance (50mm), minimum quality (10/15)
            if dist <= 0 or dist < 50 or quality < 10:
                continue
            
            # 1-degree bins for better resolution
            bin_idx = int(angle / 1.0)
            
            # Keep measurement with best quality
            if bin_idx not in angle_bins or quality > angle_bins[bin_idx][0]:
                angle_bins[bin_idx] = (quality, angle, dist)
    
    return list(angle_bins.values())


def validate_scan_quality(scan: List[Tuple], min_resolution: float = 1.0,
                         min_coverage: float = 0.65) -> Tuple[bool, float, float, int, str]:
    """
    Validate scan quality with improved parameters for better coverage.
    
    Args:
        scan: List of (quality, angle, distance) tuples
        min_resolution: Maximum angle gap (degrees)
        min_coverage: Minimum coverage fraction (relaxed to 65%)
    
    Returns:
        (is_valid, coverage, max_gap, point_count, message)
    """
    if not scan or len(scan) < 10:
        return False, 0.0, 360.0, len(scan), "Too few points"
    
    angles = sorted([angle for _, angle, dist in scan if dist > 0])
    
    if len(angles) < 10:
        return False, 0.0, 360.0, len(angles), "Too few valid measurements"
    
    # Calculate coverage in 1-degree bins for finer resolution
    angle_bins = set()
    for angle in angles:
        bin_idx = int(angle / min_resolution)
        angle_bins.add(bin_idx)
    
    total_bins = int(360 / min_resolution)
    coverage = len(angle_bins) / total_bins
    
    # Find maximum gap
    max_gap = 0
    for i in range(len(angles)):
        next_i = (i + 1) % len(angles)
        gap = angles[next_i] - angles[i]
        if gap < 0:  # Wrapped around 360°
            gap = (360 - angles[i]) + angles[next_i]
        max_gap = max(max_gap, gap)
    
    # Relaxed validation for RPLidar A1 characteristics
    is_valid = coverage >= min_coverage and max_gap <= 20.0
    message = f"Coverage: {coverage*100:.1f}%, Max gap: {max_gap:.1f}°, Points: {len(angles)}"
    
    return is_valid, coverage, max_gap, len(angles), message


def capture_2d_slice(lidar: RPLidar, scan_config: dict, should_stop: Callable) -> Optional[List[Tuple]]:
    """
    Capture a single 2D slice using RPLidar (follows dump_one_scan.py logic).
    
    Args:
        lidar: RPLidar instance
        scan_config: Scan configuration parameters
        should_stop: Function returning True if scan should abort
    
    Returns:
        List of (quality, angle, distance) tuples, or None if failed/stopped
    """
    max_scans = scan_config.get('max_scans_to_merge', 30)  # Increased for better coverage
    slice_timeout = scan_config.get('slice_timeout', 45.0)  # More time for thorough scan
    plateau_patience = scan_config.get('plateau_patience', 7)  # More patience before accepting
    min_quality = scan_config.get('min_quality', 10)  # Minimum quality threshold
    
    t0 = time.time()
    collected_scans = []
    last_coverage = 0.0
    plateau_count = 0
    
    try:
        for i, s in enumerate(lidar.iter_scans(max_buf_meas=1500, min_len=50)):
            # Check for stop request
            if should_stop():
                return None
            
            elapsed = time.time() - t0
            
            # Skip empty scans
            if not s or len(s) < 50:
                continue
            
            collected_scans.append(s)
            merged_scan = merge_scans(collected_scans)
            
            # Validate quality
            is_valid, coverage, max_gap, point_count, message = validate_scan_quality(merged_scan)
            
            # Check coverage plateau (more sensitive threshold)
            if abs(coverage - last_coverage) < 0.005:  # Was 0.01, now more sensitive
                plateau_count += 1
            else:
                plateau_count = 0
            last_coverage = coverage
            
            # Accept if valid or plateaued (require more scans before plateau)
            if is_valid or (plateau_count >= plateau_patience and len(collected_scans) >= 5):
                return merged_scan
            
            # Stop conditions
            if len(collected_scans) >= max_scans or elapsed > slice_timeout:
                return merged_scan if collected_scans else None
    
    except Exception as e:
        return None
    
    return None


def run_scan(
    port: str = "auto",
    output_dir: str = "data",
    servo_config: Optional[dict] = None,
    scan_config: Optional[dict] = None,
    should_stop: Optional[Callable] = None,
    progress_callback: Optional[Callable] = None,
    wait_for_step: Optional[Callable] = None
) -> dict:
    """
    Execute automated 3D scan with synchronized servo and RPLidar.
    
    Args:
        port: Serial port for RPLidar ("auto" for auto-detect)
        output_dir: Directory to save output files
        servo_config: Servo configuration dict
        scan_config: Scan configuration dict
        should_stop: Function returning True to abort scan
        progress_callback: Function(progress_dict) for status updates
        wait_for_step: Function(step_request_dict) -> "continue"|"stop" for step control
    
    Returns:
        dict: {
            'success': bool,
            'stopped': bool,
            'point_count': int,
            'files': list,
            'error': str|None,
            'message': str,
            'scan_quality': dict
        }
    """
    # Default callbacks
    if should_stop is None:
        should_stop = lambda: False
    if progress_callback is None:
        progress_callback = lambda x: None
    if wait_for_step is None:
        wait_for_step = lambda x: "continue"
    
    # Default configs
    if servo_config is None:
        servo_config = {}
    if scan_config is None:
        scan_config = {}
    
    # Extract servo parameters (from config_rpi.yaml calibration)
    servo_pin = servo_config.get('pin', 18)
    servo_physical_min = servo_config.get('physical_min', -90)
    servo_physical_max = servo_config.get('physical_max', 90)
    servo_operational_min = servo_config.get('operational_min', -75)
    servo_operational_max = servo_config.get('operational_max', 87)
    servo_step_size = servo_config.get('step_size', 18)
    servo_step_delay = servo_config.get('step_delay', 0.05)
    servo_min_pulse = servo_config.get('min_pulse_width', 0.0005) * 1000  # Convert to ms
    servo_max_pulse = servo_config.get('max_pulse_width', 0.0025) * 1000
    release_while_waiting = servo_config.get('release_while_waiting', True)
    
    # Extract scan parameters
    num_steps = scan_config.get('num_steps', 9)
    physical_step_deg = scan_config.get('physical_step_deg', 20)
    inter_slice_sleep = scan_config.get('inter_slice_sleep', 1.0)
    step_permission_required = scan_config.get('step_permission_required', True)
    motor_pwm = scan_config.get('motor_pwm', 660)  # Default RPLidar PWM
    
    # Initialize variables
    servo = None
    lidar = None
    all_slices = []
    total_points = 0
    
    try:
        # Auto-detect port if needed
        if port == "auto":
            port = get_default_port()
        
        progress_callback({
            'stage': 'init',
            'message': f'Initializing servo and RPLidar on {port} (motor PWM: {motor_pwm})...',
            'slice_index': 0,
            'total_slices': num_steps
        })
        
        # Initialize servo
        servo = ServoTD8120MG(
            pin=servo_pin,
            min_angle=servo_physical_min,
            max_angle=servo_physical_max,
            min_pulse_width=servo_min_pulse,
            max_pulse_width=servo_max_pulse
        )
        
        # Move to starting position
        servo.set_angle(servo_operational_min, wait_time=0.5)
        current_command_angle = servo_operational_min
        
        # Initialize RPLidar
        lidar = RPLidar(port, baudrate=115200, timeout=3)
        info = lidar.get_info()
        health = lidar.get_health()
        
        # Set motor speed (lower = slower = better quality, range 0-1023)
        lidar.motor_speed = motor_pwm
        
        lidar.start_motor()
        time.sleep(4)  # Extended initial stabilization time for consistent motor speed
        
        progress_callback({
            'stage': 'scanning',
            'message': f'Starting 3D scan: {num_steps} slices × {physical_step_deg}° = 180° sweep',
            'slice_index': 0,
            'total_slices': num_steps
        })
        
        # Perform scan at each servo position
        for step_idx in range(num_steps):
            if should_stop():
                progress_callback({
                    'stage': 'stopped',
                    'message': 'Scan stopped by user',
                    'slice_index': step_idx,
                    'total_slices': num_steps
                })
                return {
                    'success': False,
                    'stopped': True,
                    'point_count': total_points,
                    'files': [],
                    'error': None,
                    'message': f'Scan stopped by user at slice {step_idx}/{num_steps}',
                    'scan_quality': {}
                }
            
            # Restart motor before each slice (except first, which already has motor running)
            if step_idx > 0:
                try:
                    # CRITICAL: Reset motor speed before restarting
                    lidar.motor_speed = motor_pwm
                    lidar.start_motor()
                    time.sleep(4)  # Extended stabilization time for consistent speed
                except Exception as e:
                    return {
                        'success': False,
                        'stopped': False,
                        'point_count': total_points,
                        'files': [],
                        'error': f'Failed to restart motor for slice {step_idx + 1}',
                        'message': f'Motor restart failed at slice {step_idx + 1}: {str(e)}',
                        'scan_quality': {}
                    }
            
            # Calculate Z-plane for this slice
            z_plane = step_idx * physical_step_deg
            
            progress_callback({
                'stage': 'slice_scan',
                'message': f'Scanning slice {step_idx + 1}/{num_steps} at azimuth {z_plane}° (servo={current_command_angle:.0f}°)',
                'slice_index': step_idx,
                'total_slices': num_steps,
                'azimuth': z_plane,
                'servo_angle': current_command_angle
            })
            
            # Capture 2D slice
            slice_data = capture_2d_slice(lidar, scan_config, should_stop)
            
            if slice_data is None:
                if should_stop():
                    return {
                        'success': False,
                        'stopped': True,
                        'point_count': total_points,
                        'files': [],
                        'error': None,
                        'message': f'Scan stopped during slice {step_idx + 1}/{num_steps}',
                        'scan_quality': {}
                    }
                else:
                    return {
                        'success': False,
                        'stopped': False,
                        'point_count': total_points,
                        'files': [],
                        'error': f'Failed to capture slice {step_idx + 1}',
                        'message': f'Slice {step_idx + 1}/{num_steps} failed: No valid data',
                        'scan_quality': {}
                    }
            
            # Validate slice quality
            is_valid, coverage, max_gap, point_count, quality_msg = validate_scan_quality(slice_data)
            
            # Convert to 3D coordinates for VERTICAL rotating lidar
            # The lidar spins in a VERTICAL PLANE, servo rotates that plane horizontally
            slice_3d = []
            
            for quality, elevation_deg, dist in slice_data:
                # Filter out points too close to lidar (< 50mm)
                if dist <= 0 or dist < 50:
                    continue
                
                r = dist / 1000.0  # mm to meters
                
                # Direct transformation: elevation describes position on vertical circle
                # RPLidar angle semantics:
                #   0° = forward-horizontal, 90° = up, 180° = back-horizontal, 270° = down
                # Servo angle (z_plane) = horizontal rotation of entire vertical plane
                
                elevation_rad = math.radians(elevation_deg)
                azimuth_rad = math.radians(z_plane)
                
                # Spherical to Cartesian conversion for vertically-mounted lidar:
                # 1. Project point onto vertical circle at current servo orientation
                horizontal_distance = r * math.cos(elevation_rad)  # radial distance from vertical axis
                z = r * math.sin(elevation_rad)  # height above horizontal plane
                
                # 2. Rotate horizontal component around vertical axis by servo angle
                x = horizontal_distance * math.cos(azimuth_rad)
                y = horizontal_distance * math.sin(azimuth_rad)
                
                slice_3d.append((quality, elevation_deg, dist, x, y, z))
            
            all_slices.extend(slice_3d)
            total_points += len(slice_3d)
            
            progress_callback({
                'stage': 'slice_complete',
                'message': f'Slice {step_idx + 1}/{num_steps} complete: {len(slice_3d)} points ({quality_msg})',
                'slice_index': step_idx,
                'total_slices': num_steps,
                'slice_points': len(slice_3d),
                'total_points': total_points,
                'coverage': coverage * 100,
                'quality': quality_msg
            })
            
            # CRITICAL: Stop motor and clean buffer before moving to next position
            # This prevents buffer overflow between slices
            if step_idx < num_steps - 1:
                try:
                    lidar.stop()
                    lidar.stop_motor()
                    time.sleep(0.5)  # Allow motor to fully stop
                    
                    # Explicitly clear serial input buffer
                    if hasattr(lidar, 'clean_input'):
                        lidar.clean_input()
                    elif hasattr(lidar, '_serial_port'):
                        lidar._serial_port.reset_input_buffer()
                        lidar._serial_port.reset_output_buffer()
                except Exception as e:
                    # Log but continue - buffer clear is best-effort
                    pass
            
            # Wait between slices
            time.sleep(inter_slice_sleep)
            
            # Move to next position (if not last slice)
            if step_idx < num_steps - 1:
                # Release PWM if configured
                if release_while_waiting and step_permission_required:
                    servo.detach()
                
                # Wait for step permission if required
                if step_permission_required:
                    next_command_angle = current_command_angle + servo_step_size
                    next_z_plane = (step_idx + 1) * physical_step_deg
                    
                    step_request = {
                        'slice_index': step_idx + 1,
                        'total_slices': num_steps,
                        'next_angle': next_z_plane,
                        'next_servo_angle': next_command_angle
                    }
                    
                    result = wait_for_step(step_request)
                    
                    if result == "stop" or should_stop():
                        progress_callback({
                            'stage': 'stopped',
                            'message': 'Scan stopped by user',
                            'slice_index': step_idx + 1,
                            'total_slices': num_steps
                        })
                        return {
                            'success': False,
                            'stopped': True,
                            'point_count': total_points,
                            'files': [],
                            'error': None,
                            'message': f'Scan stopped after slice {step_idx + 1}/{num_steps}',
                            'scan_quality': {}
                        }
                
                # Move servo to next position
                next_command_angle = current_command_angle + servo_step_size
                if next_command_angle <= servo_operational_max:
                    # CRITICAL: Use longer settle time - servo needs 300-500ms for 20° movement
                    servo.set_angle(next_command_angle, wait_time=0.5)
                    current_command_angle = next_command_angle
                else:
                    # Servo angle exceeds operational max - should not happen with proper config
                    progress_callback({
                        'stage': 'error',
                        'message': f'Servo angle {next_command_angle:.0f}° exceeds max {servo_operational_max}°'
                    })
                    return {
                        'success': False,
                        'stopped': False,
                        'point_count': total_points,
                        'files': [],
                        'error': f'Servo calibration error: {next_command_angle:.0f}° > {servo_operational_max}°',
                        'message': f'Servo angle exceeded operational limit',
                        'scan_quality': {}
                    }
        
        # All slices complete
        progress_callback({
            'stage': 'merging',
            'message': f'Merging {num_steps} slices ({total_points} points) into 3D point cloud...',
            'slice_index': num_steps,
            'total_slices': num_steps,
            'total_points': total_points
        })
        
        # Ensure output directory exists
        os.makedirs(output_dir, exist_ok=True)
        
        # Write CSV file
        csv_file = os.path.join(output_dir, "scan_3d.csv")
        with open(csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["quality", "elevation_deg", "distance_mm", "x_m", "y_m", "z_m"])
            writer.writerows(all_slices)
        
        # Write PLY file
        ply_file = os.path.join(output_dir, "scan_3d.ply")
        with open(ply_file, "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(all_slices)}\n")
            f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
            for _, _, _, x, y, z in all_slices:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
        
        progress_callback({
            'stage': 'complete',
            'message': f'3D scan complete: {total_points} points saved',
            'slice_index': num_steps,
            'total_slices': num_steps,
            'total_points': total_points
        })
        
        return {
            'success': True,
            'stopped': False,
            'point_count': total_points,
            'files': [csv_file, ply_file],
            'error': None,
            'message': f'3D scan completed: {num_steps} slices, {total_points} points',
            'scan_quality': {
                'num_slices': num_steps,
                'total_points': total_points,
                'points_per_slice': total_points // num_steps if num_steps > 0 else 0,
                'azimuth_range': f'0° to {(num_steps - 1) * physical_step_deg}°',
                'azimuth_step': physical_step_deg
            }
        }
    
    except Exception as e:
        return {
            'success': False,
            'stopped': False,
            'point_count': total_points,
            'files': [],
            'error': str(e),
            'message': f'3D scan failed: {str(e)}',
            'scan_quality': {}
        }
    
    finally:
        # Cleanup resources
        if lidar:
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
        
        if servo:
            try:
                servo.center()  # Return to center position
                time.sleep(0.5)
                servo.cleanup()
            except Exception:
                pass


if __name__ == "__main__":
    """Test 3D scanning standalone."""
    import sys
    
    print("="*70)
    print("3D RPLidar Scanner (Automated Servo Control)")
    print("="*70)
    print()
    
    # Simple progress callback for CLI
    def on_progress(progress: dict):
        stage = progress.get('stage', '')
        message = progress.get('message', '')
        print(f"[{stage.upper()}] {message}")
    
    # Simple step permission (auto-continue for CLI test)
    def on_step(request: dict):
        slice_idx = request.get('slice_index', 0)
        total = request.get('total_slices', 0)
        next_angle = request.get('next_angle', 0)
        print(f"\n>>> Ready for slice {slice_idx + 1}/{total} at {next_angle}°")
        print(">>> Press Enter to continue...")
        input()
        return "continue"
    
    # Detect port
    port = get_default_port()
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"Using port: {port}")
    print()
    
    # Execute scan
    result = run_scan(
        port=port,
        output_dir="data",
        servo_config={
            'pin': 18,
            'physical_min': -90,
            'physical_max': 90,
            'operational_min': -75,
            'operational_max': 87,
            'step_size': 18,
            'step_delay': 0.05,
            'min_pulse_width': 0.0005,
            'max_pulse_width': 0.0025,
            'release_while_waiting': True
        },
        scan_config={
            'num_steps': 9,
            'physical_step_deg': 20,
            'inter_slice_sleep': 1.0,
            'step_permission_required': True,
            'max_scans_to_merge': 30,  # Increased for better coverage
            'slice_timeout': 45.0,  # Increased timeout
            'plateau_patience': 7,  # More patience for coverage plateau
            'min_quality': 10,  # Minimum quality filter
            'motor_pwm': 400  # Slower motor for better coverage (0-1023, default=660)
        },
        progress_callback=on_progress,
        wait_for_step=on_step
    )
    
    # Display results
    print()
    print("="*70)
    if result['success']:
        print("3D SCAN COMPLETED SUCCESSFULLY")
        print("="*70)
        print(f"  Total points: {result['point_count']}")
        print(f"  Message: {result['message']}")
        quality = result.get('scan_quality', {})
        if quality:
            print(f"  Slices: {quality.get('num_slices', 0)}")
            print(f"  Points/slice: {quality.get('points_per_slice', 0)}")
            print(f"  Azimuth range: {quality.get('azimuth_range', 'N/A')}")
        print(f"\n  Files saved:")
        for file in result['files']:
            print(f"    - {file}")
    else:
        print("3D SCAN FAILED" if not result['stopped'] else "3D SCAN STOPPED")
        print("="*70)
        print(f"  Message: {result['message']}")
        if result['error']:
            print(f"  Error: {result['error']}")
    print("="*70)
    print()

#!/usr/bin/env python3
"""
Test coordinate transformation logic for 3D scanning.

This script verifies the mathematical correctness of converting
(azimuth, elevation, distance) → (x, y, z) coordinates.
"""

import math

def current_transform(z_plane, elevation_deg, dist):
    """Current transformation from xyzscan_servo_auto.py"""
    r = dist / 1000.0
    
    if elevation_deg > 180:
        effective_elevation = 360 - elevation_deg
        effective_azimuth = (z_plane + 180) % 360
    else:
        effective_elevation = elevation_deg
        effective_azimuth = z_plane
    
    elevation_rad = math.radians(effective_elevation)
    azimuth_rad = math.radians(effective_azimuth)
    
    horizontal_distance = r * math.cos(elevation_rad)
    z = r * math.sin(elevation_rad)
    
    x = horizontal_distance * math.cos(azimuth_rad)
    y = horizontal_distance * math.sin(azimuth_rad)
    
    return (x, y, z)


def correct_transform(z_plane, elevation_deg, dist):
    """Corrected transformation - no special handling for elevation > 180"""
    r = dist / 1000.0
    
    # Direct conversion - elevation describes position on vertical circle
    elevation_rad = math.radians(elevation_deg)
    azimuth_rad = math.radians(z_plane)
    
    # Project onto vertical circle, then rotate horizontally
    horizontal_distance = r * math.cos(elevation_rad)
    z = r * math.sin(elevation_rad)
    
    x = horizontal_distance * math.cos(azimuth_rad)
    y = horizontal_distance * math.sin(azimuth_rad)
    
    return (x, y, z)


def test_transformations():
    """Test both transformations with key cases."""
    
    print("="*80)
    print("COORDINATE TRANSFORMATION COMPARISON")
    print("="*80)
    print()
    
    # Test distance = 2000mm = 2.0m
    dist = 2000
    
    test_cases = [
        # (servo_angle, lidar_angle, expected_description)
        (0, 0, "Servo 0°, Lidar 0° → Forward horizontal"),
        (0, 45, "Servo 0°, Lidar 45° → Forward and up"),
        (0, 90, "Servo 0°, Lidar 90° → Straight up"),
        (0, 135, "Servo 0°, Lidar 135° → Backward and up"),
        (0, 180, "Servo 0°, Lidar 180° → Backward horizontal"),
        (0, 225, "Servo 0°, Lidar 225° → Backward and down"),
        (0, 270, "Servo 0°, Lidar 270° → Straight down"),
        (0, 315, "Servo 0°, Lidar 315° → Forward and down"),
        (20, 0, "Servo 20°, Lidar 0° → Forward-right horizontal"),
        (20, 90, "Servo 20°, Lidar 90° → Straight up"),
        (20, 180, "Servo 20°, Lidar 180° → Backward-left horizontal"),
        (90, 0, "Servo 90°, Lidar 0° → Right horizontal"),
        (180, 0, "Servo 180°, Lidar 0° → Backward horizontal"),
    ]
    
    errors_found = []
    
    for servo, lidar, description in test_cases:
        curr_x, curr_y, curr_z = current_transform(servo, lidar, dist)
        corr_x, corr_y, corr_z = correct_transform(servo, lidar, dist)
        
        # Check if results differ significantly
        diff = math.sqrt((curr_x - corr_x)**2 + (curr_y - corr_y)**2 + (curr_z - corr_z)**2)
        
        print(f"{description}")
        print(f"  Current:  x={curr_x:7.3f}m, y={curr_y:7.3f}m, z={curr_z:7.3f}m")
        print(f"  Correct:  x={corr_x:7.3f}m, y={corr_y:7.3f}m, z={corr_z:7.3f}m")
        
        if diff > 0.001:  # More than 1mm difference
            print(f"  ❌ DIFFERENCE: {diff:.3f}m")
            errors_found.append((servo, lidar, description, diff))
        else:
            print(f"  ✓ Match")
        print()
    
    print("="*80)
    if errors_found:
        print(f"❌ FOUND {len(errors_found)} TRANSFORMATION ERRORS:")
        print("="*80)
        for servo, lidar, desc, diff in errors_found:
            print(f"  • {desc}: {diff:.3f}m difference")
        print()
        print("The current transformation has bugs in elevation > 180° handling!")
        print()
        print("ISSUE: The code tries to mirror elevation AND flip azimuth,")
        print("       which causes points in the back hemisphere to appear")
        print("       on the wrong side, creating a cylindrical artifact.")
    else:
        print("✓ All transformations match - no errors found")
    print("="*80)
    print()
    
    # Demonstrate the cylinder problem
    print("="*80)
    print("CYLINDER PROBLEM DEMONSTRATION")
    print("="*80)
    print()
    print("For a point at the BACK of the vertical circle (elevation=180°):")
    print("with servo at different angles, we should get different X,Y:")
    print()
    
    for servo_angle in [0, 20, 40, 60, 80]:
        curr_x, curr_y, curr_z = current_transform(servo_angle, 180, 2000)
        corr_x, corr_y, corr_z = correct_transform(servo_angle, 180, 2000)
        
        print(f"Servo {servo_angle:3d}°:")
        print(f"  Current:  x={curr_x:7.3f}, y={curr_y:7.3f} ← Creates cylinder")
        print(f"  Should be: x={corr_x:7.3f}, y={corr_y:7.3f} ← Varies with servo")
    
    print()
    print("With current code, all back-hemisphere points have similar X,Y")
    print("regardless of servo angle → Creates cylinder artifact!")
    print("="*80)


if __name__ == "__main__":
    test_transformations()

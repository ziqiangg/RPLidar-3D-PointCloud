"""
Quick verification script for surface reconstruction.

Tests:
1. CSV loading
2. Coordinate system validation
3. Each reconstruction method
"""

import os
import sys
import importlib.util

def load_surface_reconstruction():
    """Load surface_reconstruction module directly."""
    spec = importlib.util.spec_from_file_location(
        "surface_reconstruction",
        os.path.join("viewer", "surface_reconstruction.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

# Load the module
sr = load_surface_reconstruction()
SurfaceReconstructor = sr.SurfaceReconstructor
reconstruct_surface = sr.reconstruct_surface

import open3d as o3d
import numpy as np
import csv


def test_coordinate_system(csv_file):
    """Verify that coordinate system is correct."""
    print("\n" + "="*60)
    print("COORDINATE SYSTEM VERIFICATION")
    print("="*60)
    
    if not os.path.exists(csv_file):
        print(f"❌ File not found: {csv_file}")
        return False
    
    # Load CSV
    points = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row['x_m'])
            y = float(row['y_m'])
            z = float(row['z_m'])
            points.append([x, y, z])
    
    points = np.array(points)
    
    print(f"\n✓ Loaded {len(points)} points")
    print(f"\nCoordinate ranges:")
    print(f"  X: [{points[:,0].min():.3f}, {points[:,0].max():.3f}] meters")
    print(f"  Y: [{points[:,1].min():.3f}, {points[:,1].max():.3f}] meters")
    print(f"  Z: [{points[:,2].min():.3f}, {points[:,2].max():.3f}] meters")
    
    # Check for cylindrical pattern (expected in raw data)
    radii = np.sqrt(points[:,0]**2 + points[:,1]**2)
    print(f"\nHorizontal radii (cylindrical check):")
    print(f"  Min: {radii.min():.3f}m")
    print(f"  Max: {radii.max():.3f}m")
    print(f"  Mean: {radii.mean():.3f}m")
    print(f"  Std: {radii.std():.3f}m")
    
    # High std deviation indicates cylindrical pattern
    if radii.std() > 0.5:
        print("\n✓ High radial variation detected:")
        print("  → Raw points show cylindrical sampling geometry (EXPECTED)")
        print("  → Use reconstruction to visualize actual 3D geometry")
    else:
        print("\n⚠ Low radial variation:")
        print("  → Points may already represent object surfaces")
    
    return True


def test_reconstruction_methods(csv_file):
    """Test all reconstruction methods."""
    print("\n" + "="*60)
    print("RECONSTRUCTION METHODS TEST")
    print("="*60)
    
    # Load point cloud
    points = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row['x_m'])
            y = float(row['y_m'])
            z = float(row['z_m'])
            points.append([x, y, z])
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
    
    methods = {
        "voxel": {"voxel_size": 0.05},
        "ball_pivoting": {},
        "poisson": {"depth": 8},
        "alpha_shape": {"alpha": 0.1}
    }
    
    results = {}
    
    for method, params in methods.items():
        print(f"\n--- Testing: {method.upper()} ---")
        try:
            geometry, desc = reconstruct_surface(pcd, method=method, **params, remove_outliers=False)
            
            if method == "voxel":
                num_voxels = len(geometry.get_voxels())
                print(f"✓ Success: {num_voxels} voxels created")
                results[method] = "✓ PASS"
            else:
                if hasattr(geometry, 'triangles'):
                    num_triangles = len(geometry.triangles)
                    if num_triangles > 0:
                        print(f"✓ Success: {num_triangles} triangles, {len(geometry.vertices)} vertices")
                        results[method] = "✓ PASS"
                    else:
                        print(f"⚠ Warning: No triangles created")
                        results[method] = "⚠ WARN"
                else:
                    print(f"⚠ Warning: Invalid geometry")
                    results[method] = "⚠ WARN"
                    
        except Exception as e:
            print(f"❌ Failed: {e}")
            results[method] = "❌ FAIL"
    
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    for method, result in results.items():
        print(f"  {method:15s}: {result}")
    
    return results


def main():
    print("="*60)
    print("RPLidar 3D Reconstruction Verification")
    print("="*60)
    
    # Check for scan data
    csv_file = "data/scan_3d.csv"
    
    if not os.path.exists(csv_file):
        print(f"\n⚠ Scan file not found: {csv_file}")
        print("\nPlease perform a 3D scan first:")
        print("  1. Run: python viewer/app.py")
        print("  2. Go to 'Scan Control' tab")
        print("  3. Select '3D Scan' and click 'Start Scan'")
        print("\nOr specify a different file:")
        print(f"  python {sys.argv[0]} <path_to_scan.csv>")
        return
    
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    
    # Run tests
    print(f"\nUsing scan file: {csv_file}\n")
    
    # Test 1: Coordinate system
    coord_ok = test_coordinate_system(csv_file)
    
    if not coord_ok:
        return
    
    # Test 2: Reconstruction methods
    results = test_reconstruction_methods(csv_file)
    
    # Final recommendations
    print("\n" + "="*60)
    print("RECOMMENDATIONS")
    print("="*60)
    print("\nFor room scanning:")
    print("  1. Use VOXEL GRID reconstruction (most reliable)")
    print("  2. Adjust voxel_size: 0.03-0.08m for best results")
    print("  3. Smaller voxels = more detail but slower")
    print("\nTo visualize your scan:")
    print(f"  python viewer/standalone_viewer.py {csv_file} 2 voxel 0.05")
    print("\nOr use the GUI:")
    print("  python viewer/app.py")
    print("  → Visualization tab → Load File → Select reconstruction method")
    
    print("\n✓ Verification complete!")


if __name__ == "__main__":
    main()

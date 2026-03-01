"""
Standalone point cloud viewer script.
Launched as a subprocess to avoid GLFW conflicts with main GUI.

Supports surface reconstruction for proper 3D geometry visualization.
"""
import sys
import os
import open3d as o3d
import numpy as np


def main():
    if len(sys.argv) < 2:
        print("Usage: python standalone_viewer.py <point_cloud_file> [point_size] [reconstruction_method] [voxel_size]")
        print("\nReconstruction methods:")
        print("  raw           - Show raw points only (default)")
        print("  voxel         - Voxel grid (best for room scanning)")
        print("  ball_pivoting - Ball-pivoting mesh")
        print("  poisson       - Poisson reconstruction")
        print("  alpha_shape   - Alpha shape mesh")
        print("\nExample: python standalone_viewer.py scan_3d.csv 2 voxel 0.05")
        sys.exit(1)
    
    file_path = sys.argv[1]
    point_size = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    method = sys.argv[3] if len(sys.argv) > 3 else "raw"
    voxel_size = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05
    
    if not os.path.exists(file_path):
        print(f"Error: File not found: {file_path}")
        sys.exit(1)
    
    print(f"Loading {file_path}...")
    
    # Load point cloud based on file type
    _, ext = os.path.splitext(file_path)
    ext = ext.lower()
    
    if ext == '.ply':
        pcd = o3d.io.read_point_cloud(file_path)
    elif ext == '.csv':
        # Load CSV (assuming x_m, y_m, z_m columns)
        import csv
        points = []
        with open(file_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row.get('x_m', row.get('x', 0)))
                y = float(row.get('y_m', row.get('y', 0)))
                z = float(row.get('z_m', row.get('z', 0)))
                points.append([x, y, z])
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
    else:
        print(f"Error: Unsupported file type: {ext}")
        sys.exit(1)
    
    if len(pcd.points) == 0:
        print("Error: Point cloud is empty!")
        sys.exit(1)
    
    filename = os.path.basename(file_path)
    print(f"\n{'='*50}")
    print(f"File: {filename}")
    print(f"Points: {len(pcd.points)}")
    print(f"Method: {method}")
    print(f"{'='*50}\n")
    
    # Prepare geometries for visualization
    geometries = []
    window_title = f"RPLidar Viewer - {filename}"
    
    if method == "raw":
        # Raw point cloud visualization
        if not pcd.has_colors():
            pcd.paint_uniform_color([1.0, 0.0, 0.0])
        geometries.append(pcd)
        window_title += " (Raw Points)"
        
    else:
        # Use surface reconstruction
        try:
            from surface_reconstruction import reconstruct_surface
            
            print(f"Performing {method} reconstruction...")
            geometry, desc = reconstruct_surface(
                pcd,
                method=method,
                voxel_size=voxel_size,
                remove_outliers=False
            )
            
            if method == "voxel":
                geometries.append(geometry)
                # Show original points in gray
                pcd_vis = o3d.geometry.PointCloud(pcd)
                pcd_vis.paint_uniform_color([0.3, 0.3, 0.3])
                geometries.append(pcd_vis)
            else:
                # For mesh methods
                if hasattr(geometry, 'triangles') and len(geometry.triangles) > 0:
                    geometry.paint_uniform_color([0.7, 0.7, 0.9])
                    geometries.append(geometry)
                    print(f"✓ Mesh created: {len(geometry.triangles)} triangles")
                else:
                    print("✗ Mesh creation failed, showing raw points")
                    pcd.paint_uniform_color([1.0, 0.0, 0.0])
                    geometries.append(pcd)
                
                # Show original points
                pcd_vis = o3d.geometry.PointCloud(pcd)
                pcd_vis.paint_uniform_color([0.8, 0.0, 0.0])
                geometries.append(pcd_vis)
            
            window_title += f" ({desc})"
            
        except Exception as e:
            print(f"Warning: Reconstruction failed: {e}")
            print("Falling back to raw points")
            pcd.paint_uniform_color([1.0, 0.0, 0.0])
            geometries.append(pcd)
            window_title += " (Raw - Reconstruction Failed)"
    
    # Add coordinate frame for reference
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.3,
        origin=[0, 0, 0]
    )
    geometries.append(coord_frame)
    
    print(f"\n{'='*50}")
    print(f"Launching viewer...")
    print(f"{'='*50}\n")
    
    # Launch viewer
    o3d.visualization.draw_geometries(
        geometries,
        window_name=window_title,
        width=1280,
        height=800,
        point_show_normal=False
    )
    
    print("Viewer closed.")


if __name__ == "__main__":
    main()

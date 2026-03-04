"""
Standalone point cloud viewer script.
Launched as a subprocess to avoid GLFW conflicts with main GUI.
"""
import sys
import os
import open3d as o3d
import numpy as np


def main():
    if len(sys.argv) < 2:
        print("Usage: python standalone_viewer.py <point_cloud_file> [point_size]")
        sys.exit(1)
    
    file_path = sys.argv[1]
    point_size = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    
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
    
    # Add colors if missing
    if not pcd.has_colors():
        colors = np.tile([1.0, 0.0, 0.0], (len(pcd.points), 1))
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    filename = os.path.basename(file_path)
    print(f"\n{'='*50}")
    print(f"Launching 3D Viewer: {filename}")
    print(f"Points: {len(pcd.points)}")
    print(f"Point Size: {point_size}")
    print(f"{'='*50}\n")
    
    # Launch viewer
    o3d.visualization.draw_geometries(
        [pcd],
        window_name=f"RPLidar Viewer - {filename}",
        width=1024,
        height=768,
        point_show_normal=False
    )
    
    print("Viewer closed.")


if __name__ == "__main__":
    main()

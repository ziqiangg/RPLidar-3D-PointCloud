"""
Test if the scan.ply file can be visualized with classic Open3D viewer
"""
import open3d as o3d
import numpy as np

print("Loading scan.ply...")
pcd = o3d.io.read_point_cloud("data/scan.ply")

if pcd.is_empty():
    print("ERROR: Point cloud is empty!")
    exit(1)

print(f"✓ Loaded {len(pcd.points)} points")
print(f"  Has colors: {pcd.has_colors()}")
print(f"  Has normals: {pcd.has_normals()}")

# Add bright colors if missing  
if not pcd.has_colors():
    print("  Adding bright red colors...")
    colors = np.tile([1.0, 0.0, 0.0], (len(pcd.points), 1))
    pcd.colors = o3d.utility.Vector3dVector(colors)

# Show bounds
bounds = pcd.get_axis_aligned_bounding_box()
print(f"  Bounds: min={bounds.min_bound}")
print(f"          max={bounds.max_bound}")
print(f"  Extent: {bounds.get_extent()}")
print(f"  Center: {bounds.get_center()}")

# Show first few points
points_array = np.asarray(pcd.points)
print(f"\nFirst 5 points:")
for i in range(min(5, len(points_array))):
    print(f"  {i}: {points_array[i]}")

print("\n" + "="*50)
print("Launching visualization window...")
print("- Points should appear as BRIGHT RED")
print("- Use mouse to rotate/pan/zoom")
print("- Close window to exit")
print("="*50 + "\n")

# Use the classic visualizer
o3d.visualization.draw_geometries(
    [pcd],
    window_name="Scan PLY Test - Classic Viewer",
    width=1024,
    height=768,
    point_show_normal=False
)

print("\nVisualization closed.")

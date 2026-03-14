"""
Standalone point cloud viewer script.
Launched as a subprocess to avoid GLFW conflicts with main GUI.
"""
import sys
import os
import open3d as o3d
import numpy as np


def _estimate_axis_size(points_np: np.ndarray) -> float:
    """Estimate a readable axis size from cloud bounds with sane fallbacks."""
    if points_np.size == 0:
        return 0.10

    mins = points_np.min(axis=0)
    maxs = points_np.max(axis=0)
    extent = maxs - mins
    max_extent = float(np.max(extent)) if extent.size else 0.0

    if max_extent <= 1e-6:
        return 0.10

    # About 15% of cloud span keeps the frame visible but not dominant.
    return max(0.05, min(0.60, 0.05 * max_extent))


def _create_axis_labels(axis_size: float) -> o3d.geometry.LineSet:
    """Create simple geometric X/Y/Z labels near the positive axis tips."""
    pts = []
    lines = []
    colors = []

    h = 0.10 * axis_size
    tip = 1.15 * axis_size

    def add_line(p0, p1, color):
        i0 = len(pts)
        pts.append(p0)
        i1 = len(pts)
        pts.append(p1)
        lines.append([i0, i1])
        colors.append(color)

    # X (red), drawn in YZ plane at +X tip
    add_line([tip, -h, -h], [tip, h, h], [1.0, 0.0, 0.0])
    add_line([tip, -h, h], [tip, h, -h], [1.0, 0.0, 0.0])

    # Y (green), drawn in XZ plane at +Y tip
    add_line([-h, tip, h], [0.0, tip, 0.0], [0.0, 1.0, 0.0])
    add_line([h, tip, h], [0.0, tip, 0.0], [0.0, 1.0, 0.0])
    add_line([0.0, tip, 0.0], [0.0, tip, -h], [0.0, 1.0, 0.0])

    # Z (blue), drawn in XY plane at +Z tip
    add_line([-h, h, tip], [h, h, tip], [0.0, 0.4, 1.0])
    add_line([h, h, tip], [-h, -h, tip], [0.0, 0.4, 1.0])
    add_line([-h, -h, tip], [h, -h, tip], [0.0, 0.4, 1.0])

    label_lines = o3d.geometry.LineSet()
    label_lines.points = o3d.utility.Vector3dVector(np.asarray(pts, dtype=np.float64))
    label_lines.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    label_lines.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=np.float64))
    return label_lines


def _is_2d_scan(file_path: str, points_np: np.ndarray) -> bool:
    """Detect 2D scan outputs so viewer can preserve pre-axes rendering."""
    name = os.path.basename(file_path).lower()
    if name in ("scan.csv", "scan.ply"):
        return True

    if points_np.size == 0:
        return False

    # 2D pipeline outputs z ~ 0 for all points.
    z_span = float(np.max(points_np[:, 2]) - np.min(points_np[:, 2]))
    return z_span <= 1e-6


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
    
    # Add directional XYZ axes at the sensor origin.
    # This matches the merged-cloud math in robust_3d_scan_module.py where
    # points are expressed in a single global frame around (0, 0, 0).
    points_np = np.asarray(pcd.points)
    show_axes = not _is_2d_scan(file_path, points_np)
    axes = None
    axis_labels = None
    if show_axes:
        axis_size = _estimate_axis_size(points_np)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=axis_size,
            origin=[0.0, 0.0, 0.0],
        )
        axis_labels = _create_axis_labels(axis_size)
        print(f"{'='*50}")
        print("Controls:")
        print("  A -> toggle axes + XYZ labels")
        print(f"{'='*50}\n")

    # Launch viewer with explicit render options so point size is honored.
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(
        window_name=f"RPLidar Viewer - {filename}",
        width=1024,
        height=768,
        visible=True,
    )
    vis.add_geometry(pcd)
    if show_axes:
        vis.add_geometry(axes)
        vis.add_geometry(axis_labels)

    state = {'axes_visible': show_axes}

    def _toggle_axes(v):
        if not show_axes:
            return False
        if state['axes_visible']:
            v.remove_geometry(axes, reset_bounding_box=False)
            v.remove_geometry(axis_labels, reset_bounding_box=False)
            state['axes_visible'] = False
            print("[VIEWER] Axes hidden")
        else:
            v.add_geometry(axes, reset_bounding_box=False)
            v.add_geometry(axis_labels, reset_bounding_box=False)
            state['axes_visible'] = True
            print("[VIEWER] Axes shown")
        return False

    if show_axes:
        vis.register_key_callback(ord('A'), _toggle_axes)

    render_opt = vis.get_render_option()
    render_opt.point_size = float(max(1, point_size))

    vis.run()
    vis.destroy_window()
    
    print("Viewer closed.")


if __name__ == "__main__":
    main()

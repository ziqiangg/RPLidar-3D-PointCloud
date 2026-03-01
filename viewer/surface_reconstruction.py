"""
Surface Reconstruction Module for RPLidar Point Clouds

When scanning with a vertically-mounted LiDAR rotated about a fixed point,
raw points naturally lie on cylindrical surfaces (sampling geometry artifact).
This module reconstructs the actual 3D environment geometry.

Reconstruction Methods:
1. Voxel Grid: Fast, shows volumetric occupancy (best for rooms/structures)
2. Ball Pivoting: Surface mesh for sparse data
3. Poisson: Dense surface reconstruction (requires normals)
4. Alpha Shapes: Concave hull reconstruction
"""

import numpy as np
import open3d as o3d
from typing import Tuple, Optional


class SurfaceReconstructor:
    """Reconstructs 3D surfaces from LiDAR point clouds."""
    
    def __init__(self, pcd: o3d.geometry.PointCloud):
        """
        Initialize reconstructor with point cloud.
        
        Args:
            pcd: Open3D point cloud with x,y,z coordinates
        """
        self.pcd = pcd
        self.num_points = len(pcd.points)
        
    def voxel_grid(self, voxel_size: float = 0.05) -> o3d.geometry.VoxelGrid:
        """
        Create voxel grid representation (RECOMMENDED for room scanning).
        
        Bins points into 3D voxels - a cube room will appear as a filled block.
        This is the most intuitive visualization for architectural spaces.
        
        Args:
            voxel_size: Size of each voxel in meters (default: 5cm)
        
        Returns:
            VoxelGrid geometry showing occupied space
        """
        print(f"[VOXEL] Creating voxel grid (voxel_size={voxel_size}m)...")
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
            self.pcd,
            voxel_size=voxel_size
        )
        num_voxels = len(voxel_grid.get_voxels())
        print(f"[VOXEL] Created {num_voxels} voxels from {self.num_points} points")
        return voxel_grid
    
    def ball_pivoting(self, radii: Optional[list] = None) -> Tuple[o3d.geometry.TriangleMesh, bool]:
        """
        Ball-Pivoting Algorithm for surface meshing (good for sparse data).
        
        Creates triangular mesh by "rolling" balls of varying radii.
        Works well with uneven point density.
        
        Args:
            radii: List of ball radii to use (auto-calculated if None)
        
        Returns:
            (mesh, success): Triangle mesh and success flag
        """
        print(f"[BALL_PIVOT] Starting ball-pivoting reconstruction...")
        
        # Estimate normals if not present
        if not self.pcd.has_normals():
            print("[BALL_PIVOT] Computing normals...")
            self.pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=0.1, max_nn=30
                )
            )
            self.pcd.orient_normals_consistent_tangent_plane(15)
        
        # Calculate radii based on point cloud scale
        if radii is None:
            distances = self.pcd.compute_nearest_neighbor_distance()
            avg_dist = np.mean(distances)
            radii = [avg_dist * 1.5, avg_dist * 2.5, avg_dist * 4.0]
            print(f"[BALL_PIVOT] Auto radii: {[f'{r:.4f}' for r in radii]}")
        
        # Perform ball pivoting
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            self.pcd,
            o3d.utility.DoubleVector(radii)
        )
        
        num_triangles = len(mesh.triangles)
        success = num_triangles > 0
        
        if success:
            print(f"[BALL_PIVOT] Created mesh: {num_triangles} triangles, {len(mesh.vertices)} vertices")
            # Compute vertex normals for proper shading
            mesh.compute_vertex_normals()
        else:
            print("[BALL_PIVOT] Failed to create mesh (no triangles)")
        
        return mesh, success
    
    def poisson(self, depth: int = 9) -> Tuple[o3d.geometry.TriangleMesh, bool]:
        """
        Poisson surface reconstruction (best for dense, complete scans).
        
        Creates smooth, watertight mesh. Requires oriented normals.
        May fill holes and extrapolate beyond actual data.
        
        Args:
            depth: Octree depth (higher = more detail, slower). Range: 6-12
        
        Returns:
            (mesh, success): Triangle mesh and success flag
        """
        print(f"[POISSON] Starting Poisson reconstruction (depth={depth})...")
        
        # Estimate normals if not present
        if not self.pcd.has_normals():
            print("[POISSON] Computing normals...")
            self.pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=0.1, max_nn=30
                )
            )
            self.pcd.orient_normals_consistent_tangent_plane(15)
        
        # Perform Poisson reconstruction
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            self.pcd,
            depth=depth,
            width=0,
            scale=1.1,
            linear_fit=False
        )
        
        # Remove low-density vertices (outlier removal)
        densities = np.asarray(densities)
        density_threshold = np.quantile(densities, 0.01)
        vertices_to_remove = densities < density_threshold
        mesh.remove_vertices_by_mask(vertices_to_remove)
        
        num_triangles = len(mesh.triangles)
        success = num_triangles > 0
        
        if success:
            print(f"[POISSON] Created mesh: {num_triangles} triangles, {len(mesh.vertices)} vertices")
            mesh.compute_vertex_normals()
        else:
            print("[POISSON] Failed to create mesh")
        
        return mesh, success
    
    def alpha_shape(self, alpha: float = 0.1) -> Tuple[o3d.geometry.TriangleMesh, bool]:
        """
        Alpha shape (concave hull) reconstruction.
        
        Creates mesh wrapping around point cloud with given "tightness".
        Good for irregular shapes.
        
        Args:
            alpha: Tightness parameter (smaller = tighter fit)
        
        Returns:
            (mesh, success): Triangle mesh and success flag
        """
        print(f"[ALPHA_SHAPE] Creating alpha shape (alpha={alpha})...")
        
        try:
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
                self.pcd,
                alpha=alpha
            )
            
            num_triangles = len(mesh.triangles)
            success = num_triangles > 0
            
            if success:
                print(f"[ALPHA_SHAPE] Created mesh: {num_triangles} triangles")
                mesh.compute_vertex_normals()
            else:
                print("[ALPHA_SHAPE] Failed to create mesh")
            
            return mesh, success
        except Exception as e:
            print(f"[ALPHA_SHAPE] Error: {e}")
            return o3d.geometry.TriangleMesh(), False
    
    def statistical_outlier_removal(self, nb_neighbors: int = 20, std_ratio: float = 2.0) -> o3d.geometry.PointCloud:
        """
        Remove outlier points before reconstruction.
        
        Args:
            nb_neighbors: Number of neighbors to analyze
            std_ratio: Standard deviation threshold
        
        Returns:
            Filtered point cloud
        """
        print(f"[FILTER] Removing outliers (nb_neighbors={nb_neighbors}, std_ratio={std_ratio})...")
        cl, ind = self.pcd.remove_statistical_outlier(nb_neighbors, std_ratio)
        print(f"[FILTER] Kept {len(cl.points)}/{self.num_points} points")
        return cl
    
    def downsample(self, voxel_size: float = 0.02) -> o3d.geometry.PointCloud:
        """
        Downsample point cloud for faster processing.
        
        Args:
            voxel_size: Voxel size for downsampling (meters)
        
        Returns:
            Downsampled point cloud
        """
        print(f"[DOWNSAMPLE] Downsampling (voxel_size={voxel_size}m)...")
        downsampled = self.pcd.voxel_down_sample(voxel_size)
        print(f"[DOWNSAMPLE] {len(downsampled.points)}/{self.num_points} points")
        return downsampled


def reconstruct_surface(
    pcd: o3d.geometry.PointCloud,
    method: str = "voxel",
    voxel_size: float = 0.05,
    depth: int = 9,
    alpha: float = 0.1,
    remove_outliers: bool = True
) -> Tuple[o3d.geometry.Geometry, str]:
    """
    Convenience function for surface reconstruction.
    
    Args:
        pcd: Input point cloud
        method: "voxel", "ball_pivoting", "poisson", or "alpha_shape"
        voxel_size: Voxel size for voxel grid (meters)
        depth: Octree depth for Poisson
        alpha: Alpha parameter for alpha shapes
        remove_outliers: Whether to remove outliers before reconstruction
    
    Returns:
        (geometry, description): Reconstructed geometry and description string
    """
    reconstructor = SurfaceReconstructor(pcd)
    
    # Optional outlier removal
    if remove_outliers and method != "voxel":
        pcd_filtered = reconstructor.statistical_outlier_removal()
        reconstructor = SurfaceReconstructor(pcd_filtered)
    
    # Perform reconstruction
    if method == "voxel":
        geometry = reconstructor.voxel_grid(voxel_size)
        desc = f"Voxel Grid ({voxel_size}m voxels)"
    elif method == "ball_pivoting":
        geometry, success = reconstructor.ball_pivoting()
        desc = "Ball Pivoting Mesh" if success else "Ball Pivoting (Failed)"
    elif method == "poisson":
        geometry, success = reconstructor.poisson(depth)
        desc = f"Poisson Mesh (depth={depth})" if success else "Poisson (Failed)"
    elif method == "alpha_shape":
        geometry, success = reconstructor.alpha_shape(alpha)
        desc = f"Alpha Shape (α={alpha})" if success else "Alpha Shape (Failed)"
    else:
        raise ValueError(f"Unknown method: {method}")
    
    return geometry, desc


def visualize_with_reconstruction(
    file_path: str,
    method: str = "voxel",
    voxel_size: float = 0.05,
    point_size: int = 2,
    show_original: bool = True
):
    """
    Load point cloud and visualize with reconstruction.
    
    Args:
        file_path: Path to CSV or PLY file
        method: Reconstruction method ("voxel", "ball_pivoting", "poisson", "alpha_shape")
        voxel_size: Voxel size for voxel grid (meters)
        point_size: Point size for visualization
        show_original: Whether to show original points alongside reconstruction
    """
    import csv
    
    # Load point cloud
    print(f"\n{'='*60}")
    print(f"Loading: {file_path}")
    print(f"Method: {method}")
    print(f"{'='*60}\n")
    
    _, ext = os.path.splitext(file_path)
    if ext.lower() == '.ply':
        pcd = o3d.io.read_point_cloud(file_path)
    elif ext.lower() == '.csv':
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
        raise ValueError(f"Unsupported file type: {ext}")
    
    print(f"Loaded {len(pcd.points)} points\n")
    
    # Reconstruct surface
    geometry, desc = reconstruct_surface(pcd, method=method, voxel_size=voxel_size)
    
    # Prepare visualization
    geometries = []
    
    if method == "voxel":
        geometries.append(geometry)
        if show_original:
            # Show original points in different color
            pcd_vis = o3d.geometry.PointCloud(pcd)
            pcd_vis.paint_uniform_color([0.5, 0.5, 0.5])
            geometries.append(pcd_vis)
    else:
        # For mesh methods
        if len(geometry.triangles) > 0:
            geometry.paint_uniform_color([0.7, 0.7, 0.9])
            geometries.append(geometry)
        
        if show_original:
            pcd_vis = o3d.geometry.PointCloud(pcd)
            pcd_vis.paint_uniform_color([1.0, 0.0, 0.0])
            geometries.append(pcd_vis)
    
    # Add coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    geometries.append(coord_frame)
    
    # Visualize
    print(f"\n{'='*60}")
    print(f"Launching viewer: {desc}")
    print(f"{'='*60}\n")
    
    o3d.visualization.draw_geometries(
        geometries,
        window_name=f"RPLidar Reconstruction - {desc}",
        width=1280,
        height=800,
        point_show_normal=False
    )


if __name__ == "__main__":
    import sys
    import os
    
    if len(sys.argv) < 2:
        print("Usage: python surface_reconstruction.py <file.csv|file.ply> [method] [voxel_size]")
        print("\nMethods:")
        print("  voxel         - Voxel grid (default, best for rooms)")
        print("  ball_pivoting - Ball-pivoting mesh")
        print("  poisson       - Poisson reconstruction")
        print("  alpha_shape   - Alpha shape mesh")
        sys.exit(1)
    
    file_path = sys.argv[1]
    method = sys.argv[2] if len(sys.argv) > 2 else "voxel"
    voxel_size = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    
    visualize_with_reconstruction(file_path, method=method, voxel_size=voxel_size)

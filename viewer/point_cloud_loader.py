"""
Point Cloud Loader for RPLidar scans.

Handles loading and processing of CSV and PLY scan files for visualization.
"""

import os
import csv
import numpy as np
import open3d as o3d
from typing import Optional, Tuple
from . import config


class PointCloudLoader:
    """
    Loads and processes point cloud data from RPLidar scan files.
    
    Supports both CSV and PLY formats.
    """
    
    def __init__(self):
        """Initialize the point cloud loader."""
        self.current_pcd: Optional[o3d.geometry.PointCloud] = None
        self.current_file: Optional[str] = None
        self.scan_type: Optional[str] = None
    
    def load_csv(self, file_path: str) -> Optional[o3d.geometry.PointCloud]:
        """
        Load point cloud from CSV file.
        
        Args:
            file_path: Path to CSV file
            
        Returns:
            Open3D PointCloud object or None if load failed
        """
        if not os.path.exists(file_path):
            print(f"CSV file not found: {file_path}")
            return None
        
        try:
            points = []
            
            with open(file_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x = float(row['x_m'])
                    y = float(row['y_m'])
                    z = float(row.get('z_m', 0.0))  # Default to 0 for 2D scans
                    points.append([x, y, z])
            
            if not points:
                print(f"No points found in CSV file: {file_path}")
                return None
            
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(points))
            
            # Color points based on height (z-value) or default color
            colors = self._generate_colors(np.array(points))
            pcd.colors = o3d.utility.Vector3dVector(colors)
            
            print(f"Loaded {len(points)} points from CSV: {file_path}")
            
            self.current_pcd = pcd
            self.current_file = file_path
            
            return pcd
            
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            return None
    
    def load_ply(self, file_path: str) -> Optional[o3d.geometry.PointCloud]:
        """
        Load point cloud from PLY file.
        
        Args:
            file_path: Path to PLY file
            
        Returns:
            Open3D PointCloud object or None if load failed
        """
        if not os.path.exists(file_path):
            print(f"PLY file not found: {file_path}")
            return None
        
        try:
            pcd = o3d.io.read_point_cloud(file_path)
            
            if pcd.is_empty():
                print(f"PLY file is empty: {file_path}")
                return None
            
            # Add colors if not present
            if not pcd.has_colors():
                points = np.asarray(pcd.points)
                colors = self._generate_colors(points)
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            print(f"Loaded {len(pcd.points)} points from PLY: {file_path}")
            
            self.current_pcd = pcd
            self.current_file = file_path
            
            return pcd
            
        except Exception as e:
            print(f"Error loading PLY file: {e}")
            return None
    
    def load_scan(self, scan_type: str = "2d", file_format: str = "ply") -> Optional[o3d.geometry.PointCloud]:
        """
        Load the appropriate scan file based on type.
        
        Args:
            scan_type: "2d" or "3d"
            file_format: "csv" or "ply"
            
        Returns:
            Open3D PointCloud object or None if load failed
        """
        self.scan_type = scan_type
        
        if scan_type == "2d":
            file_path = config.SCAN_2D_PLY if file_format == "ply" else config.SCAN_2D_CSV
        elif scan_type == "3d":
            file_path = config.SCAN_3D_PLY if file_format == "ply" else config.SCAN_3D_CSV
        else:
            print(f"Unknown scan type: {scan_type}")
            return None
        
        if file_format == "ply":
            return self.load_ply(file_path)
        elif file_format == "csv":
            return self.load_csv(file_path)
        else:
            print(f"Unknown file format: {file_format}")
            return None
    
    def load_file(self, file_path: str) -> Optional[o3d.geometry.PointCloud]:
        """
        Load point cloud from file (auto-detects format from extension).
        
        Args:
            file_path: Path to file
            
        Returns:
            Open3D PointCloud object or None if load failed
        """
        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            return None
        
        ext = os.path.splitext(file_path)[1].lower()
        
        if ext == ".ply":
            return self.load_ply(file_path)
        elif ext == ".csv":
            return self.load_csv(file_path)
        else:
            print(f"Unsupported file format: {ext}")
            return None
    
    def get_current_cloud(self) -> Optional[o3d.geometry.PointCloud]:
        """Get the currently loaded point cloud."""
        return self.current_pcd
    
    def get_point_count(self) -> int:
        """Get the number of points in the current cloud."""
        if self.current_pcd:
            return len(self.current_pcd.points)
        return 0
    
    def get_bounds(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get the bounding box of the current point cloud.
        
        Returns:
            Tuple of (min_bound, max_bound) or None if no cloud loaded
        """
        if self.current_pcd:
            return (self.current_pcd.get_min_bound(), self.current_pcd.get_max_bound())
        return None
    
    def _generate_colors(self, points: np.ndarray) -> np.ndarray:
        """
        Generate colors for points based on height (z-coordinate).
        
        Args:
            points: Nx3 numpy array of points
            
        Returns:
            Nx3 numpy array of RGB colors (0-1 range)
        """
        if len(points) == 0:
            return np.array([])
        
        z_values = points[:, 2]
        
        # If all z-values are the same (2D scan), use default color
        if np.all(z_values == z_values[0]):
            return np.tile(config.POINT_COLOR_DEFAULT, (len(points), 1))
        
        # Color by height using a colormap (blue=low, red=high)
        z_min, z_max = z_values.min(), z_values.max()
        z_range = z_max - z_min
        
        if z_range == 0:
            return np.tile(config.POINT_COLOR_DEFAULT, (len(points), 1))
        
        # Normalize z to 0-1
        z_norm = (z_values - z_min) / z_range
        
        # Create colormap (blue -> cyan -> green -> yellow -> red)
        colors = np.zeros((len(points), 3))
        colors[:, 0] = z_norm  # Red channel
        colors[:, 1] = 1 - np.abs(z_norm - 0.5) * 2  # Green channel (peaks at 0.5)
        colors[:, 2] = 1 - z_norm  # Blue channel
        
        return colors


# Standalone test
if __name__ == "__main__":
    import sys
    
    loader = PointCloudLoader()
    
    # Try loading 2D scan
    print("Loading 2D scan...")
    pcd = loader.load_scan("2d", "ply")
    
    if pcd:
        print(f"Loaded {loader.get_point_count()} points")
        bounds = loader.get_bounds()
        if bounds:
            print(f"Bounds: {bounds[0]} to {bounds[1]}")
        
        # Visualize
        print("Visualizing... close window to exit.")
        o3d.visualization.draw_geometries([pcd], window_name="Test Viewer")
    else:
        print("Failed to load scan. Make sure scan files exist.")

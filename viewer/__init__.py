"""
RPLidar 3D Point Cloud Viewer Package

A comprehensive GUI application for visualizing RPLidar scans.
"""

__version__ = "1.0.0"
__author__ = "RPLidar Team"

from . import config
from .scan_controller import ScanController
from .point_cloud_loader import PointCloudLoader

__all__ = [
    "config",
    "ScanController",
    "PointCloudLoader",
]

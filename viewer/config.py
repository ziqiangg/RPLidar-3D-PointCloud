"""
Configuration settings for the RPLidar Viewer application.
"""

import os


# Scan Types
SCAN_TYPE_2D = "2d"
SCAN_TYPE_3D = "3d"

# File Paths
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, "data")
SCAN_2D_CSV = os.path.join(DATA_DIR, "scan.csv")
SCAN_2D_PLY = os.path.join(DATA_DIR, "scan.ply")
SCAN_3D_CSV = os.path.join(BASE_DIR, "scan_3d.csv")
SCAN_3D_PLY = os.path.join(BASE_DIR, "scan_3d.ply")

# Visualization Settings
POINT_SIZE = 2.0
BACKGROUND_COLOR = [0.1, 0.1, 0.1, 1.0]  # Dark gray (RGBA)
POINT_COLOR_DEFAULT = [0.0, 0.8, 1.0]  # Cyan
WINDOW_NAME = "RPLidar 3D Point Cloud Viewer"
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# Scan Scripts
SCRIPT_2D_SCAN = os.path.join(BASE_DIR, "dump_one_scan.py")
SCRIPT_3D_SCAN = os.path.join(BASE_DIR, "xyzscan_servoless.py")

# Auto-refresh settings
AUTO_REFRESH_INTERVAL = 1.0  # seconds

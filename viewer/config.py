"""
Configuration settings for the RPLidar Viewer application.
"""

import os


# Scan Types
SCAN_TYPE_2D = "2d"
SCAN_TYPE_ROBUST_3D = "robust_3d"
SCAN_TYPE_PANORAMA = "panorama"
SUPPORTED_SCAN_TYPES = (SCAN_TYPE_2D, SCAN_TYPE_ROBUST_3D, SCAN_TYPE_PANORAMA)

# File Paths
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, "data")
PERSISTENT_DIR = os.path.join(DATA_DIR, "persistent")
SCAN_2D_CSV = os.path.join(DATA_DIR, "scan.csv")
SCAN_2D_PLY = os.path.join(DATA_DIR, "scan.ply")
SCAN_3D_CSV = os.path.join(DATA_DIR, "scan_3d.csv")
SCAN_3D_PLY = os.path.join(DATA_DIR, "scan_3d.ply")
PANORAMA_IMAGES_DIR = os.path.join(DATA_DIR, "images")
PANORAMA_INCOMING_DIR = os.path.join(PANORAMA_IMAGES_DIR, "incoming")
PANORAMA_STITCHED_FILE = os.path.join(PANORAMA_IMAGES_DIR, "panorama_stitched.jpg")

# Panorama capture profile (C270 + configured sweep)
PANORAMA_CAPTURE_STEP_DEG = 20
PANORAMA_CAPTURE_START_DEG = 0
PANORAMA_CAPTURE_END_DEG = 160
PANORAMA_EXPECTED_IMAGE_COUNT = 18
PANORAMA_CAMERA_DIAGONAL_FOV_DEG = 55
PANORAMA_CAMERA_WIDTH = 1280
PANORAMA_CAMERA_HEIGHT = 720
# Axis alignment for dual-camera panorama ordering versus LiDAR frame.
# Front camera image 00 points toward -Y by default => -90 deg offset from +X reference.
PANORAMA_FRONT_YAW_OFFSET_DEG = -90.0
PANORAMA_REAR_RELATIVE_YAW_DEG = 180.0

# Panorama-aware point-colorization controls.
# Set AUTO_YAW to False for deterministic trial-and-error tuning via YAW_OFFSET_DEG.
PANORAMA_COLORIZE_AUTO_YAW = False
PANORAMA_COLORIZE_YAW_OFFSET_DEG = 60.0

# Visualization Settings
POINT_SIZE = 2.0
BACKGROUND_COLOR = [0.1, 0.1, 0.1, 1.0]  # Dark gray (RGBA)
POINT_COLOR_DEFAULT = [0.0, 0.8, 1.0]  # Cyan
WINDOW_NAME = "RPLidar 3D Point Cloud Viewer"
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# Scan Scripts
SCRIPT_2D_SCAN = os.path.join(BASE_DIR, "dump_one_scan.py")

# Auto-refresh settings
AUTO_REFRESH_INTERVAL = 1.0  # seconds

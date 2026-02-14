"""
Configuration settings for the RPLidar Viewer application.
"""

import os

# MQTT Configuration
MQTT_BROKER = "localhost"  # Default MQTT broker address
MQTT_PORT = 1883
MQTT_KEEPALIVE = 60

# MQTT Topics
MQTT_TOPIC_SCAN_COMMAND = "rplidar/scan/command"  # Send scan commands to edge device
MQTT_TOPIC_SCAN_STATUS = "rplidar/scan/status"    # Receive scan status from edge device
MQTT_TOPIC_SCAN_RESULT = "rplidar/scan/result"    # Receive scan result file path from edge device
MQTT_TOPIC_SCAN_DATA = "rplidar/scan/data"        # Receive actual scan data (optional)

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

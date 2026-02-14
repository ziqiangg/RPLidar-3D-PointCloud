"""
RPLidar 3D Point Cloud Viewer Application

Main GUI application with Open3D visualization and MQTT edge device control.
"""

import os
import sys
import time
import threading
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from viewer import config
from viewer.mqtt_handler import MQTTHandler
from viewer.point_cloud_loader import PointCloudLoader


class RPLidarViewerApp:
    """
    Main application for RPLidar 3D point cloud visualization.
    
    Features:
    - Open3D-based 3D visualization
    - MQTT edge device control
    - File browser for loading any PLY/CSV files
    - Real-time status updates
    - Auto-load when edge device completes scan
    """
    
    def __init__(self, enable_mqtt: bool = True):
        """
        Initialize the viewer application.
        
        Args:
            enable_mqtt: Enable MQTT connectivity for edge device control
        """
        self.enable_mqtt = enable_mqtt
        
        # Initialize components
        self.loader = PointCloudLoader()
        
        if self.enable_mqtt:
            self.mqtt_handler = MQTTHandler()
            self.mqtt_handler.set_status_callback(self._on_mqtt_status)
            self.mqtt_handler.set_result_callback(self._on_mqtt_result)
        else:
            self.mqtt_handler = None
        
        # GUI components
        self.window = None
        self.scene_widget = None
        self.info_label = None
        self.status_label = None
        self.mqtt_status_label = None
        
        # State
        self.current_scan_type = "2d"
        self.current_file = None
        self.point_size = config.POINT_SIZE
        
    def initialize_gui(self):
        """Initialize the Open3D GUI window and widgets."""
        app = gui.Application.instance
        app.initialize()
        
        self.window = app.create_window(
            config.WINDOW_NAME,
            config.WINDOW_WIDTH,
            config.WINDOW_HEIGHT
        )
        
        # Create layout
        em = self.window.theme.font_size
        
        # Main vertical layout
        main_layout = gui.Vert(0, gui.Margins(em, em, em, em))
        
        # Top control panel
        control_panel = self._create_control_panel(em)
        main_layout.add_child(control_panel)
        
        # 3D scene widget
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.scene_widget.scene.set_background(config.BACKGROUND_COLOR)
        main_layout.add_child(self.scene_widget)
        
        # Bottom info panel
        info_panel = self._create_info_panel(em)
        main_layout.add_child(info_panel)
        
        self.window.add_child(main_layout)
        
        # Set up scene view
        self._setup_scene()
        
        # Window callbacks
        self.window.set_on_layout(self._on_layout)
        self.window.set_on_close(self._on_close)
    
    def _create_control_panel(self, em: float) -> gui.Widget:
        """Create the top control panel with buttons."""
        panel = gui.Horiz(0.5 * em)
        
        # File loading section
        load_file_btn = gui.Button("Load File...")
        load_file_btn.set_on_clicked(self._on_load_file)
        panel.add_child(load_file_btn)
        
        panel.add_fixed(em)
        
        # MQTT control section (only if MQTT enabled)
        if self.mqtt_handler:
            panel.add_child(gui.Label("Edge Device:"))
            
            # Scan type selection
            self.scan_type_combo = gui.Combobox()
            self.scan_type_combo.add_item("2D Scan")
            self.scan_type_combo.add_item("3D Scan")
            self.scan_type_combo.set_on_selection_changed(self._on_scan_type_changed)
            panel.add_child(self.scan_type_combo)
            
            # Trigger scan on edge device
            self.trigger_scan_btn = gui.Button("Trigger Scan")
            self.trigger_scan_btn.set_on_clicked(self._on_trigger_scan)
            panel.add_child(self.trigger_scan_btn)
            
            panel.add_fixed(em)
        
        # View controls
        clear_btn = gui.Button("Clear View")
        clear_btn.set_on_clicked(self._on_clear)
        panel.add_child(clear_btn)
        
        panel.add_stretch()
        
        return panel
    
    def _create_info_panel(self, em: float) -> gui.Widget:
        """Create the bottom information panel."""
        panel = gui.Vert(0.25 * em)
        
        # Info label (point count, bounds, etc.)
        self.info_label = gui.Label("No point cloud loaded")
        panel.add_child(self.info_label)
        
        # Status label (scan status)
        self.status_label = gui.Label("Status: Ready")
        panel.add_child(self.status_label)
        
        # MQTT status label
        if self.mqtt_handler:
            self.mqtt_status_label = gui.Label("MQTT: Connecting...")
            panel.add_child(self.mqtt_status_label)
        
        return panel
    
    def _setup_scene(self):
        """Set up the 3D scene with default view."""
        bounds = o3d.geometry.AxisAlignedBoundingBox([-2, -2, -2], [2, 2, 2])
        self.scene_widget.setup_camera(60, bounds, [0, 0, 0])
        
        # Set default view
        self.scene_widget.look_at([0, 0, 0], [2, 2, 2], [0, 0, 1])
    
    def _on_layout(self, layout_context):
        """Handle window layout changes."""
        r = self.window.content_rect
        self.scene_widget.frame = r
    
    def _on_close(self):
        """Handle window close event."""
        print("Shutting down...")
        if self.mqtt_handler:
            self.mqtt_handler.disconnect()
        return True
    
    def _on_scan_type_changed(self, new_val, new_idx):
        """Handle scan type selection change."""
        self.current_scan_type = "2d" if new_idx == 0 else "3d"
    
    def _on_load_file(self):
        """Handle load file button click - open file dialog."""
        file_dialog = gui.FileDialog(gui.FileDialog.OPEN, "Select Point Cloud File", self.window.theme)
        file_dialog.add_filter(".ply .csv", "Point Cloud files (.ply, .csv)")
        file_dialog.add_filter(".ply", "PLY files (.ply)")
        file_dialog.add_filter(".csv", "CSV files (.csv)")
        file_dialog.add_filter("", "All files")
        
        file_dialog.set_on_cancel(self._on_file_dialog_cancel)
        file_dialog.set_on_done(self._on_file_dialog_done)
        self.window.show_dialog(file_dialog)
    
    def _on_file_dialog_cancel(self):
        """Handle file dialog cancel."""
        self.window.close_dialog()
    
    def _on_file_dialog_done(self, filename):
        """Handle file dialog completion."""
        self.window.close_dialog()
        self.load_and_display_file(filename)
    
    def _on_trigger_scan(self):
        """Handle trigger scan button - send MQTT command to edge device."""
        if self.mqtt_handler and self.mqtt_handler.connected:
            self.mqtt_handler.send_scan_command(self.current_scan_type)
            self._update_status("Edge device: Scan command sent")
        else:
            self._update_status("Error: MQTT not connected")
    
    def _on_clear(self):
        """Handle clear button click."""
        self.scene_widget.scene.clear_geometry()
        self.current_file = None
        self._update_info_label("No point cloud loaded")
    
    def _on_mqtt_status(self, status: str, message: str):
        """
        Callback for MQTT status updates from edge device.
        
        Args:
            status: Status code from edge device
            message: Status message
        """
        def update():
            self.status_label.text = f"Edge Device: {status} - {message}"
        
        gui.Application.instance.post_to_main_thread(self.window, update)
    
    def _on_mqtt_result(self, scan_type: str, file_path: str, success: bool):
        """
        Callback for scan result from edge device.
        
        Args:
            scan_type: Type of scan completed
            file_path: Path to the scan file on edge device (or network path)
            success: Whether scan was successful
        """
        def update():
            if success:
                self._update_status(f"Scan complete: {file_path}")
                # Try to load the file if it's accessible
                if os.path.exists(file_path):
                    self.load_and_display_file(file_path)
                else:
                    self._update_status(f"File not accessible: {file_path} (use Load File to browse)")
            else:
                self._update_status(f"Scan failed on edge device")
        
        gui.Application.instance.post_to_main_thread(self.window, update)
    
    def load_and_display_file(self, file_path: str):
        """
        Load and display a point cloud file.
        
        Args:
            file_path: Path to PLY or CSV file
        """
        if not os.path.exists(file_path):
            self._update_status(f"Error: File not found: {file_path}")
            return
        
        pcd = self.loader.load_file(file_path)
        
        if pcd:
            # Clear existing geometry
            self.scene_widget.scene.clear_geometry()
            
            # Add point cloud to scene
            material = rendering.MaterialRecord()
            material.shader = "defaultUnlit"
            material.point_size = self.point_size
            
            self.scene_widget.scene.add_geometry("points", pcd, material)
            
            # Update camera to fit the point cloud
            bounds = pcd.get_axis_aligned_bounding_box()
            self.scene_widget.setup_camera(60, bounds, bounds.get_center())
            
            # Update info label
            self.current_file = file_path
            count = self.loader.get_point_count()
            bounds_tuple = self.loader.get_bounds()
            
            filename = os.path.basename(file_path)
            if bounds_tuple:
                min_b, max_b = bounds_tuple
                info_text = (f"File: {filename} | Points: {count} | "
                           f"X[{min_b[0]:.2f}, {max_b[0]:.2f}] "
                           f"Y[{min_b[1]:.2f}, {max_b[1]:.2f}] "
                           f"Z[{min_b[2]:.2f}, {max_b[2]:.2f}]")
            else:
                info_text = f"File: {filename} | Points: {count}"
            
            self._update_info_label(info_text)
            self._update_status(f"Loaded: {filename}")
        else:
            self._update_status(f"Failed to load: {file_path}")
    
    def _update_info_label(self, text: str = None):
        """Update the info label with current point cloud info."""
        if text:
            self.info_label.text = text
        else:
            if self.current_file:
                filename = os.path.basename(self.current_file)
                count = self.loader.get_point_count()
                self.info_label.text = f"File: {filename} | Points: {count}"
            else:
                self.info_label.text = "No point cloud loaded"
    
    def _update_status(self, message: str):
        """Update the status label."""
        self.status_label.text = f"Status: {message}"
    
    def run(self, initial_file: str = None):
        """
        Run the application.
        
        Args:
            initial_file: Optional file to load on startup
        """
        # Initialize GUI
        self.initialize_gui()
        
        # Connect MQTT if enabled
        if self.mqtt_handler:
            mqtt_connected = self.mqtt_handler.connect()
            
            def update_mqtt_status():
                if self.mqtt_status_label:
                    status = "Connected" if mqtt_connected else "Disconnected"
                    self.mqtt_status_label.text = f"MQTT: {status} ({config.MQTT_BROKER})"
            
            gui.Application.instance.post_to_main_thread(self.window, update_mqtt_status)
        
        # Load initial file if specified
        if initial_file and os.path.exists(initial_file):
            self.load_and_display_file(initial_file)
        
        # Run the application
        gui.Application.instance.run()


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="RPLidar 3D Point Cloud Viewer")
    parser.add_argument("file", nargs="?", help="Point cloud file to load on startup (PLY or CSV)")
    parser.add_argument("--no-mqtt", action="store_true", help="Disable MQTT connectivity")
    parser.add_argument("--broker", type=str, help="MQTT broker address (default: localhost)")
    parser.add_argument("--port", type=int, help="MQTT broker port (default: 1883)")
    
    args = parser.parse_args()
    
    # Update config if broker/port specified
    if args.broker:
        config.MQTT_BROKER = args.broker
    if args.port:
        config.MQTT_PORT = args.port
    
    # Create and run app
    app = RPLidarViewerApp(enable_mqtt=not args.no_mqtt)
    app.run(initial_file=args.file)


if __name__ == "__main__":
    main()

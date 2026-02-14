"""
RPLidar 3D Point Cloud Viewer Application

Tabbed GUI application with scan control and visualization.
"""

import os
import sys
import subprocess
import threading
import tkinter as tk
from tkinter import filedialog
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from viewer import config
from viewer.point_cloud_loader import PointCloudLoader
from viewer.scan_controller import ScanController


class RPLidarViewerApp:
    """
    Main application for RPLidar scanning and visualization.
    
    Features:
    - Tab 1: Scan Control - Select and run scanning scripts
    - Tab 2: Visualization - Load and view point cloud files
    """
    
    def __init__(self):
        """Initialize the viewer application."""
        print("[DEBUG] Initializing RPLidarViewerApp...")
        # Initialize components
        self.loader = PointCloudLoader()
        self.scan_controller = ScanController()
        print("[DEBUG] Components initialized")
        
        # Set up callbacks
        self.scan_controller.set_status_callback(self._on_scan_status)
        self.scan_controller.set_completion_callback(self._on_scan_complete)
        
        # GUI components
        self.window = None
        self.tabs = None
        self.scene_widget = None
        self.info_label = None
        self.viz_status_label = None
        self.scan_status_label = None
        
        # State
        self.current_file = None
        self.point_size = config.POINT_SIZE
        
    def initialize_gui(self):
        """Initialize the Open3D GUI window and widgets."""
        print("[DEBUG] Initializing GUI...")
        app = gui.Application.instance
        app.initialize()
        print("[DEBUG] GUI app initialized")
        
        self.window = app.create_window(
            "RPLidar Scanner & Viewer",
            config.WINDOW_WIDTH,
            config.WINDOW_HEIGHT
        )
        print("[DEBUG] Window created")
        
        em = self.window.theme.font_size
        
        # Create main layout
        main_layout = gui.Vert(0, gui.Margins(0, 0, 0, 0))
        
        # Create tabbed interface
        self.tabs = gui.TabControl()
        self.tabs.set_on_selected_tab_changed(self._on_tab_changed)
        
        # Tab 1: Scan Control
        print("[DEBUG] Creating scan control tab...")
        scan_tab = self._create_scan_tab(em)
        self.tabs.add_tab("Scan Control", scan_tab)
        print("[DEBUG] Scan control tab added")
        
        # Tab 2: Visualization CONTROLS ONLY (no SceneWidget inside)
        print("[DEBUG] Creating visualization controls...")
        viz_controls = self._create_visualization_controls(em)
        print("[DEBUG] Visualization controls created, adding to tabs...")
        self.tabs.add_tab("Visualization", viz_controls)
        print("[DEBUG] Visualization tab added")
        
        # Add TabControl to main layout
        main_layout.add_child(self.tabs)
        
        # Create SceneWidget DIRECTLY in main layout (not nested in tabs!)
        # This is CRITICAL - SceneWidget mouse breaks when inside layouts (Issue #5268)
        print("[DEBUG] Creating SceneWidget OUTSIDE tab structure...")
        self._create_scene_widget()
        
        # Add SceneWidget directly - it will expand to fill remaining space
        main_layout.add_child(self.scene_widget)
        print("[DEBUG] SceneWidget added directly to main layout")
        
        self.window.add_child(main_layout)
        
        # Window callbacks
        self.window.set_on_close(self._on_close)
    
    def _create_scan_tab(self, em: float) -> gui.Widget:
        """Create the scan control tab."""
        tab = gui.Vert(0, gui.Margins(em, em, em, em))
        
        # Title
        title = gui.Label("RPLidar Scan Control")
        tab.add_child(title)
        tab.add_fixed(em)
        
        # Script selection
        script_panel = gui.CollapsableVert("Scan Script", 0.25 * em, gui.Margins(em, 0, 0, 0))
        
        # Script selection - use VBox with labels instead of RadioButton with children
        script_container = gui.Vert(0.5 * em)
        
        script_2d_checkbox = gui.Checkbox("dump_one_scan.py - Single 2D 360° scan")
        script_2d_checkbox.checked = True
        script_2d_checkbox.set_on_checked(lambda checked: self._on_script_2d_checked(checked))
        script_container.add_child(script_2d_checkbox)
        
        script_3d_checkbox = gui.Checkbox("xyzscan_servoless.py - 3D scan (manual servo control)")
        script_3d_checkbox.checked = False
        script_3d_checkbox.set_on_checked(lambda checked: self._on_script_3d_checked(checked))
        script_container.add_child(script_3d_checkbox)
        
        self.script_2d_checkbox = script_2d_checkbox
        self.script_3d_checkbox = script_3d_checkbox
        
        script_panel.add_child(script_container)
        tab.add_child(script_panel)
        
        tab.add_fixed(em)
        
        # Port configuration
        port_panel = gui.CollapsableVert("Port Configuration", 0.25 * em, gui.Margins(em, 0, 0, 0))
        
        port_horiz = gui.Horiz(0.5 * em)
        port_horiz.add_child(gui.Label("Serial Port:"))
        self.port_input = gui.TextEdit()
        self.port_input.placeholder_text = "Auto-detect (leave empty)"
        port_horiz.add_child(self.port_input)
        
        port_panel.add_child(port_horiz)
        tab.add_child(port_panel)
        
        tab.add_fixed(em)
        
        # Control buttons
        button_panel = gui.Horiz(0.5 * em)
        
        self.start_scan_btn = gui.Button("Start Scan")
        self.start_scan_btn.set_on_clicked(self._on_start_scan)
        button_panel.add_child(self.start_scan_btn)
        
        self.stop_scan_btn = gui.Button("Stop Scan")
        self.stop_scan_btn.set_on_clicked(self._on_stop_scan)
        self.stop_scan_btn.enabled = False
        button_panel.add_child(self.stop_scan_btn)
        
        button_panel.add_stretch()
        tab.add_child(button_panel)
        
        tab.add_fixed(em)
        
        # Status display
        status_panel = gui.CollapsableVert("Scan Status", 0.25 * em, gui.Margins(em, 0, 0, 0))
        status_panel.set_is_open(True)
        
        self.scan_status_label = gui.Label("Status: Ready")
        status_panel.add_child(self.scan_status_label)
        
        tab.add_child(status_panel)
        
        tab.add_stretch()
        
        return tab
    
    def _create_visualization_controls(self, em: float) -> gui.Widget:
        """Create the visualization control panel (NO SceneWidget - that's added directly)."""
        print("[DEBUG] _create_visualization_controls called")
        try:
            # Vertical layout for controls
            controls = gui.Vert(0.5 * em, gui.Margins(em, em, em, em))
            print("[DEBUG] Controls container created")
            
            # Top control bar
            control_bar = gui.Horiz(0.5 * em, gui.Margins(0, 0, 0, 0))
            
            load_btn = gui.Button("Load File...")
            load_btn.set_on_clicked(self._on_load_file)
            control_bar.add_child(load_btn)
            
            clear_btn = gui.Button("Clear")
            clear_btn.set_on_clicked(self._on_clear)
            control_bar.add_child(clear_btn)
            
            control_bar.add_stretch()
            
            # Point size control
            control_bar.add_child(gui.Label("Point Size:"))
            self.point_size_slider = gui.Slider(gui.Slider.INT)
            self.point_size_slider.set_limits(1, 10)
            self.point_size_slider.int_value = int(self.point_size)
            self.point_size_slider.set_on_value_changed(self._on_point_size_changed)
            control_bar.add_child(self.point_size_slider)
            
            controls.add_child(control_bar)
            print("[DEBUG] Control bar added")
            
            # Info panel
            info_panel = gui.Vert(0, gui.Margins(0, em, 0, 0))
            
            self.info_label = gui.Label("No point cloud loaded")
            info_panel.add_child(self.info_label)
            
            self.viz_status_label = gui.Label("Status: Ready")
            info_panel.add_child(self.viz_status_label)
            
            controls.add_child(info_panel)
            controls.add_stretch()
            
            print("[DEBUG] Visualization controls created successfully")
            return controls
        except Exception as e:
            print(f"[DEBUG] ERROR in _create_visualization_controls: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def _create_scene_widget(self):
        """Create SceneWidget directly (NO layout nesting - fixes Issue #5268)."""
        print("[DEBUG] Creating SceneWidget...")
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.scene_widget.scene.set_background(config.BACKGROUND_COLOR)
        
        # Enable axes (no lighting needed for unlit shader)
        self.scene_widget.scene.show_axes(True)
        print("[DEBUG] SceneWidget created and configured")
        
        # Set up default camera view
        bounds = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array([-1, -1, -1], dtype=np.float32), 
            max_bound=np.array([1, 1, 1], dtype=np.float32)
        )
        self.scene_widget.setup_camera(60.0, bounds, np.array([0, 0, 0], dtype=np.float32))
        
        # Set view controls for mouse interaction
        self.scene_widget.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)
        
        # Start hidden (will be shown when Visualization tab is selected)
        self.scene_widget.visible = False
        print("[DEBUG] SceneWidget camera and controls configured")
    
    def _on_close(self):
        """Handle window close event."""
        print("[DEBUG] Window close requested")
        try:
            if self.scan_controller.is_running():
                print("[DEBUG] Stopping running scan...")
                self.scan_controller.stop_scan()
            print("[DEBUG] Cleanup complete")
        except Exception as e:
            print(f"[DEBUG] Error during cleanup: {e}")
        return True
    
    def _on_tab_changed(self, idx):
        """Handle tab change event."""
        print(f"[DEBUG] Tab changed to index: {idx}")
        tab_names = ["Scan Control", "Visualization"]
        if idx < len(tab_names):
            print(f"[DEBUG] Now viewing: {tab_names[idx]} tab")
            
            # Show SceneWidget only when Visualization tab is selected (index 1)
            if hasattr(self, 'scene_widget'):
                if idx == 1:  # Visualization tab
                    self.scene_widget.visible = True
                    print("[DEBUG] SceneWidget visible")
                else:
                    self.scene_widget.visible = False
                    print("[DEBUG] SceneWidget hidden")
    
    def _on_script_2d_checked(self, checked):
        """Handle 2D script checkbox."""
        print(f"[DEBUG] 2D script checkbox: {checked}")
        if checked:
            self.script_3d_checkbox.checked = False
    
    def _on_script_3d_checked(self, checked):
        """Handle 3D script checkbox."""
        print(f"[DEBUG] 3D script checkbox: {checked}")
        if checked:
            self.script_2d_checkbox.checked = False
    
    def _on_start_scan(self):
        """Handle start scan button click."""
        print("[DEBUG] Start scan button clicked")
        if self.scan_controller.is_running():
            print("[DEBUG] Scan already running, ignoring")
            return
        
        # Get selected script based on checkboxes
        scan_type = "2d" if self.script_2d_checkbox.checked else "3d"
        print(f"[DEBUG] Selected scan type: {scan_type}")
        
        # Get port if specified
        port = self.port_input.text_value.strip()
        params = {"port": port} if port else {}
        print(f"[DEBUG] Port: '{port}', Params: {params}")
        
        # Disable start button, enable stop button
        print("[DEBUG] Disabling start button, enabling stop button")
        self.start_scan_btn.enabled = False
        self.stop_scan_btn.enabled = True
        
        # Start scan
        print(f"[DEBUG] Starting scan with type={scan_type}, params={params}")
        self.scan_controller.start_scan(scan_type, params)
        print("[DEBUG] Scan started")
    
    def _on_stop_scan(self):
        """Handle stop scan button click."""
        print("[DEBUG] Stop scan button clicked")
        self.scan_controller.stop_scan()
        self.start_scan_btn.enabled = True
        self.stop_scan_btn.enabled = False
    
    def _on_scan_status(self, status: str, message: str):
        """Callback for scan status updates."""
        print(f"[DEBUG] Scan status callback: status={status}, message={message}")
        def update():
            print(f"[DEBUG] Updating scan status label...")
            if self.scan_status_label:
                self.scan_status_label.text = f"Status: {status} - {message}"
            
            if status in ["completed", "error", "stopped"]:
                print(f"[DEBUG] Scan finished, re-enabling buttons")
                self.start_scan_btn.enabled = True
                self.stop_scan_btn.enabled = False
        
        gui.Application.instance.post_to_main_thread(self.window, update)
    
    def _on_scan_complete(self, scan_type: str, success: bool, file_path: str):
        """Callback for scan completion."""
        print(f"[DEBUG] Scan complete callback: type={scan_type}, success={success}, file={file_path}")
        if success and os.path.exists(file_path):
            # Just notify - don't auto-load to avoid threading issues
            # User can manually switch to viz tab and load
            print(f"[INFO] Scan saved to: {file_path}")
            print(f"[INFO] Switch to Visualization tab and click 'Load File...' to view results")
    
    def _on_load_file(self):
        """Handle load file button click."""
        print("[DEBUG] Load file button clicked")
        
        try:
            # CRITICAL FIX: Run tkinter dialog in separate thread to avoid crash (Issue #4427)
            # https://github.com/isl-org/Open3D/issues/4427
            selected_file = [None]  # Use list to store result from thread
            
            def ask_file():
                """Run file dialog in separate thread."""
                root = tk.Tk()
                root.withdraw()  # Hide the root window
                root.attributes('-topmost', True)  # Bring dialog to front
                
                # Set initial directory
                initial_dir = config.DATA_DIR if os.path.exists(config.DATA_DIR) else os.getcwd()
                
                print("[DEBUG] Opening file dialog...")
                filename = filedialog.askopenfilename(
                    parent=root,
                    title="Select Point Cloud File",
                    initialdir=initial_dir,
                    filetypes=[
                        ("Point Cloud files", "*.ply *.csv"),
                        ("PLY files", "*.ply"),
                        ("CSV files", "*.csv"),
                        ("All files", "*.*")
                    ]
                )
                
                root.destroy()  # Clean up Tk window
                selected_file[0] = filename  # Store result
            
            # Run dialog in thread and wait for completion
            dialog_thread = threading.Thread(target=ask_file)
            dialog_thread.start()
            dialog_thread.join()  # Wait for thread to complete
            
            filename = selected_file[0]
            
            if filename:
                print(f"[DEBUG] File selected: {filename}")
                self.load_and_display_file(filename)
            else:
                print("[DEBUG] File dialog cancelled")
                
        except Exception as e:
            print(f"[DEBUG] File dialog error: {e}")
            import traceback
            traceback.print_exc()
    
    def _on_clear(self):
        """Handle clear button click."""
        print("[DEBUG] Clear button clicked")
        
        try:
            self.scene_widget.scene.clear_geometry()
            self.current_file = None
            self._update_info_label("No point cloud loaded")
            self._update_viz_status("Cleared")
            
            # Use force_redraw for SceneWidget
            self.scene_widget.force_redraw()
            print("[DEBUG] Clear complete")
            
        except Exception as e:
            print(f"[DEBUG] Clear error: {e}")
            import traceback
            traceback.print_exc()
    
    def _on_point_size_changed(self, value):
        """Handle point size slider change."""
        print(f"[DEBUG] Point size changed to: {value}")
        self.point_size = float(value)
        
        # Update existing geometry's point size without reloading
        if self.scene_widget.scene.has_geometry("points"):
            # Remove and re-add with new material
            if self.current_file and os.path.exists(self.current_file):
                print("[DEBUG] Reloading with new point size...")
                pcd = self.loader.load_file(self.current_file)
                if pcd:
                    self.scene_widget.scene.remove_geometry("points")
                    
                    material = rendering.MaterialRecord()
                    material.shader = "defaultUnlit"  # Match the shader used in load_and_display_file
                    material.point_size = self.point_size
                    
                    self.scene_widget.scene.add_geometry("points", pcd, material, add_downsampled_copy_for_fast_rendering=False)
                    self.scene_widget.force_redraw()
                    print("[DEBUG] Point size updated")
    
    def load_and_display_file(self, file_path: str):
        """Load and display a point cloud file."""
        print(f"[DEBUG] load_and_display_file called with: {file_path}")
        if not os.path.exists(file_path):
            print(f"[DEBUG] File not found: {file_path}")
            self._update_viz_status(f"Error: File not found: {file_path}")
            return
        
        print("[DEBUG] Loading file...")
        pcd = self.loader.load_file(file_path)
        print(f"[DEBUG] File loaded, pcd is None: {pcd is None}")
        
        if pcd:
            print("[DEBUG] Clearing existing geometry...")
            # Clear existing geometry
            self.scene_widget.scene.clear_geometry()
            print("[DEBUG] Geometry cleared")
            
            # Add point cloud to scene with MaterialRecord (REQUIRED!)
            print("[DEBUG] Creating material...")
            material = rendering.MaterialRecord()
            material.shader = "defaultUnlit"  # Unlit shader doesn't require lighting setup
            material.point_size = self.point_size
            print(f"[DEBUG] Material created with point_size={self.point_size}")
            
            print("[DEBUG] Adding geometry to scene...")
            # Verify geometry has points
            print(f"[DEBUG] Geometry has {len(pcd.points)} points")
            print(f"[DEBUG] Points has colors: {pcd.has_colors()}")
            
            # Validate geometry before adding
            if len(pcd.points) == 0:
                print("[DEBUG] ERROR: Point cloud is empty!")
                self._update_viz_status("Error: Empty point cloud")
                return
            
            # Ensure geometry is valid
            if pcd.has_colors():
                colors = np.asarray(pcd.colors)
                if np.any(np.isnan(colors)) or np.any(np.isinf(colors)):
                    print("[DEBUG] WARNING: Colors contain NaN or Inf, regenerating...")
                    pcd.colors = o3d.utility.Vector3dVector(np.tile([1.0, 0.0, 0.0], (len(pcd.points), 1)))
            
            points = np.asarray(pcd.points)
            if np.any(np.isnan(points)) or np.any(np.isinf(points)):
                print("[DEBUG] ERROR: Points contain NaN or Inf!")
                self._update_viz_status("Error: Invalid geometry data")
                return
            
            # CRITICAL: Disable downsampled copy which causes rendering bugs (Issue #6464)
            self.scene_widget.scene.add_geometry("points", pcd, material, add_downsampled_copy_for_fast_rendering=False)
            print("[DEBUG] Geometry added (downsampling disabled)")
            
            # Setup camera properly
            print("[DEBUG] Setting up camera...")
            try:
                bounds = pcd.get_axis_aligned_bounding_box()
                center = bounds.get_center()
                extent = bounds.get_extent()
                print(f"[DEBUG] Bounds center: {center}, extent: {extent}")
                
                # Calculate appropriate camera distance
                max_extent = max(extent[0], extent[1], 0.1)  # Use at least 0.1 for flat scans
                cam_distance = max_extent * 2.0
                print(f"[DEBUG] Camera distance: {cam_distance}")
                
                # For 2D scans (flat in XY), adjust the viewing angle
                is_2d_scan = extent[2] < 0.01  # Z extent is nearly zero
                print(f"[DEBUG] Is 2D scan: {is_2d_scan}")
                
                # Expand bounds for 2D scans to avoid clipping issues
                if is_2d_scan:
                    # Add some thickness to the bounding box for camera setup
                    bounds = o3d.geometry.AxisAlignedBoundingBox(
                        min_bound=np.array([bounds.min_bound[0], bounds.min_bound[1], -0.5], dtype=np.float32),
                        max_bound=np.array([bounds.max_bound[0], bounds.max_bound[1], 0.5], dtype=np.float32)
                    )
                    print("[DEBUG] Expanded bounds for 2D scan")
                
                # Setup camera with proper bounds
                print("[DEBUG] Calling setup_camera...")
                center = center.astype(np.float32)
                
                # Validate camera parameters
                if np.any(np.isnan(center)) or np.any(np.isinf(center)):
                    print("[DEBUG] ERROR: Camera center contains NaN or Inf!")
                    center = np.array([0, 0, 0], dtype=np.float32)
                
                self.scene_widget.setup_camera(60.0, bounds, center)
                print("[DEBUG] setup_camera complete")
                
                # DON'T call look_at - it may be causing the crash
                # setup_camera should be sufficient to view the geometry
                print("[DEBUG] Camera setup complete (skipping look_at)")
                
            except Exception as e:
                print(f"Camera setup warning: {e}")
            
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
            self._update_viz_status(f"Loaded: {filename}")
            
            print("[DEBUG] Calling force_redraw() to update scene...")
            # CRITICAL: Use force_redraw() instead of window.post_redraw()
            # This ensures the SceneWidget actually renders the geometry
            self.scene_widget.force_redraw()
            print("[DEBUG] Load complete - scene force refreshed")
        else:
            self._update_viz_status(f"Failed to load: {file_path}")
            self.window.post_redraw()
    
    def _update_info_label(self, text: str):
        """Update the info label."""
        print(f"[DEBUG] Updating info label: {text}")
        if self.info_label:
            self.info_label.text = text
        else:
            print("[DEBUG] Warning: info_label is None")
    
    def _update_viz_status(self, message: str):
        """Update the visualization status label."""
        print(f"[DEBUG] Updating viz status: {message}")
        if self.viz_status_label:
            self.viz_status_label.text = f"Status: {message}"
        else:
            print("[DEBUG] Warning: viz_status_label is None")
    
    def run(self, initial_file: str = None):
        """Run the application."""
        print("[DEBUG] Starting application...")
        # Initialize GUI
        self.initialize_gui()
        print("[DEBUG] GUI initialized")
        
        # Don't auto-load files - let user manually load via the Load button
        if initial_file:
            print(f"[INFO] To view {initial_file}, switch to Visualization tab and click 'Load File...'")
        
        # Run the application
        print("[DEBUG] Starting GUI main loop...")
        try:
            gui.Application.instance.run()
            print("[DEBUG] GUI main loop ended normally")
        except Exception as e:
            print(f"[ERROR] GUI main loop error: {e}")
            import traceback
            traceback.print_exc()
            raise


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="RPLidar Scanner & Viewer")
    parser.add_argument("file", nargs="?", help="Point cloud file to load on startup")
    
    args = parser.parse_args()
    
    # Create and run app
    app = RPLidarViewerApp()
    app.run(initial_file=args.file)


if __name__ == "__main__":
    main()

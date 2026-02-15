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
        self.info_label = None
        self.viz_status_label = None
        self.scan_status_label = None
        
        # State
        self.current_file = None
        self.current_pcd = None  # Store loaded point cloud
        self.point_size = config.POINT_SIZE
        self.is_3d_scanning = False  # Track if 3D scan is actively scanning
        self.is_first_3d_angle = False  # Track if this is the first angle input for 3D scan
        
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
        
        # Tab 1: Scan Control
        print("[DEBUG] Creating scan control tab...")
        scan_tab = self._create_scan_tab(em)
        self.tabs.add_tab("Scan Control", scan_tab)
        print("[DEBUG] Scan control tab added")
        
        # Tab 2: Visualization Controls
        print("[DEBUG] Creating visualization controls...")
        viz_controls = self._create_visualization_controls(em)
        print("[DEBUG] Visualization controls created, adding to tabs...")
        self.tabs.add_tab("Visualization", viz_controls)
        print("[DEBUG] Visualization tab added")
        
        # Add TabControl to main layout
        main_layout.add_child(self.tabs)
        
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
        
        # 3D Scan Interactive Control (always visible for proper layout)
        self.scan_3d_panel = gui.CollapsableVert("3D Scan Control (Interactive)", 0.25 * em, gui.Margins(em, 0, 0, 0))
        self.scan_3d_panel.set_is_open(True)  # Always open to prevent layout overlap
        
        info_label = gui.Label("For 3D scans, enter servo angles (0-359) or 'q' to quit:")
        self.scan_3d_panel.add_child(info_label)
        
        self.scan_3d_panel.add_fixed(em * 0.5)
        
        # Input field and send button
        input_horiz = gui.Horiz(0.5 * em)
        input_horiz.add_child(gui.Label("Angle/Cmd:"))
        
        self.angle_input = gui.TextEdit()
        self.angle_input.placeholder_text = "e.g., 0, 30, 90, or q"
        input_horiz.add_child(self.angle_input)
        
        self.send_angle_btn = gui.Button("Send")
        self.send_angle_btn.set_on_clicked(self._on_send_angle)
        self.send_angle_btn.enabled = False  # Disabled until 3D scan starts
        input_horiz.add_child(self.send_angle_btn)
        
        self.scan_3d_panel.add_child(input_horiz)
        
        # Status label for 3D scan guidance
        self.scan_3d_status_label = gui.Label("Click 'Start Scan' to begin")
        self.scan_3d_panel.add_child(self.scan_3d_status_label)
        
        tab.add_child(self.scan_3d_panel)
        
        tab.add_fixed(em)
        
        # Control buttons
        tab.add_stretch()
        
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
        
        # Don't add stretch here - it's already added after 3D panel
        
        return tab
    
    def _create_visualization_controls(self, em: float) -> gui.Widget:
        """Create the visualization control panel."""
        print("[DEBUG] _create_visualization_controls called")
        try:
            # Vertical layout for controls
            controls = gui.Vert(0.5 * em, gui.Margins(em, em, em, em))
            print("[DEBUG] Controls container created")
            
            # Title
            title = gui.Label("Point Cloud Visualization")
            controls.add_child(title)
            controls.add_fixed(em)
            
            # Control panel
            control_panel = gui.CollapsableVert("File Controls", 0.25 * em, gui.Margins(em, 0, 0, 0))
            control_panel.set_is_open(True)
            
            # Top control bar
            control_bar = gui.Horiz(0.5 * em, gui.Margins(0, 0, 0, 0))
            
            load_btn = gui.Button("Load File...")
            load_btn.set_on_clicked(self._on_load_file)
            control_bar.add_child(load_btn)
            
            clear_btn = gui.Button("Clear")
            clear_btn.set_on_clicked(self._on_clear)
            control_bar.add_child(clear_btn)
            
            control_bar.add_stretch()
            
            control_panel.add_child(control_bar)
            controls.add_child(control_panel)
            controls.add_fixed(em)
            
            # Visualization panel
            viz_panel = gui.CollapsableVert("Visualization", 0.25 * em, gui.Margins(em, 0, 0, 0))
            viz_panel.set_is_open(True)
            
            # Point size control
            size_horiz = gui.Horiz(0.5 * em)
            size_horiz.add_child(gui.Label("Point Size:"))
            self.point_size_slider = gui.Slider(gui.Slider.INT)
            self.point_size_slider.set_limits(1, 10)
            self.point_size_slider.int_value = int(self.point_size)
            self.point_size_slider.set_on_value_changed(self._on_point_size_changed)
            size_horiz.add_child(self.point_size_slider)
            viz_panel.add_child(size_horiz)
            
            viz_panel.add_fixed(em * 0.5)
            
            # Visualize button
            self.visualize_btn = gui.Button("Visualize in 3D Window")
            self.visualize_btn.set_on_clicked(self._on_visualize)
            self.visualize_btn.enabled = False  # Disabled until file loaded
            viz_panel.add_child(self.visualize_btn)
            
            controls.add_child(viz_panel)
            controls.add_fixed(em)
            
            # Info panel
            info_panel = gui.CollapsableVert("Information", 0.25 * em, gui.Margins(em, 0, 0, 0))
            info_panel.set_is_open(True)
            
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
        
        # For 3D scans, enable the interactive control
        if scan_type == "3d":
            self.send_angle_btn.enabled = True
            self.scan_3d_status_label.text = "Insert an integer between 0-359 or 'q' to finish."
            self.is_3d_scanning = False
            self.is_first_3d_angle = True  # Mark that we're waiting for the first angle
        
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
        self.send_angle_btn.enabled = False  # Disable interactive control
        self.is_3d_scanning = False
        self.is_first_3d_angle = False
        self.scan_3d_status_label.text = "Scan stopped"
    
    def _on_send_angle(self):
        """Handle send angle button click for 3D scans."""
        angle_text = self.angle_input.text_value.strip()
        print(f"[DEBUG] Sending angle/command: {angle_text}")
        
        if not angle_text:
            self.scan_3d_status_label.text = "Please enter an angle (0-359) or 'q'"
            return
        
        # Handle 'q' to quit
        if angle_text.lower() == 'q':
            # If it's the first angle, we need to skip the initial prompt first
            if self.is_first_3d_angle:
                self.scan_controller.send_input('n')  # Skip initial scan prompt
                import time
                time.sleep(0.1)
            self.scan_controller.send_input('q')
            self.scan_3d_status_label.text = "Finishing scan..."
            self.angle_input.text_value = ""
            self.is_3d_scanning = False
            self.is_first_3d_angle = False
            return
        
        # Validate angle
        try:
            angle_val = int(angle_text)
            if not (0 <= angle_val <= 359):
                self.scan_3d_status_label.text = "Error: Angle must be 0-359"
                return
        except ValueError:
            self.scan_3d_status_label.text = f"Error: '{angle_text}' is not a valid angle"
            return
        
        # Handle first angle input (script is at "Start first scan at 0 deg?" prompt)
        if self.is_first_3d_angle:
            self.is_first_3d_angle = False
            
            if angle_val == 0:
                # User wants to scan at 0 - just confirm
                print(f"[DEBUG] First scan at 0 deg - sending 'y' to confirm")
                self.scan_controller.send_input('y')
                self.is_3d_scanning = True
                self.scan_3d_status_label.text = "Scan in progress..."
            else:
                # User wants to scan at different angle - skip 0, set angle, confirm
                print(f"[DEBUG] First scan at {angle_val} deg - skipping 0, setting angle, confirming")
                
                import threading
                def first_angle_setup():
                    import time
                    # Skip the "Start first scan at 0?" prompt
                    self.scan_controller.send_input('n')
                    time.sleep(0.2)
                    # Send the desired angle
                    self.scan_controller.send_input(angle_text)
                    time.sleep(0.2)
                    # Confirm scanning at that angle
                    self.scan_controller.send_input('y')
                    self.is_3d_scanning = True
                    def update_status():
                        self.scan_3d_status_label.text = "Scan in progress..."
                    gui.Application.instance.post_to_main_thread(self.window, update_status)
                
                threading.Thread(target=first_angle_setup, daemon=True).start()
        else:
            # Subsequent angles: send angle, then auto-confirm
            print(f"[DEBUG] Sending angle {angle_val}")
            self.scan_controller.send_input(angle_text)  # Send angle to "Next angle" prompt
            
            # Auto-confirm readiness for next scan after a brief delay
            import threading
            def auto_confirm():
                import time
                time.sleep(0.5)  # Wait for script to prompt for readiness
                print("[DEBUG] Auto-sending 'y' to confirm scan")
                self.scan_controller.send_input('y')
                self.is_3d_scanning = True
                def update_status():
                    self.scan_3d_status_label.text = "Scan in progress..."
                gui.Application.instance.post_to_main_thread(self.window, update_status)
            
            threading.Thread(target=auto_confirm, daemon=True).start()
        
        # Clear input field
        self.angle_input.text_value = ""
    
    def _on_scan_status(self, status: str, message: str):
        """Callback for scan status updates."""
        print(f"[DEBUG] Scan status callback: status={status}, message={message}")
        def update():
            print(f"[DEBUG] Updating scan status label...")
            if self.scan_status_label:
                self.scan_status_label.text = f"Status: {status} - {message}"
            
            # For 3D scans, detect scan completion and prompt for next angle
            if self.is_3d_scanning and "SCAN COMPLETE" in message:
                self.is_3d_scanning = False
                self.scan_3d_status_label.text = "Scan done. Insert an integer between 0-359 or 'q' to finish."
            
            if status in ["completed", "error", "stopped"]:
                print(f"[DEBUG] Scan finished, re-enabling buttons")
                self.start_scan_btn.enabled = True
                self.stop_scan_btn.enabled = False
                self.send_angle_btn.enabled = False  # Disable interactive control
                self.is_3d_scanning = False
                self.is_first_3d_angle = False
                if self.scan_3d_status_label:
                    self.scan_3d_status_label.text = "Scan complete."
        
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
        
        def clear_operation():
            try:
                self.current_file = None
                self.current_pcd = None
                self._update_info_label("No point cloud loaded")
                self._update_viz_status("Cleared")
                self.visualize_btn.enabled = False
                print("[DEBUG] Clear complete")
                
            except Exception as e:
                print(f"[DEBUG] Clear error: {e}")
                import traceback
                traceback.print_exc()
        
        # Post to main thread to ensure thread-safe GUI updates
        gui.Application.instance.post_to_main_thread(self.window, clear_operation)
    
    def _on_point_size_changed(self, value):
        """Handle point size slider change."""
        print(f"[DEBUG] Point size changed to: {value}")
        self.point_size = float(value)
        # Point size will be applied when visualize button is clicked
    
    def load_and_display_file(self, file_path: str):
        """Load a point cloud file."""
        print(f"[DEBUG] load_and_display_file called with: {file_path}")
        if not os.path.exists(file_path):
            print(f"[DEBUG] File not found: {file_path}")
            self._update_viz_status(f"Error: File not found: {file_path}")
            return
        
        print("[DEBUG] Loading file...")
        pcd = self.loader.load_file(file_path)
        print(f"[DEBUG] File loaded, pcd is None: {pcd is None}")
        
        if pcd:
            # Validate geometry
            if len(pcd.points) == 0:
                print("[DEBUG] ERROR: Point cloud is empty!")
                self._update_viz_status("Error: Empty point cloud")
                return
            
            # Add bright colors if missing
            if not pcd.has_colors():
                print("[DEBUG] Adding default red colors...")
                colors = np.tile([1.0, 0.0, 0.0], (len(pcd.points), 1))
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # Store the loaded point cloud
            self.current_pcd = pcd
            self.current_file = file_path
            
            # Update info label
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
            
            # Enable visualize button
            self.visualize_btn.enabled = True
            
            print("[DEBUG] Load complete - click 'Visualize' to view")
        else:
            self._update_viz_status(f"Failed to load: {file_path}")
    
    def _on_visualize(self):
        """Open the point cloud in a classic viewer window via subprocess."""
        print("[DEBUG] Visualize button clicked")
        
        if not self.current_file or not os.path.exists(self.current_file):
            print("[DEBUG] No valid file loaded")
            self._update_viz_status("Error: No point cloud loaded")
            return
        
        try:
            # Launch standalone viewer as subprocess (prevents GLFW conflicts)
            viewer_script = os.path.join(os.path.dirname(__file__), 'standalone_viewer.py')
            python_exe = sys.executable
            
            # Build command
            cmd = [python_exe, viewer_script, self.current_file, str(int(self.point_size))]
            
            print(f"[DEBUG] Launching viewer subprocess: {' '.join(cmd)}")
            
            # Use Popen to launch non-blocking subprocess
            subprocess.Popen(
                cmd,
                creationflags=subprocess.CREATE_NEW_CONSOLE if sys.platform == 'win32' else 0
            )
            
            self._update_viz_status("Viewer window opened")
            print("[DEBUG] Viewer subprocess launched")
            
        except Exception as e:
            print(f"[DEBUG] Viewer error: {e}")
            import traceback
            traceback.print_exc()
            self._update_viz_status(f"Error: {str(e)}")
    
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

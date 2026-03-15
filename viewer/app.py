"""
RPLidar 3D Point Cloud Viewer Application

Tabbed GUI application with scan control and visualization.
"""

import os
import sys
import glob
import shutil
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
from viewer.panorama_stitcher import (
    find_panorama_images,
    stitch_equirectangular_panorama,
    show_panorama_window,
)


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
        self.scan_controller.set_data_callback(self._on_scan_data)
        
        # GUI components
        self.window = None
        self.tabs = None
        self.info_label = None
        self.viz_status_label = None
        self.scan_status_label = None
        self.step_scan_btn = None
        self.panorama_status_label = None
        self.panorama_info_label = None
        self.panorama_last_output = config.PANORAMA_STITCHED_FILE
        self.panorama_last_sources = []
        self.panorama_frames_by_scan = {}
        self.panorama_stitched_scans = set()
        
        # State
        self.current_file = None
        self.current_pcd = None  # Store loaded point cloud
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

        print("[DEBUG] Creating panorama tab...")
        panorama_tab = self._create_panorama_tab(em)
        self.tabs.add_tab("Panorama", panorama_tab)
        print("[DEBUG] Panorama tab added")
        
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
        
        # Scan type selection
        scan_type_panel = gui.CollapsableVert("Scan Type", 0.25 * em, gui.Margins(em, 0, 0, 0))
        scan_type_container = gui.Vert(0.5 * em)

        self.scan_type_2d_checkbox = gui.Checkbox("2D Scan (single 360°)")
        self.scan_type_2d_checkbox.checked = True
        self.scan_type_2d_checkbox.set_on_checked(self._on_scan_type_2d_checked)
        scan_type_container.add_child(self.scan_type_2d_checkbox)

        self.scan_type_robust_checkbox = gui.Checkbox("3D Scan (Robust, final aggregated files)")
        self.scan_type_robust_checkbox.checked = False
        self.scan_type_robust_checkbox.set_on_checked(self._on_scan_type_robust_checked)
        scan_type_container.add_child(self.scan_type_robust_checkbox)

        self.scan_type_panorama_checkbox = gui.Checkbox("Panorama (servo sweep)")
        self.scan_type_panorama_checkbox.checked = False
        self.scan_type_panorama_checkbox.set_on_checked(self._on_scan_type_panorama_checked)
        scan_type_container.add_child(self.scan_type_panorama_checkbox)

        scan_type_panel.add_child(scan_type_container)
        tab.add_child(scan_type_panel)
        
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
        tab.add_stretch()
        
        button_panel = gui.Horiz(0.5 * em)
        
        self.start_scan_btn = gui.Button("Start Scan")
        self.start_scan_btn.set_on_clicked(self._on_start_scan)
        button_panel.add_child(self.start_scan_btn)
        
        self.stop_scan_btn = gui.Button("Stop Scan")
        self.stop_scan_btn.set_on_clicked(self._on_stop_scan)
        self.stop_scan_btn.enabled = False
        button_panel.add_child(self.stop_scan_btn)

        # Kept as None for backward compatibility
        self.step_scan_btn = None
        
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
            
            self.save_ply_btn = gui.Button("Save PLY...")
            self.save_ply_btn.set_on_clicked(self._on_save_ply)
            self.save_ply_btn.enabled = False  # Disabled until point cloud loaded
            control_bar.add_child(self.save_ply_btn)
            
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

    def _create_panorama_tab(self, em: float) -> gui.Widget:
        """Create panorama stitching controls and OpenCV viewing actions."""
        tab = gui.Vert(0.5 * em, gui.Margins(em, em, em, em))

        title = gui.Label("Panorama Stitching (OpenCV)")
        tab.add_child(title)
        tab.add_fixed(em)

        profile_panel = gui.CollapsableVert("Capture Profile", 0.25 * em, gui.Margins(em, 0, 0, 0))
        profile_panel.set_is_open(True)
        profile_text = (
            f"Camera: Logitech C270 | Diagonal FOV: {config.PANORAMA_CAMERA_DIAGONAL_FOV_DEG} deg | "
            f"Resolution: {config.PANORAMA_CAMERA_WIDTH}x{config.PANORAMA_CAMERA_HEIGHT}\n"
            f"Sweep: {config.PANORAMA_CAPTURE_START_DEG}-{config.PANORAMA_CAPTURE_END_DEG} deg in "
            f"{config.PANORAMA_CAPTURE_STEP_DEG} deg steps "
            f"(dual-camera set: {config.PANORAMA_EXPECTED_IMAGE_COUNT} images expected)"
        )
        self.panorama_info_label = gui.Label(profile_text)
        profile_panel.add_child(self.panorama_info_label)
        tab.add_child(profile_panel)
        tab.add_fixed(em)

        actions_panel = gui.CollapsableVert("Panorama Actions", 0.25 * em, gui.Margins(em, 0, 0, 0))
        actions_panel.set_is_open(True)

        actions_row = gui.Horiz(0.5 * em)
        show_btn = gui.Button("Show Stitched Panorama")
        show_btn.set_on_clicked(self._on_panorama_show)
        actions_row.add_child(show_btn)

        save_btn = gui.Button("Save Panorama...")
        save_btn.set_on_clicked(self._on_panorama_save)
        actions_row.add_child(save_btn)

        save_images_btn = gui.Button("Save Images...")
        save_images_btn.set_on_clicked(self._on_panorama_save_images)
        actions_row.add_child(save_images_btn)

        actions_row.add_stretch()
        actions_panel.add_child(actions_row)
        tab.add_child(actions_panel)
        tab.add_fixed(em)

        status_panel = gui.CollapsableVert("Status", 0.25 * em, gui.Margins(em, 0, 0, 0))
        status_panel.set_is_open(True)
        self.panorama_status_label = gui.Label("Status: Waiting for panorama images")
        status_panel.add_child(self.panorama_status_label)
        tab.add_child(status_panel)
        tab.add_stretch()

        # Support existing captures: if six images already exist locally, stitch automatically.
        worker = threading.Thread(target=self._auto_stitch_existing_images_if_ready, daemon=True)
        worker.start()

        return tab
    
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
    
    def _on_start_scan(self):
        """Handle start scan button click."""
        print("[DEBUG] Start scan button clicked")
        if self.scan_controller.is_running():
            print("[DEBUG] Scan already running, ignoring")
            return
        
        # Determine selected scan type
        if hasattr(self, 'scan_type_panorama_checkbox') and self.scan_type_panorama_checkbox.checked:
            scan_type = config.SCAN_TYPE_PANORAMA
        elif hasattr(self, 'scan_type_robust_checkbox') and self.scan_type_robust_checkbox.checked:
            scan_type = config.SCAN_TYPE_ROBUST_3D
        else:
            scan_type = config.SCAN_TYPE_2D
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
        started = self.scan_controller.start_scan(scan_type, params)
        if started:
            print("[DEBUG] Scan started")
        else:
            print("[DEBUG] Scan failed to start, restoring button state")
            self.start_scan_btn.enabled = True
            self.stop_scan_btn.enabled = False

    def _on_scan_type_2d_checked(self, checked: bool):
        """Keep scan type selection mutually exclusive."""
        if checked:
            self.scan_type_robust_checkbox.checked = False
            self.scan_type_panorama_checkbox.checked = False
        elif not self.scan_type_robust_checkbox.checked and not self.scan_type_panorama_checkbox.checked:
            self.scan_type_2d_checkbox.checked = True

    def _on_scan_type_robust_checked(self, checked: bool):
        """Keep scan type selection mutually exclusive."""
        if checked:
            self.scan_type_2d_checkbox.checked = False
            self.scan_type_panorama_checkbox.checked = False
        elif not self.scan_type_2d_checkbox.checked and not self.scan_type_panorama_checkbox.checked:
            self.scan_type_2d_checkbox.checked = True

    def _on_scan_type_panorama_checked(self, checked: bool):
        """Keep scan type selection mutually exclusive."""
        if checked:
            self.scan_type_2d_checkbox.checked = False
            self.scan_type_robust_checkbox.checked = False
        elif not self.scan_type_2d_checkbox.checked and not self.scan_type_robust_checkbox.checked:
            self.scan_type_2d_checkbox.checked = True
    
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

            # No longer need to enable step button - automatic mode only
            # waiting_for_step = status == "started" and "Awaiting step permission" in message
            # if self.step_scan_btn:
            #     self.step_scan_btn.enabled = waiting_for_step

            if status in ["completed", "error", "stopped"]:
                print(f"[DEBUG] Scan finished, re-enabling buttons")
                self.start_scan_btn.enabled = True
                self.stop_scan_btn.enabled = False
        
        gui.Application.instance.post_to_main_thread(self.window, update)
    
    def _on_scan_complete(self, scan_type: str, success: bool, file_path: str):
        """Callback for scan completion."""
        print(f"[DEBUG] Scan complete callback: type={scan_type}, success={success}, file={file_path}")
        if scan_type == config.SCAN_TYPE_PANORAMA and success:
            print("[INFO] Panorama capture completed on RP5. Waiting for MQTT image reassembly on laptop...")
        if success and os.path.exists(file_path):
            # Just notify - don't auto-load to avoid threading issues
            # User can manually switch to viz tab and load
            print(f"[INFO] Scan saved to: {file_path}")
            print(f"[INFO] Switch to Visualization tab and click 'Load File...' to view results")

    def _extract_panorama_frame_index(self, file_path: str):
        """Parse panorama frame index from names like panorama_00.jpg."""
        base = os.path.basename(file_path).lower()
        if not base.startswith("panorama_"):
            return None
        stem = base.split(".")[0]
        idx_text = stem.replace("panorama_", "")
        if not idx_text.isdigit():
            return None
        return int(idx_text)

    def _on_scan_data(self, scan_id: str, scan_type: str, file_paths: list):
        """Callback for completed MQTT reassembly on laptop."""
        print(f"[DEBUG] Scan data callback: id={scan_id}, type={scan_type}, files={len(file_paths)}")
        if scan_type != config.SCAN_TYPE_PANORAMA:
            return

        if scan_id not in self.panorama_frames_by_scan:
            self.panorama_frames_by_scan[scan_id] = {}

        indexed = self.panorama_frames_by_scan[scan_id]
        for p in file_paths:
            if not p.lower().endswith((".jpg", ".jpeg", ".png")):
                continue
            idx = self._extract_panorama_frame_index(p)
            if idx is None:
                continue
            indexed[idx] = p

        expected_indices = list(range(config.PANORAMA_EXPECTED_IMAGE_COUNT))
        selected_paths = [indexed[i] for i in expected_indices if i in indexed]

        if scan_id in self.panorama_stitched_scans:
            return

        if len(selected_paths) < config.PANORAMA_EXPECTED_IMAGE_COUNT:
            self._update_panorama_status(
                f"Received {len(selected_paths)} panorama images, waiting for {config.PANORAMA_EXPECTED_IMAGE_COUNT}"
            )
            return

        self._update_panorama_status(
            f"Received {len(selected_paths)} images. Stitching panorama..."
        )

        worker = threading.Thread(
            target=self._stitch_panorama_from_images,
            args=(selected_paths, False, scan_id),
            daemon=True,
        )
        worker.start()

    def _on_panorama_show(self):
        """Open stitched panorama using OpenCV renderer."""
        worker = threading.Thread(target=self._show_stitched_panorama, daemon=True)
        worker.start()

    def _on_panorama_save(self):
        """Save stitched panorama to persistent storage via file dialog."""
        worker = threading.Thread(target=self._save_panorama_via_dialog, daemon=True)
        worker.start()

    def _on_panorama_save_images(self):
        """Save latest individual panorama source images to a chosen folder."""
        worker = threading.Thread(target=self._save_panorama_images_via_dialog, daemon=True)
        worker.start()

    def _auto_stitch_existing_images_if_ready(self):
        """Auto-stitch if expected capture frames are already present on disk."""
        images = find_panorama_images(config.PANORAMA_IMAGES_DIR)
        indexed = {}
        for p in images:
            stem = os.path.basename(p).split(".")[0]
            try:
                idx = int(stem.replace("panorama_", ""))
            except ValueError:
                continue
            indexed[idx] = p

        expected_indices = list(range(config.PANORAMA_EXPECTED_IMAGE_COUNT))
        selected_paths = [indexed[i] for i in expected_indices if i in indexed]
        if len(selected_paths) < config.PANORAMA_EXPECTED_IMAGE_COUNT:
            return

        self._update_panorama_status(
            f"Found existing {len(selected_paths)} panorama images. Stitching..."
        )
        self._stitch_panorama_from_images(selected_paths, auto_show=False, scan_id="existing")

    def _stitch_panorama_from_images(self, image_paths: list, auto_show: bool, scan_id: str):
        """Perform OpenCV stitch and optionally show the result."""
        ok, message, output_path = stitch_equirectangular_panorama(
            image_paths=image_paths,
            output_path=config.PANORAMA_STITCHED_FILE,
            front_yaw_offset_deg=float(config.PANORAMA_FRONT_YAW_OFFSET_DEG),
            step_deg=float(config.PANORAMA_CAPTURE_STEP_DEG),
            rear_relative_yaw_deg=float(config.PANORAMA_REAR_RELATIVE_YAW_DEG),
        )
        if not ok:
            self._update_panorama_status(f"Stitch failed: {message}")
            return

        self._promote_panorama_sources(image_paths)
        self._prune_incoming_scan_folders()
        self.panorama_last_output = output_path
        self.panorama_stitched_scans.add(scan_id)
        if scan_id in self.panorama_frames_by_scan:
            del self.panorama_frames_by_scan[scan_id]
        self._update_panorama_status(message)

        if auto_show:
            self._show_stitched_panorama()

    def _promote_panorama_sources(self, image_paths: list):
        """Promote staged source captures to canonical images folder after stitch success."""
        os.makedirs(config.PANORAMA_IMAGES_DIR, exist_ok=True)

        # Remove old stitched panorama variants from previous successful runs.
        old_stitched = glob.glob(os.path.join(config.PANORAMA_IMAGES_DIR, "panorama_stitched*.jpg"))
        old_stitched += glob.glob(os.path.join(config.PANORAMA_IMAGES_DIR, "panorama_stitched*.jpeg"))
        old_stitched += glob.glob(os.path.join(config.PANORAMA_IMAGES_DIR, "panorama_stitched*.png"))
        canonical_stitched = os.path.normpath(config.PANORAMA_STITCHED_FILE)
        for path in old_stitched:
            if os.path.normpath(path) == canonical_stitched:
                continue
            try:
                os.remove(path)
            except Exception:
                pass

        existing = glob.glob(os.path.join(config.PANORAMA_IMAGES_DIR, "panorama_*.jpg"))
        existing += glob.glob(os.path.join(config.PANORAMA_IMAGES_DIR, "panorama_*.jpeg"))
        existing += glob.glob(os.path.join(config.PANORAMA_IMAGES_DIR, "panorama_*.png"))
        for path in existing:
            base = os.path.basename(path).lower()
            stem = base.split(".")[0]
            idx_text = stem.replace("panorama_", "")
            if idx_text.isdigit():
                try:
                    os.remove(path)
                except Exception:
                    pass

        promoted = []
        for src in image_paths:
            dst = os.path.join(config.PANORAMA_IMAGES_DIR, os.path.basename(src))
            shutil.copy2(src, dst)
            promoted.append(dst)

        self.panorama_last_sources = promoted

    def _prune_incoming_scan_folders(self):
        """Delete staged incoming folders after successful promotion/stitch."""
        incoming_root = config.PANORAMA_INCOMING_DIR
        if not os.path.isdir(incoming_root):
            return

        for child in os.listdir(incoming_root):
            child_path = os.path.join(incoming_root, child)
            if os.path.isdir(child_path):
                try:
                    shutil.rmtree(child_path)
                except Exception:
                    pass

    def _show_stitched_panorama(self):
        """Display latest stitched panorama in OpenCV window."""
        try:
            ok, message = show_panorama_window(self.panorama_last_output, "C270 Panorama")
            if ok:
                self._update_panorama_status(message)
            else:
                self._update_panorama_status(f"OpenCV view failed: {message}")
        except Exception as e:
            self._update_panorama_status(f"OpenCV view failed: {e}")

    def _save_panorama_via_dialog(self):
        """Save stitched panorama image with a timestamped default filename."""
        if not os.path.exists(self.panorama_last_output):
            self._update_panorama_status("No stitched panorama to save yet")
            return

        try:
            if not os.path.exists(config.PERSISTENT_DIR):
                os.makedirs(config.PERSISTENT_DIR)

            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            default_filename = f"panorama_{timestamp}.jpg"

            selected_file = [None]

            def ask_save_file():
                root = tk.Tk()
                root.withdraw()
                root.attributes('-topmost', True)

                filename = filedialog.asksaveasfilename(
                    parent=root,
                    title="Save Stitched Panorama",
                    initialdir=config.PERSISTENT_DIR,
                    initialfile=default_filename,
                    defaultextension=".jpg",
                    filetypes=[
                        ("JPEG files", "*.jpg"),
                        ("PNG files", "*.png"),
                        ("All files", "*.*")
                    ]
                )

                root.destroy()
                selected_file[0] = filename

            dialog_thread = threading.Thread(target=ask_save_file)
            dialog_thread.start()
            dialog_thread.join()

            filename = selected_file[0]
            if not filename:
                return

            shutil.copy2(self.panorama_last_output, filename)
            self._update_panorama_status(f"Saved panorama: {os.path.basename(filename)}")
        except Exception as e:
            self._update_panorama_status(f"Save failed: {e}")

    def _save_panorama_images_via_dialog(self):
        """Save latest promoted panorama source images into a chosen directory."""
        sources = [p for p in self.panorama_last_sources if os.path.exists(p)]
        if not sources:
            self._update_panorama_status("No panorama source images available to save")
            return

        try:
            selected_dir = [None]

            def ask_dir():
                root = tk.Tk()
                root.withdraw()
                root.attributes('-topmost', True)
                folder = filedialog.askdirectory(
                    parent=root,
                    title="Save Panorama Source Images",
                    initialdir=config.PERSISTENT_DIR if os.path.exists(config.PERSISTENT_DIR) else config.DATA_DIR,
                    mustexist=False,
                )
                root.destroy()
                selected_dir[0] = folder

            dialog_thread = threading.Thread(target=ask_dir)
            dialog_thread.start()
            dialog_thread.join()

            target_dir = selected_dir[0]
            if not target_dir:
                return

            os.makedirs(target_dir, exist_ok=True)
            copied = 0
            for src in sources:
                dst = os.path.join(target_dir, os.path.basename(src))
                shutil.copy2(src, dst)
                copied += 1

            self._update_panorama_status(f"Saved {copied} panorama source images")
        except Exception as e:
            self._update_panorama_status(f"Save images failed: {e}")

    def _update_panorama_status(self, message: str):
        """Thread-safe panorama status update."""
        def update():
            if self.panorama_status_label:
                self.panorama_status_label.text = f"Status: {message}"

        gui.Application.instance.post_to_main_thread(self.window, update)
    
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
                self.save_ply_btn.enabled = False
                print("[DEBUG] Clear complete")
                
            except Exception as e:
                print(f"[DEBUG] Clear error: {e}")
                import traceback
                traceback.print_exc()
        
        # Post to main thread to ensure thread-safe GUI updates
        gui.Application.instance.post_to_main_thread(self.window, clear_operation)
    
    def _on_save_ply(self):
        """Handle save PLY button click - save point cloud with timestamp to persistent folder."""
        print("[DEBUG] Save PLY button clicked")
        
        if not self.current_pcd:
            print("[DEBUG] No point cloud loaded to save")
            self._update_viz_status("Error: No point cloud loaded")
            return
        
        try:
            # Ensure persistent directory exists
            if not os.path.exists(config.PERSISTENT_DIR):
                os.makedirs(config.PERSISTENT_DIR)
                print(f"[DEBUG] Created persistent directory: {config.PERSISTENT_DIR}")
            
            # Generate timestamped filename
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            default_filename = f"scan_{timestamp}.ply"
            
            # CRITICAL FIX: Run tkinter dialog in separate thread to avoid crash
            selected_file = [None]  # Use list to store result from thread
            
            def ask_save_file():
                """Run file dialog in separate thread."""
                root = tk.Tk()
                root.withdraw()  # Hide the root window
                root.attributes('-topmost', True)  # Bring dialog to front
                
                print("[DEBUG] Opening save file dialog...")
                filename = filedialog.asksaveasfilename(
                    parent=root,
                    title="Save Point Cloud",
                    initialdir=config.PERSISTENT_DIR,
                    initialfile=default_filename,
                    defaultextension=".ply",
                    filetypes=[
                        ("PLY files", "*.ply"),
                        ("All files", "*.*")
                    ]
                )
                
                root.destroy()  # Clean up Tk window
                selected_file[0] = filename  # Store result
            
            # Run dialog in thread and wait for completion
            dialog_thread = threading.Thread(target=ask_save_file)
            dialog_thread.start()
            dialog_thread.join()  # Wait for thread to complete
            
            filename = selected_file[0]
            
            if filename:
                print(f"[DEBUG] Saving to: {filename}")
                
                # Save the point cloud
                success = o3d.io.write_point_cloud(filename, self.current_pcd)
                
                if success:
                    print(f"[INFO] Point cloud saved to: {filename}")
                    self._update_viz_status(f"Saved: {os.path.basename(filename)}")
                else:
                    print(f"[ERROR] Failed to save point cloud to: {filename}")
                    self._update_viz_status("Error: Failed to save file")
            else:
                print("[DEBUG] Save dialog cancelled")
                
        except Exception as e:
            print(f"[DEBUG] Save error: {e}")
            self._update_viz_status(f"Error: {str(e)}")
            import traceback
            traceback.print_exc()
    
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
            
            # Enable visualize and save buttons
            self.visualize_btn.enabled = True
            self.save_ply_btn.enabled = True
            
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

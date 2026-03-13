"""
Scan Controller for managing RPLidar scan execution via MQTT.

Communicates with Raspberry Pi scanner service over MQTT.
"""

import os
import sys
from typing import Callable, Optional

# Add parent directory to path for mqtt_protocol import
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from laptop_viewer_client import LaptopViewerClient
from mqtt_protocol import ScanStatus
from . import config


class ScanController:
    """
    Controls the execution of RPLidar scans via MQTT.
    
    Sends commands to Raspberry Pi scanner service and receives results.
    """
    
    def __init__(self):
        """Initialize the scan controller."""
        self.scan_running = False
        self.status_callback: Optional[Callable] = None
        self.completion_callback: Optional[Callable] = None
        self.current_scan_type: Optional[str] = None
        self.current_scan_id: Optional[str] = None
        
        # Initialize MQTT client
        try:
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "config_laptop.yaml"
            )
            self.mqtt_client = LaptopViewerClient(config_path)
            self.mqtt_client.set_status_callback(self._on_mqtt_status)
            self.mqtt_client.set_data_callback(self._on_mqtt_data)
            
            # Connect to broker
            self.mqtt_client.connect()
            print("[SCAN] Connected to MQTT broker")
        except Exception as e:
            print(f"[SCAN] Warning: Failed to initialize MQTT client: {e}")
            self.mqtt_client = None
    
    def set_status_callback(self, callback: Callable):
        """
        Set callback for status updates.
        
        Args:
            callback: Function(status: str, message: str)
        """
        self.status_callback = callback
    
    def set_completion_callback(self, callback: Callable):
        """
        Set callback for scan completion.
        
        Args:
            callback: Function(scan_type: str, success: bool, file_path: str)
        """
        self.completion_callback = callback
    
    def start_scan(self, scan_type: str, params: dict = None):
        """
        Start a scan via MQTT command to Raspberry Pi.
        
        Args:
            scan_type: "2d" or "3d"
            params: Optional parameters including:
                - port: Serial port (e.g., "/dev/ttyUSB0") or "auto"
        """
        if not self.mqtt_client:
            self._update_status("error", "MQTT client not initialized")
            return False
        
        if self.scan_running:
            self._update_status("error", "A scan is already running")
            return False
        
        params = params or {}
        
        if scan_type not in config.SUPPORTED_SCAN_TYPES:
            self._update_status(
                "error",
                f"Scan type '{scan_type}' not supported "
                f"(supported: {', '.join(config.SUPPORTED_SCAN_TYPES)})"
            )
            return False
        
        # Get port parameter
        port = params.get("port", "").strip()
        if not port:
            port = "auto"
        
        # Request scan via MQTT
        try:
            self.current_scan_type = scan_type
            self.scan_running = True
            
            self.current_scan_id = self.mqtt_client.request_scan(
                scan_type=scan_type,
                port=port
            )
            
            self._update_status("started", f"Scan request sent to Raspberry Pi (ID: {self.current_scan_id})")
            print(f"[SCAN] Scan requested: {self.current_scan_id}")
            return True
            
        except Exception as e:
            self._update_status("error", f"Failed to request scan: {e}")
            self.scan_running = False
            return False
    
    def send_input(self, text: str):
        """
        Send input (deprecated - no longer used for 2D scans).
        
        Args:
            text: Input text
        """
        print("[SCAN] send_input() not supported for MQTT-based scans")
    
    def stop_scan(self):
        """Stop the currently running scan."""
        if not self.scan_running or not self.current_scan_id:
            print("[SCAN] No scan running to stop")
            return
        
        if not self.mqtt_client:
            self._update_status("error", "MQTT client not available")
            return
        
        try:
            print(f"[SCAN] Requesting stop for scan: {self.current_scan_id}")
            self.mqtt_client.stop_scan(self.current_scan_id)
            self._update_status("stopped", "Stop request sent to Raspberry Pi")
            # Optimistic local clear so UI is never hard-stuck in running state.
            # If the scan is still active on RPi, next start request will receive "busy".
            self.scan_running = False
            self.current_scan_id = None
            self.current_scan_type = None
        except Exception as e:
            print(f"[SCAN] Error stopping scan: {e}")
            self._update_status("error", f"Error stopping scan: {e}")

    def step_scan(self):
        """Allow one step in the current 3D scan."""
        if not self.scan_running or not self.current_scan_id:
            print("[SCAN] No scan running to step")
            return

        if self.current_scan_type != config.SCAN_TYPE_3D:
            print("[SCAN] Step ignored: current scan is not 3D")
            return

        if not self.mqtt_client:
            self._update_status("error", "MQTT client not available")
            return

        try:
            print(f"[SCAN] Requesting step for scan: {self.current_scan_id}")
            self.mqtt_client.step_scan(self.current_scan_id)
            self._update_status("started", "Step permission sent to Raspberry Pi")
        except Exception as e:
            print(f"[SCAN] Error sending step: {e}")
            self._update_status("error", f"Error sending step: {e}")
    
    def is_running(self) -> bool:
        """Check if a scan is currently running."""
        return self.scan_running
    
    def _merge_robust_slices(self, scan_id: str):
        """Merges all slice files received for this scan into a single PLY."""
        import glob
        
        # Look for slice files in data directory
        data_dir = config.DATA_DIR
        # Pattern matches files sent by robust_3d_scan_module
        # Note: The filenames are like robust_slice_0_123456.ply. 
        # But LaptopViewerClient saves them as <scan_id>_<original_name> usually? 
        # No, LaptopViewerClient (based on typical implementation) usually saves raw file content.
        # Let's check LaptopViewerClient later if needed. For now assuming they land in data_dir.
        
        # Using the tracked file list from mqtt_client is safer
        files_to_merge = self.mqtt_client.received_files_by_scan.get(scan_id, [])
        if not files_to_merge:
            print("[SCAN] No files found to merge for robust scan")
            return

        print(f"[SCAN] Merging {len(files_to_merge)} slice files...")
        
        merged_points = []
        
        for file_path in files_to_merge:
            if not file_path.endswith(".ply"): continue
            
            try:
                with open(file_path, 'r') as f:
                    header_ended = False
                    for line in f:
                        if "end_header" in line:
                            header_ended = True
                            continue
                        if header_ended:
                            merged_points.append(line)
            except Exception as e:
                print(f"[SCAN] Error reading slice file {file_path}: {e}")
        
        # Write merged file
        output_file = config.SCAN_3D_PLY
        try:
            with open(output_file, 'w') as f:
                f.write(f"ply\nformat ascii 1.0\nelement vertex {len(merged_points)}\nproperty float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n")
                f.writelines(merged_points)
            print(f"[SCAN] Merged robust scan saved to {output_file}")
        except Exception as e:
            print(f"[SCAN] Error saving merged PLY: {e}")

    def _on_mqtt_status(self, scan_id: str, status: ScanStatus):
        """
        Handle status update from MQTT.
        
        Args:
            scan_id: Scan identifier
            status: Status message from Raspberry Pi
        """
        print(f"[SCAN] Status for {scan_id}: {status.status} - {status.message}")
        
        # Only process if it's our current scan
        if scan_id != self.current_scan_id:
            print(f"[SCAN] Status for different scan, ignoring")
            return
        
        # Update GUI status
        self._update_status(status.status, status.message)
        
        # Handle completion
        if status.status in ["completed", "error", "stopped"]:
            self.scan_running = False
            
            if status.status == "completed":
                # Success
                output_file = config.SCAN_3D_PLY # Default
                
                # Determine output file based on scan type
                if self.current_scan_type == config.SCAN_TYPE_3D:
                    output_file = config.SCAN_3D_PLY
                elif self.current_scan_type == "robust_3d":
                    self._merge_robust_slices(scan_id)
                    output_file = config.SCAN_3D_PLY
                    # Note: Completion callback will load this file
                else:
                    output_file = config.SCAN_2D_PLY
                
                if self.completion_callback:
                    self.completion_callback(self.current_scan_type, True, output_file)
            else:
                # Error or stopped
                if self.completion_callback:
                    self.completion_callback(self.current_scan_type, False, "")
            
            self.current_scan_id = None
            self.current_scan_type = None
    
    def _on_mqtt_data(self, scan_id: str, file_paths: list):
        """
        Handle data received from MQTT.
        
        Args:
            scan_id: Scan identifier
            file_paths: List of received file paths
        """
        print(f"[SCAN] Data received for {scan_id}:")
        for path in file_paths:
            print(f"  - {path}")
        
        # Files are automatically saved by the MQTT client
        # GUI can now load them
    
    def _update_status(self, status: str, message: str):
        """Update status via callback."""
        print(f"[STATUS] {status}: {message}")
        if self.status_callback:
            self.status_callback(status, message)


# Standalone test
if __name__ == "__main__":
    import time
    
    def status_cb(status, message):
        print(f"Status: {status} - {message}")
    
    def complete_cb(scan_type, success, file_path):
        print(f"Scan complete: type={scan_type}, success={success}, file={file_path}")
    
    print("="*70)
    print("Scan Controller Test (MQTT Mode)")
    print("="*70)
    print()
    
    controller = ScanController()
    controller.set_status_callback(status_cb)
    controller.set_completion_callback(complete_cb)
    
    print("Starting 2D scan...")
    controller.start_scan("2d")
    
    # Wait for completion
    print("Waiting for scan to complete...")
    try:
        while controller.is_running():
            time.sleep(1)
        print("\nScan finished!")
        time.sleep(2)  # Wait for data transfer
    except KeyboardInterrupt:
        print("\nStopping scan...")
        controller.stop_scan()
    
    print("Done!")

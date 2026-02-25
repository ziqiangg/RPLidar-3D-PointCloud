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
        except Exception as e:
            print(f"[SCAN] Error stopping scan: {e}")
            self._update_status("error", f"Error stopping scan: {e}")
    
    def is_running(self) -> bool:
        """Check if a scan is currently running."""
        return self.scan_running
    
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
                # Service writes scan.ply for both 2D and 3D scan modes.
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

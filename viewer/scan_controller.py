"""
Scan Controller for managing RPLidar scan execution.

Provides interface for running different scan scripts and monitoring their status.
"""

import os
import sys
import subprocess
import threading
from typing import Callable, Optional
from . import config


class ScanController:
    """
    Controls the execution of RPLidar scan scripts.
    
    Manages running dump_one_scan.py (2D) and xyzscan_servoless.py (3D) scans.
    """
    
    def __init__(self):
        """Initialize the scan controller."""
        self.scan_running = False
        self.scan_process: Optional[subprocess.Popen] = None
        self.scan_thread: Optional[threading.Thread] = None
        self.status_callback: Optional[Callable] = None
        self.completion_callback: Optional[Callable] = None
        self.current_scan_type: Optional[str] = None
        
        # Get Python executable from current environment
        self.python_executable = sys.executable
    
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
        Start a scan of the specified type.
        
        Args:
            scan_type: "2d" or "3d"
            params: Optional parameters including:
                - port: Serial port (e.g., "COM3", "/dev/ttyUSB0")
        """
        if self.scan_running:
            self._update_status("error", "A scan is already running")
            return False
        
        params = params or {}
        
        if scan_type == config.SCAN_TYPE_2D:
            script_path = config.SCRIPT_2D_SCAN
            output_file = config.SCAN_2D_PLY
        elif scan_type == config.SCAN_TYPE_3D:
            script_path = config.SCRIPT_3D_SCAN
            output_file = config.SCAN_3D_PLY
        else:
            self._update_status("error", f"Unknown scan type: {scan_type}")
            return False
        
        # Verify script exists
        if not os.path.exists(script_path):
            self._update_status("error", f"Scan script not found: {script_path}")
            return False
        
        # Build command
        cmd = [self.python_executable, script_path]
        
        # Add port if specified
        if "port" in params and params["port"]:
            cmd.append(params["port"])
        
        # Start scan in a separate thread
        self.scan_thread = threading.Thread(
            target=self._run_scan,
            args=(scan_type, cmd, output_file)
        )
        self.scan_thread.daemon = True
        self.scan_thread.start()
        
        return True
    
    def send_input(self, text: str):
        """
        Send input to the running scan process.
        
        Args:
            text: Input text to send (e.g., servo angle or 'q')
        """
        if self.scan_running and self.scan_process and self.scan_process.stdin:
            try:
                self.scan_process.stdin.write(text + "\n")
                self.scan_process.stdin.flush()
                print(f"[SCAN] Sent input: {text}")
            except Exception as e:
                print(f"[SCAN] Error sending input: {e}")
                self._update_status("error", f"Error sending input: {e}")
    
    def stop_scan(self):
        """Stop the currently running scan."""
        if self.scan_running and self.scan_process:
            try:
                print("[SCAN] Stopping scan...")
                # For interactive 3D scans, send 'q' first
                if self.current_scan_type == config.SCAN_TYPE_3D and self.scan_process.stdin:
                    try:
                        self.send_input('q')
                        # Wait briefly for graceful exit
                        self.scan_process.wait(timeout=3)
                        self._update_status("stopped", "Scan stopped by user")
                        return
                    except subprocess.TimeoutExpired:
                        pass
                
                # Try graceful termination
                self.scan_process.terminate()
                try:
                    self.scan_process.wait(timeout=2)
                    self._update_status("stopped", "Scan stopped by user")
                except subprocess.TimeoutExpired:
                    # Force kill if terminate didn't work
                    print("[SCAN] Process not responding, force killing...")
                    self.scan_process.kill()
                    self.scan_process.wait(timeout=2)
                    self._update_status("stopped", "Scan forcefully stopped")
            except Exception as e:
                print(f"[SCAN] Error stopping scan: {e}")
                self._update_status("error", f"Error stopping scan: {e}")
            finally:
                self.scan_running = False
                self.scan_process = None
                self.current_scan_type = None
    
    def is_running(self) -> bool:
        """Check if a scan is currently running."""
        return self.scan_running
    
    def _run_scan(self, scan_type: str, cmd: list, output_file: str):
        """
        Run the scan script (internal method, runs in separate thread).
        
        Args:
            scan_type: Type of scan ("2d" or "3d")
            cmd: Command to execute
            output_file: Expected output file path
        """
        self.scan_running = True
        self.current_scan_type = scan_type
        self._update_status("started", f"Starting {scan_type} scan...")
        
        try:
            # Run the scan script (enable stdin for 3D interactive mode)
            self.scan_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # Merge stderr into stdout for unified output
                stdin=subprocess.PIPE if scan_type == config.SCAN_TYPE_3D else None,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            # Stream output
            for line in self.scan_process.stdout:
                print(f"[SCAN] {line.rstrip()}")
                self._update_status("running", line.rstrip())
            
            # Wait for completion
            return_code = self.scan_process.wait()
            
            if return_code == 0:
                # Check if output file was created
                if os.path.exists(output_file):
                    self._update_status("completed", f"Scan completed successfully")
                    if self.completion_callback:
                        self.completion_callback(scan_type, True, output_file)
                else:
                    self._update_status("error", "Scan completed but output file not found")
                    if self.completion_callback:
                        self.completion_callback(scan_type, False, "")
            else:
                # Scan failed or was terminated
                self._update_status("error", f"Scan exited with code {return_code}")
                if self.completion_callback:
                    self.completion_callback(scan_type, False, "")
        
        except Exception as e:
            self._update_status("error", f"Error running scan: {e}")
            if self.completion_callback:
                self.completion_callback(scan_type, False, "")
        
        finally:
            self.scan_running = False
            self.scan_process = None
            self.current_scan_type = None
    
    def _update_status(self, status: str, message: str):
        """Update status via callback."""
        print(f"[STATUS] {status}: {message}")
        if self.status_callback:
            self.status_callback(status, message)


# Standalone test
if __name__ == "__main__":
    def status_cb(status, message):
        print(f"Status: {status} - {message}")
    
    def complete_cb(scan_type, success, file_path):
        print(f"Scan complete: type={scan_type}, success={success}, file={file_path}")
    
    controller = ScanController()
    controller.set_status_callback(status_cb)
    controller.set_completion_callback(complete_cb)
    
    print("Starting 2D scan...")
    controller.start_scan("2d")
    
    # Wait for completion
    import time
    while controller.is_running():
        time.sleep(1)
    
    print("Done!")

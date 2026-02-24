"""
Raspberry Pi Scanner Service

This service runs on the Raspberry Pi and:
1. Listens for scan commands via MQTT
2. Executes scan scripts (dump_one_scan.py)
3. Sends final status back to laptop
4. Transmits scan data files via MQTT
"""

import os
import sys
import time
import uuid
import yaml
import logging
import traceback
from pathlib import Path
from typing import Optional

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mqtt_protocol import (
    MQTTClientBase,
    Topics,
    ScanCommand,
    StopCommand,
    ScanStatus,
    DataMessage,
    MessageDecoder
)


class RPiScannerService(MQTTClientBase):
    """
    Scanner service for Raspberry Pi.
    
    Listens for scan commands and executes scanning scripts.
    """
    
    def __init__(self, config_path: str = "config_rpi.yaml"):
        """Initialize scanner service with configuration."""
        # Load configuration
        self.config = self._load_config(config_path)
        
        # Setup logging
        self._setup_logging()
        
        # Initialize MQTT client
        super().__init__(
            broker_host=self.config['mqtt']['broker_host'],
            broker_port=self.config['mqtt']['broker_port'],
            client_id=self.config['device']['client_id'],
            qos=self.config['mqtt']['qos']
        )
        
        # Scanner state
        self.current_scan_id: Optional[str] = None
        self.scan_running = False
        
        self.logger.info("RPi Scanner Service initialized")
    
    def _load_config(self, config_path: str) -> dict:
        """Load configuration from YAML file."""
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    
    def _setup_logging(self):
        """Setup logging configuration."""
        log_level = getattr(logging, self.config['logging']['level'])
        log_file = self.config['logging']['file']
        
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
        
        self.logger = logging.getLogger('RPiScannerService')
    
    def on_connected(self):
        """Called when connected to MQTT broker."""
        self.logger.info("Connected to MQTT broker - subscribing to command topics")
        
        # Subscribe to command topics
        self.subscribe(Topics.COMMAND_SCAN, self._handle_scan_command)
        self.subscribe(Topics.COMMAND_STOP, self._handle_stop_command)
        
        self.logger.info("Ready to receive scan commands")
    
    def on_disconnected(self):
        """Called when disconnected from broker."""
        self.logger.warning("Disconnected from MQTT broker - will attempt reconnect")
    
    def _handle_scan_command(self, topic: str, payload: bytes):
        """Handle incoming scan command."""
        try:
            # Parse command
            command = ScanCommand.from_json(payload.decode('utf-8'))
            self.logger.info(f"Received scan command: {command.scan_id}, type: {command.scan_type}")
            
            # Check if already scanning
            if self.scan_running:
                self.logger.warning(f"Scan already in progress: {self.current_scan_id}")
                status = ScanStatus.create_error(
                    command.scan_id,
                    "Scanner busy - another scan is in progress",
                    f"Current scan: {self.current_scan_id}"
                )
                self.publish(Topics.status_topic(command.scan_id), status.to_json())
                return
            
            # Execute scan
            self._execute_scan(command)
            
        except Exception as e:
            self.logger.error(f"Error handling scan command: {e}")
            self.logger.error(traceback.format_exc())
    
    def _handle_stop_command(self, topic: str, payload: bytes):
        """Handle incoming stop command."""
        try:
            command = StopCommand.from_json(payload.decode('utf-8'))
            self.logger.info(f"Received stop command for scan: {command.scan_id}")
            
            if self.current_scan_id == command.scan_id and self.scan_running:
                self.logger.info("Stopping current scan")
                # Note: dump_one_scan.py doesn't support interruption
                # This is a limitation - would need to modify the script
                # For now, just log the request
                self.logger.warning("Stop command received but scan cannot be interrupted")
            else:
                self.logger.info("No matching scan to stop")
                
        except Exception as e:
            self.logger.error(f"Error handling stop command: {e}")
    
    def _execute_scan(self, command: ScanCommand):
        """Execute a scan based on command."""
        self.current_scan_id = command.scan_id
        self.scan_running = True
        
        try:
            self.logger.info(f"Starting scan {command.scan_id}")
            
            # Import and run the scan directly (not subprocess)
            # This allows us to capture the results programmatically
            scan_result = self._run_scan_2d(command.port)
            
            if scan_result['success']:
                self.logger.info(f"Scan completed successfully: {scan_result['point_count']} points")
                
                # Send success status
                status = ScanStatus.create_completed(
                    command.scan_id,
                    scan_result['point_count'],
                    f"Scan completed with {scan_result['point_count']} points"
                )
                self.publish(Topics.status_topic(command.scan_id), status.to_json())
                
                # Send data files
                self._send_scan_data(command.scan_id, scan_result['files'])
                
            else:
                self.logger.error(f"Scan failed: {scan_result['error']}")
                
                # Send error status
                status = ScanStatus.create_error(
                    command.scan_id,
                    "Scan execution failed",
                    scan_result['error']
                )
                self.publish(Topics.status_topic(command.scan_id), status.to_json())
        
        except Exception as e:
            self.logger.error(f"Exception during scan execution: {e}")
            self.logger.error(traceback.format_exc())
            
            # Send error status
            status = ScanStatus.create_error(
                command.scan_id,
                "Unexpected error during scan",
                str(e)
            )
            self.publish(Topics.status_topic(command.scan_id), status.to_json())
        
        finally:
            self.scan_running = False
            self.current_scan_id = None
    
    def _run_scan_2d(self, port: str) -> dict:
        """
        Run 2D scan by importing and executing dump_one_scan logic.
        
        Returns:
            dict with keys: success, point_count, files, error
        """
        try:
            # Import the scan module
            import dump_one_scan as scan_module
            
            # Determine port
            if port == "auto":
                from utils.port_config import get_default_port
                port = get_default_port()
            
            self.logger.info(f"Using port: {port}")
            
            # Execute scan by calling module functions
            # We need to refactor dump_one_scan.py to be importable
            # For now, run as subprocess
            import subprocess
            
            script_path = self.config['scripts']['scan_2d']
            cmd = [sys.executable, script_path]
            
            if port != "auto":
                cmd.append(port)
            
            self.logger.info(f"Executing: {' '.join(cmd)}")
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30  # 30 second timeout
            )
            
            if result.returncode == 0:
                self.logger.info("Scan script completed successfully")
                self.logger.debug(f"Output: {result.stdout}")
                
                # Check for output files
                data_dir = self.config['data']['output_dir']
                csv_file = os.path.join(data_dir, "scan.csv")
                ply_file = os.path.join(data_dir, "scan.ply")
                
                if not os.path.exists(csv_file) or not os.path.exists(ply_file):
                    return {
                        'success': False,
                        'error': 'Output files not found',
                        'point_count': 0,
                        'files': []
                    }
                
                # Count points from CSV
                point_count = self._count_csv_points(csv_file)
                
                return {
                    'success': True,
                    'point_count': point_count,
                    'files': [csv_file, ply_file],
                    'error': None
                }
            else:
                self.logger.error(f"Scan script failed with code {result.returncode}")
                self.logger.error(f"stderr: {result.stderr}")
                return {
                    'success': False,
                    'error': f"Script failed: {result.stderr}",
                    'point_count': 0,
                    'files': []
                }
                
        except subprocess.TimeoutExpired:
            self.logger.error("Scan script timed out")
            return {
                'success': False,
                'error': 'Scan timed out after 30 seconds',
                'point_count': 0,
                'files': []
            }
        except Exception as e:
            self.logger.error(f"Error running scan: {e}")
            self.logger.error(traceback.format_exc())
            return {
                'success': False,
                'error': str(e),
                'point_count': 0,
                'files': []
            }
    
    def _count_csv_points(self, csv_file: str) -> int:
        """Count number of points in CSV file."""
        try:
            with open(csv_file, 'r') as f:
                # Skip header
                next(f)
                return sum(1 for line in f)
        except Exception as e:
            self.logger.error(f"Error counting CSV points: {e}")
            return 0
    
    def _send_scan_data(self, scan_id: str, files: list):
        """Send scan data files via MQTT."""
        try:
            for file_path in files:
                file_format = 'csv' if file_path.endswith('.csv') else 'ply'
                
                self.logger.info(f"Sending {file_format} file: {file_path}")
                
                # Create chunked messages
                messages = DataMessage.create_from_file(
                    scan_id,
                    file_path,
                    file_format,
                    chunk_size=256 * 1024  # 256KB chunks
                )
                
                self.logger.info(f"Sending {len(messages)} chunks for {file_format}")
                
                # Publish each chunk
                for msg in messages:
                    success = self.publish(Topics.data_topic(scan_id), msg.to_json())
                    if not success:
                        self.logger.error(f"Failed to send chunk {msg.chunk_index}")
                    time.sleep(0.05)  # Small delay between chunks
                
                self.logger.info(f"Completed sending {file_format} file")
                
        except Exception as e:
            self.logger.error(f"Error sending scan data: {e}")
            self.logger.error(traceback.format_exc())
    
    def run(self):
        """Run the scanner service."""
        self.logger.info("Starting RPi Scanner Service...")
        
        # Connect to MQTT broker
        if not self.connect():
            self.logger.error("Failed to connect to MQTT broker - exiting")
            return
        
        self.logger.info("Scanner service running - press Ctrl+C to exit")
        
        try:
            # Keep alive
            while True:
                time.sleep(1)
                
                # Reconnect if disconnected
                if not self.is_connected():
                    self.logger.warning("Not connected - attempting reconnect")
                    self.connect()
                    
        except KeyboardInterrupt:
            self.logger.info("Shutdown requested")
        finally:
            self.disconnect()
            self.logger.info("Scanner service stopped")


def main():
    """Main entry point."""
    print("="*70)
    print("RPLidar Scanner Service (Raspberry Pi)")
    print("="*70)
    print()
    
    # Check for config file  
    config_file = "config_rpi.yaml"
    if not os.path.exists(config_file):
        print(f"ERROR: Configuration file not found: {config_file}")
        print("Please create config_rpi.yaml before running the service")
        sys.exit(1)
    
    # Create and run service
    service = RPiScannerService(config_file)
    service.run()


if __name__ == "__main__":
    main()

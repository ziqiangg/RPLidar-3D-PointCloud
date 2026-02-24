"""
Laptop Viewer MQTT Client

Handles MQTT communication for the laptop GUI viewer.
Sends scan commands and receives data/status from Raspberry Pi.
"""

import os
import sys
import uuid
import yaml
import logging
import time
from pathlib import Path
from typing import Optional, Callable, Dict

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


class LaptopViewerClient(MQTTClientBase):
    """
    MQTT client for laptop viewer application.
    
    Handles communication with Raspberry Pi scanner service.
    """
    
    def __init__(self, config_path: str = "config_laptop.yaml"):
        """Initialize laptop viewer client."""
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
        
        # Data reassembly buffers
        self.data_buffers: Dict[str, Dict] = {}  # scan_id -> buffer info
        
        # Callbacks for GUI
        self.status_callback: Optional[Callable] = None
        self.data_callback: Optional[Callable] = None
        
        self.logger.info("Laptop Viewer Client initialized")
    
    def _load_config(self, config_path: str) -> dict:
        """Load configuration from YAML file."""
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    
    def _setup_logging(self):
        """Setup logging configuration."""
        log_level = getattr(logging, self.config['logging']['level'])
        
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        self.logger = logging.getLogger('LaptopViewerClient')
    
    def set_status_callback(self, callback: Callable):
        """
        Set callback for status updates.
        
        Args:
            callback: Function(scan_id: str, status: ScanStatus)
        """
        self.status_callback = callback
    
    def set_data_callback(self, callback: Callable):
        """
        Set callback for completed data reception.
        
        Args:
            callback: Function(scan_id: str, file_paths: List[str])
        """
        self.data_callback = callback
    
    def on_connected(self):
        """Called when connected to MQTT broker."""
        self.logger.info("Connected to MQTT broker - subscribing to topics")
        
        # Subscribe to status and data topics
        self.subscribe(Topics.all_status(), self._handle_status_message)
        self.subscribe(Topics.all_data(), self._handle_data_message)
        
        self.logger.info("Ready to send scan commands")
    
    def on_disconnected(self):
        """Called when disconnected from broker."""
        self.logger.warning("Disconnected from MQTT broker")
    
    def request_scan(self, scan_type: str = "2d", port: str = "auto") -> str:
        """
        Request a scan from Raspberry Pi.
        
        Args:
            scan_type: Type of scan ("2d" only for now)
            port: Serial port or "auto"
        
        Returns:
            str: Scan ID
        """
        scan_id = str(uuid.uuid4())
        
        command = ScanCommand(
            scan_id=scan_id,
            scan_type=scan_type,
            port=port
        )
        
        self.logger.info(f"Requesting {scan_type} scan: {scan_id}")
        success = self.publish(Topics.COMMAND_SCAN, command.to_json())
        
        if success:
            self.logger.info(f"Scan command sent: {scan_id}")
        else:
            self.logger.error(f"Failed to send scan command: {scan_id}")
        
        return scan_id
    
    def stop_scan(self, scan_id: str):
        """
        Request to stop a scan.
        
        Args:
            scan_id: ID of scan to stop
        """
        command = StopCommand(scan_id=scan_id)
        self.logger.info(f"Requesting stop for scan: {scan_id}")
        self.publish(Topics.COMMAND_STOP, command.to_json())
    
    def _handle_status_message(self, topic: str, payload: bytes):
        """Handle incoming status message."""
        try:
            status = ScanStatus.from_json(payload.decode('utf-8'))
            self.logger.info(f"Received status for {status.scan_id}: {status.status}")
            
            # Notify callback
            if self.status_callback:
                self.status_callback(status.scan_id, status)
                
        except Exception as e:
            self.logger.error(f"Error handling status message: {e}")
    
    def _handle_data_message(self, topic: str, payload: bytes):
        """Handle incoming data chunk."""
        try:
            data_msg = DataMessage.from_json(payload.decode('utf-8'))
            scan_id = data_msg.scan_id
            
            self.logger.debug(f"Received data chunk {data_msg.chunk_index}/{data_msg.total_chunks} "
                            f"for scan {scan_id} ({data_msg.format})")
            
            # Initialize buffer if needed
            key = f"{scan_id}_{data_msg.format}"
            if key not in self.data_buffers:
                self.data_buffers[key] = {
                    'scan_id': scan_id,
                    'format': data_msg.format,
                    'total_chunks': data_msg.total_chunks,
                    'chunks': {},
                    'metadata': data_msg.metadata
                }
            
            # Store chunk
            buffer = self.data_buffers[key]
            buffer['chunks'][data_msg.chunk_index] = data_msg.decode_data()
            
            # Check if complete
            if len(buffer['chunks']) == buffer['total_chunks']:
                self.logger.info(f"All chunks received for {scan_id} ({data_msg.format})")
                self._reassemble_file(key)
                
        except Exception as e:
            self.logger.error(f"Error handling data message: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def _reassemble_file(self, buffer_key: str):
        """Reassemble file from chunks."""
        try:
            buffer = self.data_buffers[buffer_key]
            scan_id = buffer['scan_id']
            file_format = buffer['format']
            
            self.logger.info(f"Reassembling {file_format} file for scan {scan_id}")
            
            # Ensure data directory exists
            data_dir = self.config['data']['receive_dir']
            os.makedirs(data_dir, exist_ok=True)
            
            # Reassemble chunks in order
            file_data = b''
            for i in range(buffer['total_chunks']):
                if i not in buffer['chunks']:
                    self.logger.error(f"Missing chunk {i} for {scan_id}")
                    return
                file_data += buffer['chunks'][i]
            
            # Save file
            file_extension = 'csv' if file_format == 'csv' else 'ply'
            file_path = os.path.join(data_dir, f"scan.{file_extension}")
            
            with open(file_path, 'wb') as f:
                f.write(file_data)
            
            self.logger.info(f"Saved {file_format} file: {file_path}")
            
            # Clean up buffer
            del self.data_buffers[buffer_key]
            
            # Notify callback if all files received
            if self.data_callback and self._all_files_received(scan_id):
                csv_path = os.path.join(data_dir, "scan.csv")
                ply_path = os.path.join(data_dir, "scan.ply")
                self.data_callback(scan_id, [csv_path, ply_path])
                
        except Exception as e:
            self.logger.error(f"Error reassembling file: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def _all_files_received(self, scan_id: str) -> bool:
        """Check if all files for a scan have been received."""
        # Check if any buffers still exist for this scan
        for key in self.data_buffers:
            if scan_id in key:
                return False
        return True


def main():
    """Test client standalone."""
    print("="*70)
    print("Laptop Viewer Client (Test Mode)")
    print("="*70)
    print()
    
    client = LaptopViewerClient()
    
    def on_status(scan_id, status):
        print(f"\n[STATUS] {scan_id}: {status.status} - {status.message}")
        if status.point_count:
            print(f"         Points: {status.point_count}")
    
    def on_data(scan_id, files):
        print(f"\n[DATA] Received files for {scan_id}:")
        for f in files:
            print(f"       {f}")
    
    client.set_status_callback(on_status)
    client.set_data_callback(on_data)
    
    if not client.connect():
        print("ERROR: Failed to connect to MQTT broker")
        return
    
    print("Connected! Waiting for 2 seconds...")
    time.sleep(2)
    
    # Request a scan
    scan_id = client.request_scan("2d", "auto")
    print(f"\nRequested scan: {scan_id}")
    
    try:
        print("\nWaiting for scan results... (press Ctrl+C to exit)")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()

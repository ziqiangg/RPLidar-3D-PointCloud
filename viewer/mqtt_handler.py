"""
MQTT Handler for RPLidar Viewer application.

Handles MQTT communication for controlling edge devices and receiving scan results.
"""

import json
import threading
import paho.mqtt.client as mqtt
from typing import Callable, Optional
from . import config


class MQTTHandler:
    """
    Manages MQTT connections for controlling edge device and receiving scan results.
    
    - Publishes scan commands to edge device
    - Receives status updates from edge device
    - Receives scan result file paths from edge device
    """
    
    def __init__(self, broker: str = None, port: int = None):
        """
        Initialize MQTT handler.
        
        Args:
            broker: MQTT broker address (defaults to config.MQTT_BROKER)
            port: MQTT broker port (defaults to config.MQTT_PORT)
        """
        self.broker = broker or config.MQTT_BROKER
        self.port = port or config.MQTT_PORT
        self.client = mqtt.Client(client_id="rplidar_viewer")
        self.connected = False
        self.status_callback: Optional[Callable] = None
        self.result_callback: Optional[Callable] = None
        
        # Set up callbacks
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        
    def set_status_callback(self, callback: Callable):
        """
        Set the callback function for handling status updates from edge device.
        
        Args:
            callback: Function(status: str, message: str)
        """
        self.status_callback = callback
    
    def set_result_callback(self, callback: Callable):
        """
        Set the callback function for handling scan results from edge device.
        
        Args:
            callback: Function(scan_type: str, file_path: str, success: bool)
        """
        self.result_callback = callback
    
    def connect(self) -> bool:
        """
        Connect to the MQTT broker.
        
        Returns:
            bool: True if connection initiated successfully
        """
        try:
            self.client.connect(self.broker, self.port, config.MQTT_KEEPALIVE)
            self.client.loop_start()
            return True
        except Exception as e:
            print(f"MQTT connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the MQTT broker."""
        self.client.loop_stop()
        self.client.disconnect()
        self.connected = False
    
    def send_scan_command(self, scan_type: str, params: dict = None):
        """
        Send scan command to edge device.
        
        Args:
            scan_type: Type of scan ("2d" or "3d")
            params: Optional parameters (port, angles, etc.)
        """
        payload = {
            "scan_type": scan_type,
            "params": params or {}
        }
        self.client.publish(
            config.MQTT_TOPIC_SCAN_COMMAND,
            json.dumps(payload),
            qos=1
        )
        print(f"Sent scan command: {scan_type}")
    
    def _on_connect(self, client, userdata, flags, rc):
        """Callback for when the client connects to the broker."""
        if rc == 0:
            self.connected = True
            print(f"Connected to MQTT broker at {self.broker}:{self.port}")
            # Subscribe to status and result topics from edge device
            client.subscribe(config.MQTT_TOPIC_SCAN_STATUS)
            client.subscribe(config.MQTT_TOPIC_SCAN_RESULT)
            print(f"Subscribed to: {config.MQTT_TOPIC_SCAN_STATUS}, {config.MQTT_TOPIC_SCAN_RESULT}")
        else:
            print(f"MQTT connection failed with code {rc}")
    
    def _on_disconnect(self, client, userdata, rc):
        """Callback for when the client disconnects from the broker."""
        self.connected = False
        if rc != 0:
            print(f"Unexpected MQTT disconnection (code {rc})")
        else:
            print("Disconnected from MQTT broker")
    
    def _on_message(self, client, userdata, msg):
        """
        Callback for when a message is received from edge device.
        
        Handles:
        - Status updates on MQTT_TOPIC_SCAN_STATUS
        - Scan results on MQTT_TOPIC_SCAN_RESULT
        """
        try:
            payload = json.loads(msg.payload.decode())
            
            if msg.topic == config.MQTT_TOPIC_SCAN_STATUS:
                # Status update from edge device
                status = payload.get("status", "")
                message = payload.get("message", "")
                print(f"Edge device status: {status} - {message}")
                
                if self.status_callback:
                    self.status_callback(status, message)
            
            elif msg.topic == config.MQTT_TOPIC_SCAN_RESULT:
                # Scan result from edge device
                scan_type = payload.get("scan_type", "")
                file_path = payload.get("file_path", "")
                success = payload.get("success", True)
                
                print(f"Scan result received: type={scan_type}, path={file_path}, success={success}")
                
                if self.result_callback:
                    self.result_callback(scan_type, file_path, success)
                
        except json.JSONDecodeError as e:
            print(f"Invalid JSON in MQTT message: {e}")
        except Exception as e:
            print(f"Error processing MQTT message: {e}")


# Standalone test
if __name__ == "__main__":
    def test_status(status, message):
        print(f"Status: {status} - {message}")
    
    def test_result(scan_type, file_path, success):
        print(f"Result: {scan_type}, {file_path}, success={success}")
    
    handler = MQTTHandler()
    handler.set_status_callback(test_status)
    handler.set_result_callback(test_result)
    
    if handler.connect():
        print("MQTT handler running. Press Ctrl+C to exit.")
        print("Listening for edge device messages...")
        try:
            import time
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down...")
            handler.disconnect()

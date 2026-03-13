"""
Base MQTT client wrapper for common functionality.
"""

import paho.mqtt.client as mqtt
from typing import Callable, Optional, Dict, Any
import logging


class MQTTClientBase:
    """
    Base MQTT client providing common connection and communication methods.
    
    This class should be subclassed by specific client implementations
    (RPi Scanner Service, Laptop GUI Client).
    """
    
    def __init__(self, 
                 broker_host: str,
                 broker_port: int = 1883,
                 client_id: str = None,
                 qos: int = 1):
        """
        Initialize MQTT client base.
        
        Args:
            broker_host: MQTT broker hostname or IP
            broker_port: MQTT broker port (default 1883)
            client_id: Unique client identifier
            qos: Quality of Service level (0, 1, or 2)
        """
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.qos = qos
        
        # Create MQTT client
        self.client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)
        
        # Set callbacks
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        
        # Logging
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # Connection state
        self.connected = False
        
        # Custom callbacks
        self.message_callbacks: Dict[str, Callable] = {}
    
    def connect(self) -> bool:
        """
        Connect to MQTT broker.
        
        Returns:
            bool: True if connection successful
        """
        try:
            self.logger.info(f"Connecting to MQTT broker at {self.broker_host}:{self.broker_port}")
            self.client.connect(self.broker_host, self.broker_port, keepalive=60)
            self.client.loop_start()
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to MQTT broker: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from MQTT broker."""
        self.logger.info("Disconnecting from MQTT broker")
        self.client.loop_stop()
        self.client.disconnect()
        self.connected = False
    
    def subscribe(self, topic: str, callback: Optional[Callable] = None):
        """
        Subscribe to an MQTT topic.
        
        Args:
            topic: Topic to subscribe to (supports wildcards)
            callback: Optional callback for this specific topic
        """
        self.logger.info(f"Subscribing to topic: {topic}")
        self.client.subscribe(topic, qos=self.qos)
        
        if callback:
            self.message_callbacks[topic] = callback
    
    def publish(self, topic: str, payload: str, qos: Optional[int] = None) -> bool:
        """
        Publish message to MQTT topic.
        
        Args:
            topic: Topic to publish to
            payload: Message payload (string)
            qos: Quality of Service (uses default if not specified)
        
        Returns:
            bool: True if publish successful
        """
        if qos is None:
            qos = self.qos
        
        try:
            result = self.client.publish(topic, payload, qos=qos)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.debug(f"Published to {topic}: {payload[:100]}...")
                return True
            else:
                self.logger.error(f"Failed to publish to {topic}: {result.rc}")
                return False
        except Exception as e:
            self.logger.error(f"Exception publishing to {topic}: {e}")
            return False
    
    def _on_connect(self, client, userdata, flags, rc):
        """Internal callback for connection events."""
        if rc == 0:
            self.logger.info("Connected to MQTT broker successfully")
            self.connected = True
            self.on_connected()
        else:
            self.logger.error(f"Failed to connect to MQTT broker: {rc}")
            self.connected = False
    
    def _on_disconnect(self, client, userdata, rc):
        """Internal callback for disconnection events."""
        self.logger.warning(f"Disconnected from MQTT broker: {rc}")
        self.connected = False
        self.on_disconnected()
    
    def _on_message(self, client, userdata, msg):
        """Internal callback for received messages."""
        try:
            # Check for topic-specific callbacks
            for topic_pattern, callback in self.message_callbacks.items():
                if mqtt.topic_matches_sub(topic_pattern, msg.topic):
                    callback(msg.topic, msg.payload)
                    return
            
            # Fall back to general handler
            self.on_message_received(msg.topic, msg.payload)
            
        except Exception as e:
            self.logger.error(f"Error processing message: {e}")
    
    # Override these methods in subclasses
    
    def on_connected(self):
        """Called when successfully connected to broker."""
        pass
    
    def on_disconnected(self):
        """Called when disconnected from broker."""
        pass
    
    def on_message_received(self, topic: str, payload: bytes):
        """
        Called when message received (if no specific callback registered).
        
        Override this in subclasses to handle messages.
        
        Args:
            topic: Message topic
            payload: Message payload (bytes)
        """
        pass
    
    def is_connected(self) -> bool:
        """Check if client is connected to broker."""
        return self.connected

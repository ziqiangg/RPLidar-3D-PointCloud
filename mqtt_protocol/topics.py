"""
MQTT Topic definitions for RPLidar communication protocol.
"""


class Topics:
    """
    Centralized topic definitions for MQTT communication.
    
    Topic Structure:
    ----------------
    rplidar/
    ├── commands/scan      # Laptop → RPi: Start scan request
    ├── commands/stop      # Laptop → RPi: Stop scan request
    ├── commands/step      # Laptop → RPi: Allow one 3D step
    ├── status/<scan_id>   # RPi → Laptop: Final scan status
    └── data/<scan_id>     # RPi → Laptop: Point cloud data chunks
    """
    
    # Base topic prefix
    BASE = "rplidar"
    
    # Command topics (Laptop → Raspberry Pi)
    COMMAND_SCAN = f"{BASE}/commands/scan"
    COMMAND_STOP = f"{BASE}/commands/stop"
    COMMAND_STEP = f"{BASE}/commands/step"
    
    # Status topics (Raspberry Pi → Laptop)
    STATUS_PREFIX = f"{BASE}/status"
    
    # Data topics (Raspberry Pi → Laptop)
    DATA_PREFIX = f"{BASE}/data"
    
    @classmethod
    def status_topic(cls, scan_id: str) -> str:
        """Get status topic for specific scan."""
        return f"{cls.STATUS_PREFIX}/{scan_id}"
    
    @classmethod
    def data_topic(cls, scan_id: str) -> str:
        """Get data topic for specific scan."""
        return f"{cls.DATA_PREFIX}/{scan_id}"
    
    @classmethod
    def all_status(cls) -> str:
        """Get wildcard topic for all status messages."""
        return f"{cls.STATUS_PREFIX}/#"
    
    @classmethod
    def all_data(cls) -> str:
        """Get wildcard topic for all data messages."""
        return f"{cls.DATA_PREFIX}/#"
    
    @classmethod
    def all_commands(cls) -> str:
        """Get wildcard topic for all commands."""
        return f"{cls.BASE}/commands/#"

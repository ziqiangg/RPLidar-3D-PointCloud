"""
Message schemas for MQTT communication.

All messages are serialized as JSON for transmission.
"""

import json
import base64
import os
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any
from enum import Enum
from datetime import datetime


class StatusType(Enum):
    """Status types for scan operations."""
    STARTED = "started"
    COMPLETED = "completed"
    ERROR = "error"
    STOPPED = "stopped"


@dataclass
class ScanCommand:
    """
    Command to initiate a scan on Raspberry Pi.
    
    Published by: Laptop
    Subscribed by: Raspberry Pi
    Topic: rplidar/commands/scan
    """
    scan_id: str
    scan_type: str  # "2d" or "robust_3d"
    port: str = "auto"  # Serial port or "auto" for auto-detection
    
    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps(asdict(self))
    
    @classmethod
    def from_json(cls, json_str: str) -> 'ScanCommand':
        """Deserialize from JSON string."""
        data = json.loads(json_str)
        return cls(**data)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ScanCommand':
        """Create from dictionary."""
        return cls(**data)


@dataclass
class StopCommand:
    """
    Command to stop a running scan.
    
    Published by: Laptop
    Subscribed by: Raspberry Pi
    Topic: rplidar/commands/stop
    """
    scan_id: str
    
    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps(asdict(self))
    
    @classmethod
    def from_json(cls, json_str: str) -> 'StopCommand':
        """Deserialize from JSON string."""
        data = json.loads(json_str)
        return cls(**data)


@dataclass
class StepCommand:
    """
    Command to allow one step in a running 3D scan.

    Published by: Laptop
    Subscribed by: Raspberry Pi
    Topic: rplidar/commands/step
    """
    scan_id: str

    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, json_str: str) -> 'StepCommand':
        """Deserialize from JSON string."""
        data = json.loads(json_str)
        return cls(**data)


@dataclass
class ScanStatus:
    """
    Status message for scan lifecycle and progress.
    
    Published by: Raspberry Pi
    Subscribed by: Laptop
    Topic: rplidar/status/<scan_id>
    """
    scan_id: str
    status: str  # "started", "completed", "error", "stopped"
    message: str
    timestamp: str
    point_count: Optional[int] = None
    error_details: Optional[str] = None
    
    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps(asdict(self))
    
    @classmethod
    def from_json(cls, json_str: str) -> 'ScanStatus':
        """Deserialize from JSON string."""
        data = json.loads(json_str)
        return cls(**data)
    
    @classmethod
    def create_completed(cls, scan_id: str, point_count: int, message: str = "Scan completed successfully") -> 'ScanStatus':
        """Create a completed status message."""
        return cls(
            scan_id=scan_id,
            status=StatusType.COMPLETED.value,
            message=message,
            point_count=point_count,
            timestamp=datetime.now().isoformat()
        )
    
    @classmethod
    def create_error(cls, scan_id: str, error_message: str, error_details: str = None) -> 'ScanStatus':
        """Create an error status message."""
        return cls(
            scan_id=scan_id,
            status=StatusType.ERROR.value,
            message=error_message,
            error_details=error_details,
            timestamp=datetime.now().isoformat()
        )

    @classmethod
    def create_started(
        cls,
        scan_id: str,
        message: str = "Scan started",
        point_count: Optional[int] = None
    ) -> 'ScanStatus':
        """Create a started/progress status message."""
        return cls(
            scan_id=scan_id,
            status=StatusType.STARTED.value,
            message=message,
            point_count=point_count,
            timestamp=datetime.now().isoformat()
        )
    
    @classmethod
    def create_stopped(cls, scan_id: str, message: str = "Scan stopped by user") -> 'ScanStatus':
        """Create a stopped status message."""
        return cls(
            scan_id=scan_id,
            status=StatusType.STOPPED.value,
            message=message,
            timestamp=datetime.now().isoformat()
        )


@dataclass
class DataMessage:
    """
    Point cloud data message (chunked for large files).
    
    Published by: Raspberry Pi
    Subscribed by: Laptop
    Topic: rplidar/data/<scan_id>
    """
    scan_id: str
    format: str  # e.g. "csv", "ply", "jpg", "png"
    chunk_index: int
    total_chunks: int
    data: str  # Base64 encoded data chunk
    metadata: Optional[Dict[str, Any]] = None
    
    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps(asdict(self))
    
    @classmethod
    def from_json(cls, json_str: str) -> 'DataMessage':
        """Deserialize from JSON string."""
        data = json.loads(json_str)
        return cls(**data)
    
    @classmethod
    def create_from_file(
        cls,
        scan_id: str,
        file_path: str,
        file_format: str,
        chunk_size: int = 256 * 1024,
        metadata_extra: Optional[Dict[str, Any]] = None,
    ) -> list['DataMessage']:
        """
        Create multiple DataMessage chunks from a file.
        
        Args:
            scan_id: Unique scan identifier
            file_path: Path to file to chunk
            file_format: File format label ("csv", "ply", "jpg", etc)
            chunk_size: Size of each chunk in bytes (default 256KB)
            metadata_extra: Optional metadata to merge into first chunk
        
        Returns:
            List of DataMessage objects
        """
        messages = []
        
        with open(file_path, 'rb') as f:
            file_data = f.read()
        
        file_size = len(file_data)
        total_chunks = (file_size + chunk_size - 1) // chunk_size
        
        # Extract filename using os.path.basename for cross-platform compatibility
        filename = os.path.basename(file_path)
        
        metadata = {
            'file_size': file_size,
            'filename': filename
        }
        if metadata_extra:
            metadata.update(metadata_extra)
        
        for i in range(total_chunks):
            start = i * chunk_size
            end = min(start + chunk_size, file_size)
            chunk_data = file_data[start:end]
            
            # Encode chunk as base64
            encoded_chunk = base64.b64encode(chunk_data).decode('utf-8')
            
            msg = cls(
                scan_id=scan_id,
                format=file_format,
                chunk_index=i,
                total_chunks=total_chunks,
                data=encoded_chunk,
                metadata=metadata if i == 0 else None  # Only include metadata in first chunk
            )
            messages.append(msg)
        
        return messages
    
    def decode_data(self) -> bytes:
        """Decode base64 data chunk to bytes."""
        return base64.b64decode(self.data)


class MessageDecoder:
    """Helper class for decoding received MQTT messages."""
    
    @staticmethod
    def decode_payload(payload: bytes) -> Dict[str, Any]:
        """Decode MQTT payload to dictionary."""
        return json.loads(payload.decode('utf-8'))
    
    @staticmethod
    def is_scan_command(topic: str) -> bool:
        """Check if topic is a scan command."""
        from .topics import Topics
        return topic == Topics.COMMAND_SCAN
    
    @staticmethod
    def is_stop_command(topic: str) -> bool:
        """Check if topic is a stop command."""
        from .topics import Topics
        return topic == Topics.COMMAND_STOP

    @staticmethod
    def is_step_command(topic: str) -> bool:
        """Check if topic is a step command."""
        from .topics import Topics
        return topic == Topics.COMMAND_STEP
    
    @staticmethod
    def is_status_message(topic: str) -> bool:
        """Check if topic is a status message."""
        from .topics import Topics
        return topic.startswith(Topics.STATUS_PREFIX)
    
    @staticmethod
    def is_data_message(topic: str) -> bool:
        """Check if topic is a data message."""
        from .topics import Topics
        return topic.startswith(Topics.DATA_PREFIX)
    
    @staticmethod
    def extract_scan_id(topic: str) -> Optional[str]:
        """Extract scan_id from status or data topic."""
        parts = topic.split('/')
        if len(parts) >= 3:
            return parts[-1]
        return None

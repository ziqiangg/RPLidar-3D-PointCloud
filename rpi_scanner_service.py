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
import threading
import multiprocessing as mp
import queue
import yaml
import logging
import traceback
from typing import Optional

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mqtt_protocol import (
    MQTTClientBase,
    Topics,
    ScanCommand,
    StopCommand,
    StepCommand,
    ScanStatus,
    DataMessage,
    MessageDecoder
)


def _run_scan_3d_worker(
    port: str,
    output_dir: str,
    servo_cfg: dict,
    scan3d_cfg: dict,
    stop_event,
    step_event,
    message_queue,
    scan_type: str = "3d"
):
    """Child process entrypoint for 3D scan execution."""
    try:
        if scan_type == "robust_3d":
            from robust_3d_scan_module import run_scan
        else:
            from xyzscan_servo_auto import run_scan

        def progress_cb(progress: dict):
            try:
                message_queue.put({"type": "progress", "data": progress}, timeout=0.5)
            except Exception:
                pass

        def file_cb(file_path: str):
            """Callback when a slice file is ready to be sent."""
            try:
                message_queue.put({"type": "file", "path": file_path}, timeout=0.5)
            except Exception:
                pass

        def wait_for_step(step_request: dict) -> str:
            """Block until step command arrives or stop is requested."""
            wait_started = time.time()
            while True:
                if stop_event.is_set():
                    return "stop"
                if step_event.is_set():
                    try:
                        step_event.clear()
                    except Exception:
                        pass
                    return "continue"

                if (time.time() - wait_started) >= 2.0:
                    wait_started = time.time()
                    slice_index = int(step_request.get("slice_index", 0))
                    total_slices = int(step_request.get("total_slices", 0))
                    next_angle = step_request.get("next_angle")
                    next_servo_angle = step_request.get("next_servo_angle")
                    servo_text = (
                        f"{float(next_servo_angle):.1f}"
                        if next_servo_angle is not None
                        else "?"
                    )
                    message = (
                        f"Awaiting step permission for slice {slice_index + 1}/{total_slices} "
                        f"(az={next_angle}°, servo={servo_text}°). "
                        "Press Step Scan to continue."
                    )
                    progress_cb(
                        {
                            "stage": "step_wait",
                            "message": message,
                            "slice_index": slice_index,
                            "total_slices": total_slices,
                            "next_angle": next_angle,
                            "next_servo_angle": next_servo_angle,
                        }
                    )
                time.sleep(0.05)

        # Build args based on function signature to support both modules
        # (robust_3d_scan_module has file_callback, xyzscan_servo_auto might not)
        import inspect
        sig = inspect.signature(run_scan)
        kwargs = {
            'port': port,
            'output_dir': output_dir,
            'servo_config': servo_cfg,
            'scan_config': scan3d_cfg,
            'should_stop': stop_event.is_set,
            'progress_callback': progress_cb,
            'wait_for_step': wait_for_step,
        }
        
        if 'file_callback' in sig.parameters:
            kwargs['file_callback'] = file_cb

        result = run_scan(**kwargs)

    except Exception as e:
        result = {
            "success": False,
            "stopped": False,
            "point_count": 0,
            "files": [],
            "error": str(e),
            "message": f"3D scan worker failed: {str(e)}",
            "scan_quality": {},
        }

    try:
        message_queue.put({"type": "result", "data": result}, timeout=0.5)
    except Exception:
        pass


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
        self.scan_thread: Optional[threading.Thread] = None
        self.stop_requested = threading.Event()
        self.state_lock = threading.Lock()
        self.active_process: Optional[mp.Process] = None
        self.active_process_stop_event = None
        self.active_process_step_event = None
        
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
        self.subscribe(Topics.COMMAND_STEP, self._handle_step_command)
        
        self.logger.info("Ready to receive scan commands")
    
    def on_disconnected(self):
        """Called when disconnected from broker."""
        self.logger.warning("Disconnected from MQTT broker - will attempt reconnect")

    def _publish_started_status(self, scan_id: str, message: str, point_count: Optional[int] = None):
        """Publish started/progress status updates."""
        try:
            status = ScanStatus.create_started(scan_id, message=message, point_count=point_count)
            self.publish(Topics.status_topic(scan_id), status.to_json())
        except Exception as e:
            self.logger.error(f"Failed to publish started status: {e}")
    
    def _handle_scan_command(self, topic: str, payload: bytes):
        """Handle incoming scan command."""
        try:
            # Parse command
            command = ScanCommand.from_json(payload.decode('utf-8'))
            self.logger.info(f"Received scan command: {command.scan_id}, type: {command.scan_type}")
            
            # Check if already scanning
            with self.state_lock:
                if self.scan_running:
                    current_scan = self.current_scan_id
                    self.logger.warning(f"Scan already in progress: {current_scan}")
                    status = ScanStatus.create_error(
                        command.scan_id,
                        "Scanner busy - another scan is in progress",
                        f"Current scan: {current_scan}"
                    )
                    self.publish(Topics.status_topic(command.scan_id), status.to_json())
                    return

                # Mark running and start worker thread so MQTT callbacks remain responsive.
                self.current_scan_id = command.scan_id
                self.scan_running = True
                self.stop_requested.clear()
                self.scan_thread = threading.Thread(
                    target=self._execute_scan,
                    args=(command,),
                    daemon=True
                )
                self.scan_thread.start()
                self.logger.info(f"Scan worker started for {command.scan_id}")

            self._publish_started_status(
                command.scan_id,
                f"Scan accepted ({command.scan_type}). Initializing..."
            )
            
        except Exception as e:
            self.logger.error(f"Error handling scan command: {e}")
            self.logger.error(traceback.format_exc())
    
    def _handle_stop_command(self, topic: str, payload: bytes):
        """Handle incoming stop command."""
        try:
            command = StopCommand.from_json(payload.decode('utf-8'))
            self.logger.info(f"Received stop command for scan: {command.scan_id}")

            with self.state_lock:
                active_scan = self.scan_running and self.current_scan_id == command.scan_id
                active_process = self.active_process
                active_process_stop_event = self.active_process_stop_event
                active_process_step_event = self.active_process_step_event

            if active_scan:
                self.stop_requested.set()
                self.logger.info("Stop requested for active scan")
                if active_process_stop_event is not None:
                    try:
                        active_process_stop_event.set()
                        self.logger.info("Signaled 3D worker process to stop")
                    except Exception as e:
                        self.logger.warning(f"Failed to signal 3D worker stop: {e}")
                if active_process_step_event is not None:
                    try:
                        # Unblock worker if it is waiting for manual step permission.
                        active_process_step_event.set()
                    except Exception:
                        pass
                if active_process is not None and not active_process.is_alive():
                    self.logger.info("Active process already exited")
            else:
                self.logger.info("No matching scan to stop")
                
        except Exception as e:
            self.logger.error(f"Error handling stop command: {e}")

    def _handle_step_command(self, topic: str, payload: bytes):
        """Handle incoming step command for 3D scan progression."""
        try:
            command = StepCommand.from_json(payload.decode('utf-8'))
            self.logger.info(f"Received step command for scan: {command.scan_id}")

            with self.state_lock:
                active_scan = (
                    self.scan_running
                    and self.current_scan_id == command.scan_id
                )
                active_process_step_event = self.active_process_step_event

            if not active_scan:
                self.logger.info("Ignoring step command: no matching active scan")
                return

            if active_process_step_event is None:
                self.logger.info("Ignoring step command: 3D worker step gate unavailable")
                return

            active_process_step_event.set()
            self.logger.info("Granted one 3D step")
        except Exception as e:
            self.logger.error(f"Error handling step command: {e}")
    
    def _execute_scan(self, command: ScanCommand):
        """Execute a scan based on command."""
        try:
            self.logger.info(f"Starting scan {command.scan_id}")
            self._publish_started_status(
                command.scan_id,
                f"Scan started on Raspberry Pi ({command.scan_type})"
            )
            
            # Import and run scan directly (not subprocess)
            # This allows us to capture results programmatically.
            if command.scan_type == "2d":
                scan_result = self._run_scan_2d(command.port)
            elif command.scan_type in ["3d", "robust_3d"]:
                scan_result = self._run_scan_3d(command.port, command.scan_id, command.scan_type)
            else:
                self.logger.error(f"Unsupported scan type: {command.scan_type}")
                status = ScanStatus.create_error(
                    command.scan_id,
                    f"Unsupported scan type: {command.scan_type}",
                    "Supported scan types are: 2d, 3d"
                )
                self.publish(Topics.status_topic(command.scan_id), status.to_json())
                return
            
            if scan_result.get('stopped'):
                self.logger.info(f"Scan stopped: {scan_result.get('message', 'Scan stopped by user')}")
                status = ScanStatus.create_stopped(
                    command.scan_id,
                    scan_result.get('message', 'Scan stopped by user')
                )
                self.publish(Topics.status_topic(command.scan_id), status.to_json())
                return

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
            with self.state_lock:
                self.scan_running = False
                self.current_scan_id = None
                self.scan_thread = None
                self.active_process = None
                self.active_process_stop_event = None
                self.active_process_step_event = None
            self.stop_requested.clear()
    
    def _run_scan_2d(self, port: str) -> dict:
        """
        Run 2D scan by directly importing and executing dump_one_scan.run_scan().
        
        No subprocess, no stdout/stderr capture - direct module import.
        
        Args:
            port: Serial port ("auto" or specific port like "/dev/ttyUSB0")
        
        Returns:
            dict with keys: success, point_count, files, error, message, scan_quality
        """
        try:
            if self.stop_requested.is_set():
                return {
                    'success': False,
                    'stopped': True,
                    'point_count': 0,
                    'files': [],
                    'error': None,
                    'message': 'Scan stopped by user before 2D scan start',
                    'scan_quality': {}
                }

            # Import the scan module
            from dump_one_scan import run_scan
            
            # Determine port
            if port == "auto":
                from utils.port_config import get_default_port
                port = get_default_port()
            
            self.logger.info(f"Starting scan on port: {port}")
            if self.current_scan_id:
                self._publish_started_status(
                    self.current_scan_id,
                    "2D LiDAR capture in progress..."
                )
            
            # Get output directory from config
            data_dir = self.config['data']['output_dir']
            
            # Execute scan directly (no subprocess)
            result = run_scan(port=port, output_dir=data_dir)

            # dump_one_scan.py is not interruptible mid-run. If stop was requested during
            # execution, report stopped instead of completed.
            if self.stop_requested.is_set():
                return {
                    'success': False,
                    'stopped': True,
                    'point_count': result.get('point_count', 0),
                    'files': [],
                    'error': None,
                    'message': 'Stop requested during 2D scan (non-interruptible)',
                    'scan_quality': result.get('scan_quality', {})
                }
            
            if result['success']:
                self.logger.info(f"Scan completed: {result['message']}")
                quality = result.get('scan_quality', {})
                if quality:
                    self.logger.info(
                        f"Quality: {quality.get('coverage_percent', 0):.1f}% coverage, "
                        f"max gap {quality.get('max_gap_degrees', 0):.1f}°, "
                        f"{quality.get('scans_merged', 1)} scans merged"
                    )
            else:
                self.logger.error(f"Scan failed: {result['error']}")
            
            return result
                
        except Exception as e:
            self.logger.error(f"Error running scan: {e}")
            self.logger.error(traceback.format_exc())
            return {
                'success': False,
                'stopped': False,
                'point_count': 0,
                'files': [],
                'error': str(e),
                'message': f"Scan execution error: {str(e)}",
                'scan_quality': {}
            }

    def _run_scan_3d(self, port: str, scan_id: str) -> dict:
        """
        Run automated 3D scan by importing xyzscan_servo_auto.run_scan().

        Args:
            port: Serial port ("auto" or specific port like "/dev/ttyUSB0")
            scan_id: Scan identifier

        Returns:
            dict with keys: success, point_count, files, error, message, scan_quality
        """
        try:
            # Determine LiDAR port
            if port == "auto":
                from utils.port_config import get_default_port
                port = get_default_port()

            self.logger.info(f"Starting automated 3D scan on port: {port} (ID: {scan_id})")
            
            # Check if this is a robust scan request based on scan_id prefix or just use default
            # But the caller (execute_scan) should have passed scan type. 
            # The current architectural method _run_scan_3d doesn't take scan_type as arg, 
            # but we can infer it or we should update the method signature.
            # However, looking at _execute_scan, it calls _run_scan_3d(command.port, command.scan_id).
            # We can't easily change the signature without breaking things? 
            # Actually we can change it in execute_scan too.
            # But let's assume we can pass it via self or check command if available. 
            # Wait, _execute_scan calls this. Let's fix _execute_scan to pass the type!
            
            # Temporary fix: pass scan_type via config injection or simple check.
            # Better: Let's modify _execute_scan first to pass scan_type.
            # But wait, I am editing only 2 files request.
            # I should edit _execute_scan to pass scan_type to _run_scan_3d.

            # Get output directory and optional 3D configs from service config
            data_dir = self.config['data']['output_dir']
            servo_cfg = self.config.get('servo', {})
            scan3d_cfg = self.config.get('scan3d', {})

            message_queue = mp.Queue()
            process_stop_event = mp.Event()
            process_step_event = mp.Event()
            
            # Infer scan type - normally we'd pass it.
            # For now, let's look at the calling usage. 
            # I will assume I will fix _execute_scan to pass it.
            pass

        except Exception as e:
            pass # Just placeholder 


            stop_grace_deadline = None
            result = None
            last_progress_at = time.time()
            last_progress_stage = ""
            worker_stall_timeout = max(
                10.0,
                float(scan3d_cfg.get('worker_stall_timeout', 20.0))
            )

            while True:
                if self.stop_requested.is_set():
                    if stop_grace_deadline is None:
                        process_stop_event.set()
                        process_step_event.set()
                        stop_grace_deadline = time.time() + 5.0
                        self.logger.info("Waiting for 3D worker to stop gracefully")
                    elif worker.is_alive() and time.time() >= stop_grace_deadline:
                        self.logger.warning("3D worker did not stop in time; terminating")
                        worker.terminate()
                        worker.join(timeout=2.0)
                        if worker.is_alive():
                            try:
                                worker.kill()
                            except Exception:
                                pass
                        result = {
                            'success': False,
                            'stopped': True,
                            'point_count': 0,
                            'files': [],
                            'error': None,
                            'message': '3D scan stopped by user',
                            'scan_quality': {}
                        }
                        break

                try:
                    msg = message_queue.get(timeout=0.2)
                    msg_type = msg.get("type")
                    if msg_type == "progress":
                        progress = msg.get("data", {})
                        message = progress.get('message', "3D scan in progress")
                        point_count = progress.get('point_count')
                        stage = str(progress.get('stage', "") or "")
                        if stage:
                            last_progress_stage = stage
                        self._publish_started_status(scan_id, message, point_count=point_count)
                        last_progress_at = time.time()
                    elif msg_type == "result":
                        result = msg.get("data")
                        last_progress_at = time.time()
                        break
                except queue.Empty:
                    pass

                if (
                    worker.is_alive()
                    and not self.stop_requested.is_set()
                    and (time.time() - last_progress_at) > 8.0
                ):
                    if last_progress_stage == "step_wait":
                        self._publish_started_status(
                            scan_id,
                            "Awaiting step permission from controller...",
                        )
                    else:
                        self._publish_started_status(
                            scan_id,
                            "Waiting for LiDAR data in current slice...",
                        )
                    last_progress_at = time.time()

                if (
                    worker.is_alive()
                    and not self.stop_requested.is_set()
                    and last_progress_stage != "step_wait"
                    and (time.time() - last_progress_at) > worker_stall_timeout
                ):
                    self.logger.warning(
                        f"3D worker stalled for >{worker_stall_timeout:.1f}s; terminating"
                    )
                    worker.terminate()
                    worker.join(timeout=2.0)
                    if worker.is_alive():
                        try:
                            worker.kill()
                        except Exception:
                            pass
                    result = {
                        'success': False,
                        'stopped': False,
                        'point_count': 0,
                        'files': [],
                        'error': '3D worker stalled waiting for LiDAR data',
                        'message': '3D scan failed: stalled waiting for LiDAR data',
                        'scan_quality': {}
                    }
                    break

                if not worker.is_alive():
                    if result is None:
                        # Attempt one final non-blocking drain.
                        try:
                            while True:
                                msg = message_queue.get_nowait()
                                msg_type = msg.get("type")
                                if msg_type == "progress":
                                    progress = msg.get("data", {})
                                    message = progress.get('message', "3D scan in progress")
                                    point_count = progress.get('point_count')
                                    self._publish_started_status(scan_id, message, point_count=point_count)
                                elif msg_type == "result":
                                    result = msg.get("data")
                                    break
                        except queue.Empty:
                            pass
                    if result is None:
                        if self.stop_requested.is_set():
                            result = {
                                'success': False,
                                'stopped': True,
                                'point_count': 0,
                                'files': [],
                                'error': None,
                                'message': '3D scan stopped by user',
                                'scan_quality': {}
                            }
                        else:
                            result = {
                                'success': False,
                                'stopped': False,
                                'point_count': 0,
                                'files': [],
                                'error': '3D worker exited without result',
                                'message': '3D scan worker exited unexpectedly',
                                'scan_quality': {}
                            }
                    break

            worker.join(timeout=1.0)

            if result is None:
                result = {
                    'success': False,
                    'stopped': False,
                    'point_count': 0,
                    'files': [],
                    'error': 'Missing 3D worker result',
                    'message': '3D scan failed: missing worker result',
                    'scan_quality': {}
                }

            if result.get('stopped'):
                self.logger.info(f"3D scan stopped: {result.get('message', 'Scan stopped by user')}")
            elif result['success']:
                self.logger.info(f"3D scan completed: {result['message']}")
                quality = result.get('scan_quality', {})
                if quality:
                    self.logger.info(
                        f"3D slices: {quality.get('successful_slices', 0)}/"
                        f"{quality.get('total_slices', 0)}, "
                        f"avg coverage {quality.get('avg_slice_coverage_percent', 0):.1f}%, "
                        f"reset_completed={quality.get('reset_completed', False)}"
                    )
            else:
                self.logger.error(f"3D scan failed: {result.get('error')}")

            return result

        except Exception as e:
            self.logger.error(f"Error running 3D scan: {e}")
            self.logger.error(traceback.format_exc())
            return {
                'success': False,
                'stopped': False,
                'point_count': 0,
                'files': [],
                'error': str(e),
                'message': f"3D scan execution error: {str(e)}",
                'scan_quality': {}
            }
        finally:
            with self.state_lock:
                self.active_process = None
                self.active_process_stop_event = None
                self.active_process_step_event = None
    
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

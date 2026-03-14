"""
Panorama scan module.

Captures one webcam frame per servo step and saves/transfers resulting images.
"""

import os
import time
import platform
from typing import Optional, Callable, Tuple, List

from utils.port_config import get_default_servo_port
from robust_3d_scan_module import PicoServoController, _is_io_error

try:
    import cv2
except ImportError:
    cv2 = None


DEFAULTS = {
    'start_deg': 0.0,
    'end_deg': 180.0,
    'step_deg': 30.0,
    'home_settle_time': 1.5,
    'step_settle_time': 1.0,
    'step_dwell_time': 0.35,
    'camera_init_time': 1.5,
    'camera_warmup_frames': 12,
    'camera_capture_retries': 2,
    'camera_probe_count': 6,
    'camera_index': 'auto',
    'image_format': 'jpg',
    'image_quality': 90,
    'image_width': 1280,
    'image_height': 720,
    'servo_io_retries': 2,
    'servo_reconnect_delay': 0.5,
}


def _choose_backends() -> List[Tuple[int, str]]:
    """Return backend candidates in platform preference order."""
    if cv2 is None:
        return []

    os_name = platform.system()
    any_backend = getattr(cv2, 'CAP_ANY', 0)

    if os_name == 'Windows':
        return [
            (getattr(cv2, 'CAP_DSHOW', any_backend), 'CAP_DSHOW'),
            (getattr(cv2, 'CAP_MSMF', any_backend), 'CAP_MSMF'),
            (any_backend, 'CAP_ANY'),
        ]
    if os_name == 'Darwin':
        return [
            (getattr(cv2, 'CAP_AVFOUNDATION', any_backend), 'CAP_AVFOUNDATION'),
            (any_backend, 'CAP_ANY'),
        ]
    return [
        (getattr(cv2, 'CAP_V4L2', any_backend), 'CAP_V4L2'),
        (any_backend, 'CAP_ANY'),
    ]


def _open_camera(index: int, backend: int):
    """Open camera with optional backend argument."""
    try:
        if backend == getattr(cv2, 'CAP_ANY', 0):
            return cv2.VideoCapture(index)
        return cv2.VideoCapture(index, backend)
    except TypeError:
        return cv2.VideoCapture(index)


def _discover_camera(
    preferred_index,
    probe_count: int,
    init_s: float,
    warmup_frames: int,
    width: int,
    height: int,
) -> Tuple[object, int, str]:
    """Discover a usable webcam index and return opened capture."""
    if cv2 is None:
        raise RuntimeError('opencv-python-headless is not installed on RP5 environment')

    indices = []
    if isinstance(preferred_index, int):
        indices.append(preferred_index)
    elif isinstance(preferred_index, str) and preferred_index.strip().lower() != 'auto':
        try:
            indices.append(int(preferred_index.strip()))
        except ValueError:
            pass

    for i in range(max(1, probe_count)):
        if i not in indices:
            indices.append(i)

    backends = _choose_backends()
    last_error = None

    for idx in indices:
        for backend, backend_name in backends:
            cap = _open_camera(idx, backend)
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()
                continue

            if width > 0:
                cap.set(getattr(cv2, 'CAP_PROP_FRAME_WIDTH', 3), width)
            if height > 0:
                cap.set(getattr(cv2, 'CAP_PROP_FRAME_HEIGHT', 4), height)

            time.sleep(max(0.0, init_s))
            ok = False
            frame = None
            for _ in range(max(1, warmup_frames)):
                ok, frame = cap.read()
                if ok and frame is not None:
                    break
                time.sleep(0.05)

            if ok and frame is not None:
                return cap, idx, backend_name

            last_error = RuntimeError(f'Camera {idx} opened but no valid frame ({backend_name})')
            cap.release()

    if last_error:
        raise last_error
    raise RuntimeError('No webcam device discovered')


def _build_angles(start_deg: float, end_deg: float, step_deg: float) -> List[float]:
    """Build angle sequence [start, start+step, ... < end]."""
    angles = []
    current = start_deg
    while current < end_deg:
        angles.append(round(current, 2))
        current += step_deg
    return angles


def run_scan(
    output_dir: str = 'data',
    servo_config: Optional[dict] = None,
    panorama_config: Optional[dict] = None,
    should_stop: Optional[Callable] = None,
    progress_callback: Optional[Callable] = None,
    file_callback: Optional[Callable] = None,
) -> dict:
    """
    Execute panorama capture (servo step + webcam frame per step).

    Returns a result dict compatible with scanner service expectations.
    """
    if should_stop is None:
        should_stop = lambda: False
    if progress_callback is None:
        progress_callback = lambda x: None
    if file_callback is None:
        file_callback = lambda x: None

    servo_config = servo_config or {}
    panorama_config = panorama_config or {}

    start_deg = float(panorama_config.get('start_deg', DEFAULTS['start_deg']))
    end_deg = float(panorama_config.get('end_deg', DEFAULTS['end_deg']))
    step_deg = float(panorama_config.get('step_deg', DEFAULTS['step_deg']))
    home_settle_s = float(panorama_config.get('home_settle_time', DEFAULTS['home_settle_time']))
    step_settle_s = float(panorama_config.get('step_settle_time', DEFAULTS['step_settle_time']))
    step_dwell_s = float(panorama_config.get('step_dwell_time', DEFAULTS['step_dwell_time']))

    camera_init_s = float(panorama_config.get('camera_init_time', DEFAULTS['camera_init_time']))
    camera_warmup_frames = int(panorama_config.get('camera_warmup_frames', DEFAULTS['camera_warmup_frames']))
    camera_capture_retries = int(panorama_config.get('camera_capture_retries', DEFAULTS['camera_capture_retries']))
    camera_probe_count = int(panorama_config.get('camera_probe_count', DEFAULTS['camera_probe_count']))
    camera_index = panorama_config.get('camera_index', DEFAULTS['camera_index'])
    image_fmt = str(panorama_config.get('image_format', DEFAULTS['image_format'])).strip().lower()
    if image_fmt not in ('jpg', 'jpeg', 'png'):
        image_fmt = 'jpg'

    image_quality = int(panorama_config.get('image_quality', DEFAULTS['image_quality']))
    image_width = int(panorama_config.get('image_width', DEFAULTS['image_width']))
    image_height = int(panorama_config.get('image_height', DEFAULTS['image_height']))

    servo_baud = int(servo_config.get('baudrate', 115200))
    servo_timeout = float(servo_config.get('timeout', 8.0))
    servo_port = get_default_servo_port(servo_config.get('serial_port', 'auto'))
    servo_io_retries = int(panorama_config.get('servo_io_retries', DEFAULTS['servo_io_retries']))
    servo_reconnect_delay = float(panorama_config.get('servo_reconnect_delay', DEFAULTS['servo_reconnect_delay']))

    if step_deg <= 0:
        return {
            'success': False,
            'stopped': False,
            'point_count': 0,
            'files': [],
            'error': 'panorama.step_deg must be > 0',
            'message': 'Invalid panorama configuration',
            'scan_quality': {}
        }
    if end_deg <= start_deg:
        return {
            'success': False,
            'stopped': False,
            'point_count': 0,
            'files': [],
            'error': 'panorama.end_deg must be > panorama.start_deg',
            'message': 'Invalid panorama configuration',
            'scan_quality': {}
        }

    angles = _build_angles(start_deg, end_deg, step_deg)
    if not angles:
        return {
            'success': False,
            'stopped': False,
            'point_count': 0,
            'files': [],
            'error': 'No panorama angles generated',
            'message': 'Invalid panorama range/step configuration',
            'scan_quality': {}
        }

    images_dir = os.path.join(output_dir, 'images')
    os.makedirs(images_dir, exist_ok=True)

    generated_files = []
    servo = None
    cap = None

    def _sleep_with_stop(duration_s: float) -> bool:
        end_at = time.time() + max(0.0, duration_s)
        while time.time() < end_at:
            if should_stop():
                return False
            time.sleep(0.05)
        return True

    def _connect_servo_controller() -> PicoServoController:
        last_exc = None
        for attempt in range(servo_io_retries + 1):
            resolved_port = get_default_servo_port(servo_config.get('serial_port', 'auto'))
            try:
                s = PicoServoController(resolved_port, baudrate=servo_baud, timeout=servo_timeout)
                s.set_policy('PANORAMA')
                return s
            except Exception as exc:
                last_exc = exc
                if attempt >= servo_io_retries:
                    break
                if not _sleep_with_stop(servo_reconnect_delay):
                    break
        if last_exc:
            raise last_exc
        raise RuntimeError('Unable to connect servo controller')

    def _move_with_recovery(target_angle: float):
        nonlocal servo
        for attempt in range(servo_io_retries + 1):
            if servo is None:
                servo = _connect_servo_controller()
            try:
                servo.set_angle(target_angle)
                return
            except Exception as exc:
                is_io = _is_io_error(exc) or 'SerialException' in str(type(exc))
                if (not is_io) or attempt >= servo_io_retries:
                    raise
                try:
                    servo.detach()
                except Exception:
                    pass
                servo = None
                if not _sleep_with_stop(servo_reconnect_delay):
                    raise RuntimeError('Stopped during servo recovery')

    try:
        progress_callback({'stage': 'init', 'message': 'Initializing panorama servo and webcam...'})

        servo = _connect_servo_controller()
        progress_callback({'stage': 'init', 'message': f'Homing servo to {start_deg:.1f}°'})
        _move_with_recovery(start_deg)
        if not _sleep_with_stop(home_settle_s):
            return {
                'success': False,
                'stopped': True,
                'point_count': 0,
                'files': generated_files,
                'error': None,
                'message': 'Stopped during homing',
                'scan_quality': {}
            }

        cap, cam_index, cam_backend = _discover_camera(
            preferred_index=camera_index,
            probe_count=max(1, camera_probe_count),
            init_s=max(0.0, camera_init_s),
            warmup_frames=max(1, camera_warmup_frames),
            width=max(0, image_width),
            height=max(0, image_height),
        )
        progress_callback({
            'stage': 'init',
            'message': f'Webcam ready on index {cam_index} ({cam_backend})',
        })

        for idx, angle in enumerate(angles):
            if should_stop():
                return {
                    'success': False,
                    'stopped': True,
                    'point_count': len(generated_files),
                    'files': generated_files,
                    'error': None,
                    'message': 'Panorama stopped by user',
                    'scan_quality': {}
                }

            if idx > 0:
                _move_with_recovery(angle)
                if not _sleep_with_stop(step_settle_s):
                    return {
                        'success': False,
                        'stopped': True,
                        'point_count': len(generated_files),
                        'files': generated_files,
                        'error': None,
                        'message': 'Stopped during panorama settle',
                        'scan_quality': {}
                    }

            if not _sleep_with_stop(step_dwell_s):
                return {
                    'success': False,
                    'stopped': True,
                    'point_count': len(generated_files),
                    'files': generated_files,
                    'error': None,
                    'message': 'Stopped during panorama dwell',
                    'scan_quality': {}
                }

            ok = False
            frame = None
            for _ in range(camera_capture_retries + 1):
                ok, frame = cap.read()
                if ok and frame is not None:
                    break
                time.sleep(0.05)

            if not ok or frame is None:
                raise RuntimeError(f'Camera capture failed at panorama step {idx + 1}')

            image_name = f'panorama_{idx:02d}.{image_fmt}'
            image_path = os.path.join(images_dir, image_name)

            if image_fmt in ('jpg', 'jpeg'):
                params = [int(getattr(cv2, 'IMWRITE_JPEG_QUALITY', 1)), max(1, min(100, image_quality))]
            else:
                params = []

            written = cv2.imwrite(image_path, frame, params)
            if not written:
                raise RuntimeError(f'Failed writing image: {image_path}')

            generated_files.append(image_path)
            file_callback(image_path)
            progress_callback({
                'stage': 'capturing',
                'message': f'Captured panorama image {idx + 1}/{len(angles)} at {angle:.1f}°',
                'point_count': len(generated_files),
            })

        return {
            'success': True,
            'stopped': False,
            'point_count': len(generated_files),
            'files': generated_files,
            'error': None,
            'message': f'Panorama capture completed ({len(generated_files)} images)',
            'scan_quality': {
                'camera_index': cam_index,
                'camera_backend': cam_backend,
                'images_captured': len(generated_files),
                'start_deg': start_deg,
                'end_deg': end_deg,
                'step_deg': step_deg,
            }
        }

    except Exception as e:
        return {
            'success': False,
            'stopped': False,
            'point_count': len(generated_files),
            'files': generated_files,
            'error': str(e),
            'message': f'Panorama scan failed: {e}',
            'scan_quality': {}
        }
    finally:
        try:
            if cap is not None:
                cap.release()
        except Exception:
            pass
        try:
            if servo is not None:
                servo.detach()
        except Exception:
            pass

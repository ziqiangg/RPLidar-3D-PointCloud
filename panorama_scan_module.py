"""
Panorama scan module.

Captures one webcam frame per servo step and saves/transfers resulting images.
"""

import os
import time
import platform
import glob
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
    'camera_step_retries': 3,
    'camera_step_retry_wait': 1.0,
    'camera_flush_frames': 6,
    'camera_post_move_wait': 0.20,
    'camera_reopen_retries': 2,
    'camera_reopen_wait': 0.5,
    'camera_probe_count': 12,
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


def _open_camera(source, backend: int):
    """Open camera by numeric index or device path with optional backend."""
    try:
        if backend == getattr(cv2, 'CAP_ANY', 0):
            return cv2.VideoCapture(source)
        return cv2.VideoCapture(source, backend)
    except TypeError:
        return cv2.VideoCapture(source)


def _build_camera_candidates(preferred_index, probe_count: int) -> List[object]:
    """Build ordered camera sources (paths and indexes) for discovery."""
    candidates = []
    seen = set()

    def _add_candidate(source):
        key = f"{type(source).__name__}:{source}"
        if key in seen:
            return
        seen.add(key)
        candidates.append(source)

    if isinstance(preferred_index, int):
        _add_candidate(preferred_index)
    elif isinstance(preferred_index, str):
        pref = preferred_index.strip()
        if pref and pref.lower() != 'auto':
            try:
                _add_candidate(int(pref))
            except ValueError:
                _add_candidate(pref)

    # Linux camera nodes are often non-deterministic by index; prefer stable by-id paths.
    if platform.system() == 'Linux':
        for path in sorted(glob.glob('/dev/v4l/by-id/*')):
            if os.path.exists(path):
                _add_candidate(path)

        video_nodes = sorted(
            glob.glob('/dev/video*'),
            key=lambda p: int(p.replace('/dev/video', '')) if p.replace('/dev/video', '').isdigit() else 10**9
        )
        for path in video_nodes:
            if os.path.exists(path):
                _add_candidate(path)

    for i in range(max(1, probe_count)):
        _add_candidate(i)

    return candidates


def _discover_camera(
    preferred_index,
    probe_count: int,
    init_s: float,
    warmup_frames: int,
    width: int,
    height: int,
) -> Tuple[object, object, str]:
    """Discover a usable webcam source and return opened capture."""
    if cv2 is None:
        raise RuntimeError('opencv-python-headless is not installed on RP5 environment')

    camera_sources = _build_camera_candidates(preferred_index, probe_count)

    backends = _choose_backends()
    last_error = None

    for source in camera_sources:
        for backend, backend_name in backends:
            cap = _open_camera(source, backend)
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
                return cap, source, backend_name

            last_error = RuntimeError(f'Camera {source} opened but no valid frame ({backend_name})')
            cap.release()

    if last_error:
        raise last_error
    raise RuntimeError('No webcam device discovered')


def _open_preferred_camera(
    preferred_source,
    init_s: float,
    warmup_frames: int,
    width: int,
    height: int,
) -> Tuple[object, object, str]:
    """Attempt reopening only the preferred source first for faster recovery."""
    backends = _choose_backends()
    last_error = None

    for backend, backend_name in backends:
        cap = _open_camera(preferred_source, backend)
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
                return cap, preferred_source, backend_name
            time.sleep(0.05)

        last_error = RuntimeError(
            f'Camera {preferred_source} reopened but no valid frame ({backend_name})'
        )
        cap.release()

    if last_error:
        raise last_error
    raise RuntimeError(f'Unable to reopen preferred camera source {preferred_source}')


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
    camera_step_retries = int(panorama_config.get('camera_step_retries', DEFAULTS['camera_step_retries']))
    camera_step_retry_wait = float(panorama_config.get('camera_step_retry_wait', DEFAULTS['camera_step_retry_wait']))
    camera_flush_frames = int(panorama_config.get('camera_flush_frames', DEFAULTS['camera_flush_frames']))
    camera_post_move_wait = float(panorama_config.get('camera_post_move_wait', DEFAULTS['camera_post_move_wait']))
    camera_reopen_retries = int(panorama_config.get('camera_reopen_retries', DEFAULTS['camera_reopen_retries']))
    camera_reopen_wait = float(panorama_config.get('camera_reopen_wait', DEFAULTS['camera_reopen_wait']))
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
    cam_index = None
    cam_backend = None

    # This tracks whether we should perform extra post-move warmup before capture.
    moved_since_last_capture = False

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

    def _capture_fresh_frame() -> Tuple[bool, object]:
        """Capture a fresh frame by flushing buffered frames first."""
        if cap is None or (hasattr(cap, 'isOpened') and not cap.isOpened()):
            return False, None

        for _ in range(max(0, camera_flush_frames)):
            try:
                cap.grab()
            except Exception:
                break

        ok = False
        frame = None
        for _ in range(camera_capture_retries + 1):
            ok, frame = cap.read()
            if ok and frame is not None:
                return ok, frame
            time.sleep(0.05)
        return ok, frame

    def _recover_camera() -> Tuple[object, object, str]:
        """Recover from camera disconnects by reopen then full rediscovery."""
        nonlocal cap, cam_index, cam_backend

        last_exc = None
        for attempt in range(camera_reopen_retries + 1):
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass
            cap = None

            try:
                if cam_index is not None:
                    cap, cam_index, cam_backend = _open_preferred_camera(
                        preferred_source=cam_index,
                        init_s=max(0.0, camera_init_s * 0.5),
                        warmup_frames=max(1, camera_warmup_frames),
                        width=max(0, image_width),
                        height=max(0, image_height),
                    )
                else:
                    raise RuntimeError('No previous camera index to reopen')
                return cap, cam_index, cam_backend
            except Exception as exc:
                last_exc = exc

            try:
                cap, cam_index, cam_backend = _discover_camera(
                    preferred_index=cam_index if cam_index is not None else camera_index,
                    probe_count=max(1, camera_probe_count),
                    init_s=max(0.0, camera_init_s),
                    warmup_frames=max(1, camera_warmup_frames),
                    width=max(0, image_width),
                    height=max(0, image_height),
                )
                return cap, cam_index, cam_backend
            except Exception as exc:
                last_exc = exc

            if attempt < camera_reopen_retries:
                if not _sleep_with_stop(max(0.0, camera_reopen_wait)):
                    raise RuntimeError('Stopped during camera recovery wait')

        if last_exc:
            raise RuntimeError(f'Camera recovery failed: {last_exc}')
        raise RuntimeError('Camera recovery failed')

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
                moved_since_last_capture = True
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

                if moved_since_last_capture and camera_post_move_wait > 0.0:
                    if not _sleep_with_stop(camera_post_move_wait):
                        return {
                            'success': False,
                            'stopped': True,
                            'point_count': len(generated_files),
                            'files': generated_files,
                            'error': None,
                            'message': 'Stopped during post-move camera wait',
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

            ok, frame = _capture_fresh_frame()

            recovery_error = None
            if not ok or frame is None:
                for recovery_attempt in range(max(0, camera_step_retries) + 1):
                    progress_callback({
                        'stage': 'capturing',
                        'message': (
                            f'Camera read failed at step {idx + 1}; attempting recovery '
                            f'({recovery_attempt + 1}/{max(0, camera_step_retries) + 1})...'
                        ),
                    })
                    try:
                        _recover_camera()
                        if moved_since_last_capture and camera_post_move_wait > 0.0:
                            if not _sleep_with_stop(camera_post_move_wait):
                                return {
                                    'success': False,
                                    'stopped': True,
                                    'point_count': len(generated_files),
                                    'files': generated_files,
                                    'error': None,
                                    'message': 'Stopped during post-recovery camera wait',
                                    'scan_quality': {}
                                }

                        ok, frame = _capture_fresh_frame()
                        if ok and frame is not None:
                            recovery_error = None
                            break
                        recovery_error = RuntimeError('Recovered camera but frame read still failed')
                    except Exception as exc:
                        recovery_error = exc

                    if recovery_attempt < max(0, camera_step_retries):
                        if not _sleep_with_stop(max(0.0, camera_step_retry_wait)):
                            return {
                                'success': False,
                                'stopped': True,
                                'point_count': len(generated_files),
                                'files': generated_files,
                                'error': None,
                                'message': 'Stopped during camera retry wait',
                                'scan_quality': {}
                            }

                if (not ok or frame is None) and recovery_error is not None:
                    raise RuntimeError(
                        f'Camera capture failed at panorama step {idx + 1} after recovery retries: {recovery_error}'
                    )

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
            moved_since_last_capture = False
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

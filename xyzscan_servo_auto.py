"""
Automated RPLidar 3D Scanner (Servo-driven azimuth sweep).

This script performs a full 3D capture by:
1. Moving servo from azimuth 0° to 180° in 1° steps (configurable)
2. Capturing one high-quality vertical slice per azimuth step
3. Converting all slices to a combined 3D point cloud
4. Saving unified CSV + PLY outputs

The module exposes run_scan(...) so it can be executed by the MQTT scanner
service in the same style as dump_one_scan.py.
"""

import csv
import math
import os
import sys
import time
from typing import Callable, Dict, List, Optional, Tuple

from rplidar import RPLidar

try:
    from gpiozero import Servo
    from gpiozero.pins.lgpio import LGPIOFactory
except Exception:  # pragma: no cover - expected on non-RPi environments
    Servo = None
    LGPIOFactory = None

from utils.port_config import get_default_port


BAUD = 115200

# Slice quality defaults (reused from xyzscan_servoless.py logic)
MIN_ANGLE_RESOLUTION = 2.0
MIN_COVERAGE = 0.80
RELAXED_MAX_GAP = 10.0
MAX_SCANS_TO_MERGE = 20
PLATEAU_PATIENCE = 5
SLICE_TIMEOUT = 5.0
BUFFER_SIZE = 5000

# Servo defaults (aligned with servotest.py)
SERVO_PIN = 18
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180
SERVO_MIN_PULSE_WIDTH = 1 / 1000
SERVO_MAX_PULSE_WIDTH = 2 / 1000
SERVO_SETTLE_TIME = 0.08
SERVO_RESET_SETTLE_TIME = 0.02
SERVO_RELEASE_AFTER_MOVE = False

# Azimuth sweep defaults
AZIMUTH_START = 0
AZIMUTH_END = 180
AZIMUTH_STEP = 1
PROGRESS_EVERY_SLICES = 1
INTER_SLICE_SLEEP = 1.0
STEP_PERMISSION_REQUIRED = True


class ServoFS90:
    """FS90-compatible servo wrapper using gpiozero/lgpio."""

    def __init__(
        self,
        pin: int,
        min_pulse_width: float = SERVO_MIN_PULSE_WIDTH,
        max_pulse_width: float = SERVO_MAX_PULSE_WIDTH,
    ):
        if Servo is None or LGPIOFactory is None:
            raise RuntimeError(
                "Servo dependencies missing. Install gpiozero and lgpio on Raspberry Pi."
            )

        self.pin = pin
        factory = LGPIOFactory()
        self.servo = Servo(
            pin,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
            pin_factory=factory,
        )

    def set_angle(self, angle: float):
        """Set servo angle, clamped to [0, 180]."""
        clamped = max(float(SERVO_MIN_ANGLE), min(float(SERVO_MAX_ANGLE), float(angle)))
        value = (clamped - 90) / 90.0  # 0° -> -1, 90° -> 0, 180° -> 1
        self.servo.value = value

    def release(self):
        """Disable PWM signal to reduce holding jitter."""
        self.servo.value = None

    def cleanup(self):
        """Release servo resources."""
        self.servo.close()


def _safe_int(value, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _safe_float(value, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _safe_bool(value, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "y", "on"}:
            return True
        if normalized in {"0", "false", "no", "n", "off"}:
            return False
    return bool(value)


def _is_stop_requested(should_stop: Optional[Callable[[], bool]]) -> bool:
    if not should_stop:
        return False
    try:
        return bool(should_stop())
    except Exception:
        return False


def _emit_progress(
    progress_callback: Optional[Callable[[Dict], None]],
    stage: str,
    message: str,
    **kwargs
):
    """Emit best-effort progress callbacks without interrupting scan flow."""
    if not progress_callback:
        return
    payload = {"stage": stage, "message": message}
    payload.update(kwargs)
    try:
        progress_callback(payload)
    except Exception:
        pass


def _sleep_with_stop(
    duration_sec: float,
    should_stop: Optional[Callable[[], bool]] = None,
    tick_sec: float = 0.1,
) -> bool:
    """
    Sleep in small chunks so stop requests can interrupt wait periods.

    Returns:
        bool: True if stop requested during sleep, else False
    """
    remaining = max(0.0, float(duration_sec))
    while remaining > 0:
        if _is_stop_requested(should_stop):
            return True
        chunk = min(max(0.01, tick_sec), remaining)
        time.sleep(chunk)
        remaining -= chunk
    return _is_stop_requested(should_stop)


def merge_scans(scans: List[List[Tuple[int, float, float]]], min_angle_resolution: float) -> List[Tuple[int, float, float]]:
    """
    Merge multiple scans, keeping the highest-quality point in each angle bin.
    """
    angle_bins: Dict[int, Tuple[int, float, float]] = {}

    for scan in scans:
        for quality, angle, distance in scan:
            if distance <= 0:
                continue
            bin_idx = int(angle / min_angle_resolution)
            if bin_idx not in angle_bins or quality > angle_bins[bin_idx][0]:
                angle_bins[bin_idx] = (quality, angle, distance)

    return list(angle_bins.values())


def validate_scan_quality(
    scan: List[Tuple[int, float, float]],
    min_angle_resolution: float,
    min_coverage: float,
    relaxed_max_gap: float,
) -> Tuple[bool, float, float, int, str]:
    """
    Validate scan quality with coverage + max-gap criteria.
    """
    if not scan or len(scan) < 10:
        return False, 0.0, 360.0, len(scan), "Too few points"

    angles = sorted([angle for _, angle, distance in scan if distance > 0])
    if len(angles) < 10:
        return False, 0.0, 360.0, len(angles), "Too few valid measurements"

    angle_bins = {int(angle / min_angle_resolution) for angle in angles}
    total_bins = int(360 / min_angle_resolution)
    coverage = len(angle_bins) / total_bins

    max_gap = 0.0
    for i in range(len(angles)):
        nxt = (i + 1) % len(angles)
        gap = angles[nxt] - angles[i]
        if gap < 0:
            gap = (360 - angles[i]) + angles[nxt]
        max_gap = max(max_gap, gap)

    is_valid = coverage >= min_coverage and max_gap <= relaxed_max_gap
    message = f"Coverage: {coverage * 100:.1f}%, Max gap: {max_gap:.1f} deg, Points: {len(angles)}"
    return is_valid, coverage, max_gap, len(angles), message


def perform_scan_at_angle(
    scan_iterator,
    min_angle_resolution: float,
    min_coverage: float,
    relaxed_max_gap: float,
    max_scans_to_merge: int,
    plateau_patience: int,
    slice_timeout: float,
    should_stop: Optional[Callable[[], bool]] = None,
) -> Dict:
    """
    Capture one vertical slice, merging multiple revolutions if needed.
    """
    start_time = time.time()
    collected_scans: List[List[Tuple[int, float, float]]] = []
    merged_slice: Optional[List[Tuple[int, float, float]]] = None
    last_coverage = 0.0
    plateau_count = 0

    while True:
        if _is_stop_requested(should_stop):
            return {
                "success": False,
                "stopped": True,
                "scan": [],
                "coverage": 0.0,
                "max_gap": 360.0,
                "point_count": 0,
                "scans_merged": len(collected_scans),
                "quality_message": "Scan stopped by user",
                "error": None,
            }

        elapsed = time.time() - start_time
        if elapsed > slice_timeout:
            break

        try:
            scan = next(scan_iterator)
        except StopIteration:
            return {
                "success": False,
                "stopped": False,
                "scan": [],
                "coverage": 0.0,
                "max_gap": 360.0,
                "point_count": 0,
                "scans_merged": len(collected_scans),
                "quality_message": "LiDAR scan iterator ended",
                "error": "LiDAR scan iterator ended",
            }
        except Exception as e:
            # Allow transient read errors/timeouts and continue until slice timeout.
            if (time.time() - start_time) >= slice_timeout:
                break
            continue

        if not scan or len(scan) < 50:
            continue

        collected_scans.append(scan)
        merged = merge_scans(collected_scans, min_angle_resolution)
        is_valid, coverage, max_gap, point_count, quality_msg = validate_scan_quality(
            merged, min_angle_resolution, min_coverage, relaxed_max_gap
        )

        if abs(coverage - last_coverage) < 0.01:
            plateau_count += 1
        else:
            plateau_count = 0
        last_coverage = coverage

        merged_slice = merged

        if is_valid:
            break
        if plateau_count >= plateau_patience and len(collected_scans) >= 5:
            break
        if len(collected_scans) >= max_scans_to_merge:
            break
        if elapsed > slice_timeout:
            break

    if not merged_slice:
        return {
            "success": False,
            "stopped": False,
            "scan": [],
            "coverage": 0.0,
            "max_gap": 360.0,
            "point_count": 0,
            "scans_merged": len(collected_scans),
            "quality_message": "No scan data received",
            "error": "No scan data received",
        }

    is_valid, coverage, max_gap, point_count, quality_msg = validate_scan_quality(
        merged_slice, min_angle_resolution, min_coverage, relaxed_max_gap
    )

    return {
        "success": True,
        "stopped": False,
        "scan": merged_slice,
        "coverage": coverage,
        "max_gap": max_gap,
        "point_count": point_count,
        "scans_merged": len(collected_scans),
        "is_valid": is_valid,
        "quality_message": quality_msg,
        "error": None,
    }


def compute_3d_coordinates(scan: List[Tuple[int, float, float]], servo_angle_deg: int) -> List[Tuple]:
    """
    Convert one vertical slice to 3D points.
    """
    points_3d = []
    azimuth_rad = math.radians(servo_angle_deg)

    for quality, lidar_angle, distance_mm in scan:
        if distance_mm <= 0:
            continue

        r = distance_mm / 1000.0
        elevation_rad = math.radians(lidar_angle)

        r_horizontal = r * math.cos(elevation_rad)
        x = r_horizontal * math.cos(azimuth_rad)
        y = r_horizontal * math.sin(azimuth_rad)
        z = r * math.sin(elevation_rad)

        points_3d.append(
            (quality, lidar_angle, distance_mm, servo_angle_deg, x, y, z)
        )

    return points_3d


def build_sweep_angles(start_angle: int, end_angle: int, step_angle: int) -> List[int]:
    """Build inclusive angle sequence from start to end."""
    step = max(1, abs(step_angle))
    if start_angle <= end_angle:
        return list(range(start_angle, end_angle + 1, step))
    return list(range(start_angle, end_angle - 1, -step))


def build_physical_sweep_angles(
    count: int,
    physical_start: float,
    physical_end: float
) -> List[float]:
    """
    Build calibrated physical servo angles for the logical slice count.
    """
    if count <= 0:
        return []
    if count == 1:
        return [float(physical_start)]

    angles: List[float] = []
    for i in range(count):
        ratio = i / (count - 1)
        angle = physical_start + (physical_end - physical_start) * ratio
        angles.append(angle)
    return angles


def reset_servo_reverse(
    servo: ServoFS90,
    current_angle: float,
    start_angle: float,
    step_angle: float,
    settle_time: float,
):
    """
    Reset servo to start angle by moving opposite the sweep direction.

    Example: sweep 0 -> 180 (increasing) resets via decreasing angles.
    """
    step = max(0.1, abs(float(step_angle)))
    current = float(current_angle)
    start = float(start_angle)

    if current > start:
        angle = current - step
        while angle > start:
            servo.set_angle(angle)
            time.sleep(settle_time)
            angle -= step
    elif current < start:
        angle = current + step
        while angle < start:
            servo.set_angle(angle)
            time.sleep(settle_time)
            angle += step

    servo.set_angle(start)
    time.sleep(settle_time)


def run_scan(
    port: str = "auto",
    output_dir: str = "data",
    servo_config: Optional[Dict] = None,
    scan_config: Optional[Dict] = None,
    should_stop: Optional[Callable[[], bool]] = None,
    progress_callback: Optional[Callable[[Dict], None]] = None,
    hardware_callback: Optional[Callable[[str, object], None]] = None,
    wait_for_step: Optional[Callable[[Dict], str]] = None,
) -> Dict:
    """
    Execute 3D scan and return structured result.

    If step permission is enabled in config, the scan blocks after each slice
    until wait_for_step(...) returns "continue".

    Returns:
        dict with keys:
        - success: bool
        - point_count: int
        - files: [csv_path, ply_path]
        - error: str | None
        - message: str
        - scan_quality: dict
    """
    servo_config = servo_config or {}
    scan_config = scan_config or {}

    servo_pin = _safe_int(servo_config.get("pin", SERVO_PIN), SERVO_PIN)
    min_pulse_width = _safe_float(
        servo_config.get("min_pulse_width", SERVO_MIN_PULSE_WIDTH),
        SERVO_MIN_PULSE_WIDTH,
    )
    max_pulse_width = _safe_float(
        servo_config.get("max_pulse_width", SERVO_MAX_PULSE_WIDTH),
        SERVO_MAX_PULSE_WIDTH,
    )
    servo_settle_time = max(
        0.0, _safe_float(servo_config.get("settle_time", SERVO_SETTLE_TIME), SERVO_SETTLE_TIME)
    )
    reset_settle_time = max(
        0.0,
        _safe_float(
            servo_config.get("reset_settle_time", SERVO_RESET_SETTLE_TIME),
            SERVO_RESET_SETTLE_TIME,
        ),
    )
    release_after_move = _safe_bool(
        servo_config.get("release_after_move", SERVO_RELEASE_AFTER_MOVE),
        SERVO_RELEASE_AFTER_MOVE,
    )

    azimuth_start = _safe_int(scan_config.get("azimuth_start", AZIMUTH_START), AZIMUTH_START)
    azimuth_end = _safe_int(scan_config.get("azimuth_end", AZIMUTH_END), AZIMUTH_END)
    azimuth_step = max(1, abs(_safe_int(scan_config.get("azimuth_step", AZIMUTH_STEP), AZIMUTH_STEP)))
    azimuth_start = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, azimuth_start))
    azimuth_end = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, azimuth_end))
    sweep_angles = build_sweep_angles(azimuth_start, azimuth_end, azimuth_step)

    # Physical calibration range for servo motion.
    # Logical azimuth remains 0..180, but physical motion can avoid end stops.
    servo_scan_start_angle = _safe_float(
        servo_config.get("scan_start_angle", float(sweep_angles[0]) if sweep_angles else float(azimuth_start)),
        float(sweep_angles[0]) if sweep_angles else float(azimuth_start),
    )
    servo_scan_end_angle = _safe_float(
        servo_config.get("scan_end_angle", float(sweep_angles[-1]) if sweep_angles else float(azimuth_end)),
        float(sweep_angles[-1]) if sweep_angles else float(azimuth_end),
    )
    servo_scan_start_angle = max(float(SERVO_MIN_ANGLE), min(float(SERVO_MAX_ANGLE), servo_scan_start_angle))
    servo_scan_end_angle = max(float(SERVO_MIN_ANGLE), min(float(SERVO_MAX_ANGLE), servo_scan_end_angle))
    physical_sweep_angles = build_physical_sweep_angles(
        len(sweep_angles),
        servo_scan_start_angle,
        servo_scan_end_angle,
    )

    min_angle_resolution = max(
        0.1,
        _safe_float(
            scan_config.get("min_angle_resolution", MIN_ANGLE_RESOLUTION),
            MIN_ANGLE_RESOLUTION,
        ),
    )
    min_coverage = _safe_float(scan_config.get("min_coverage", MIN_COVERAGE), MIN_COVERAGE)
    min_coverage = max(0.0, min(1.0, min_coverage))
    relaxed_max_gap = max(
        0.1, _safe_float(scan_config.get("relaxed_max_gap", RELAXED_MAX_GAP), RELAXED_MAX_GAP)
    )
    max_scans_to_merge = max(
        1, _safe_int(scan_config.get("max_scans_to_merge", MAX_SCANS_TO_MERGE), MAX_SCANS_TO_MERGE)
    )
    plateau_patience = max(
        1, _safe_int(scan_config.get("plateau_patience", PLATEAU_PATIENCE), PLATEAU_PATIENCE)
    )
    slice_timeout = max(0.5, _safe_float(scan_config.get("slice_timeout", SLICE_TIMEOUT), SLICE_TIMEOUT))
    buffer_size = max(200, _safe_int(scan_config.get("buffer_size", BUFFER_SIZE), BUFFER_SIZE))
    lidar_timeout = max(0.1, _safe_float(scan_config.get("lidar_timeout", 1.0), 1.0))
    progress_every_slices = max(
        1,
        _safe_int(
            scan_config.get("progress_every_slices", PROGRESS_EVERY_SLICES),
            PROGRESS_EVERY_SLICES,
        ),
    )
    inter_slice_sleep = max(
        0.0,
        _safe_float(
            scan_config.get("inter_slice_sleep", INTER_SLICE_SLEEP),
            INTER_SLICE_SLEEP,
        ),
    )
    step_permission_required = _safe_bool(
        scan_config.get("step_permission_required", STEP_PERMISSION_REQUIRED),
        STEP_PERMISSION_REQUIRED,
    )

    result = {
        "success": False,
        "stopped": False,
        "point_count": 0,
        "files": [],
        "error": "Scan did not start",
        "message": "3D scan failed",
        "scan_quality": {},
    }

    if port == "auto":
        port = get_default_port()

    lidar = None
    scan_iterator = None
    servo = None
    current_angle = servo_scan_start_angle
    reset_completed = False
    reset_error = None

    all_points_3d: List[Tuple] = []
    successful_slices = 0
    failed_slices = 0
    slice_coverages: List[float] = []
    scans_merged_per_slice: List[int] = []

    try:
        if _is_stop_requested(should_stop):
            _emit_progress(progress_callback, "stopped", "Scan stopped before start")
            return {
                "success": False,
                "stopped": True,
                "point_count": 0,
                "files": [],
                "error": None,
                "message": "Scan stopped before start",
                "scan_quality": {},
            }

        servo = ServoFS90(
            pin=servo_pin,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
        )
        if hardware_callback:
            try:
                hardware_callback("servo", servo)
            except Exception:
                pass

        lidar = RPLidar(port, baudrate=BAUD, timeout=lidar_timeout)
        if hardware_callback:
            try:
                hardware_callback("lidar", lidar)
            except Exception:
                pass
        lidar.get_info()
        lidar.get_health()
        lidar.start_motor()
        time.sleep(2)
        scan_iterator = lidar.iter_scans(max_buf_meas=buffer_size, min_len=50)

        if not sweep_angles:
            result["error"] = "No sweep angles generated"
            result["message"] = "3D scan failed: no sweep angles generated"
            return result

        _emit_progress(
            progress_callback,
            "started",
            f"3D sweep initialized: {len(sweep_angles)} slices ({azimuth_start}°->{azimuth_end}°)",
            total_slices=len(sweep_angles),
            azimuth_start=azimuth_start,
            azimuth_end=azimuth_end,
            azimuth_step=azimuth_step,
            physical_start=physical_sweep_angles[0],
            physical_end=physical_sweep_angles[-1],
        )

        # Set the scan reference start azimuth (0° by default).
        servo.set_angle(physical_sweep_angles[0])
        current_angle = physical_sweep_angles[0]
        time.sleep(servo_settle_time)
        if release_after_move:
            servo.release()

        for slice_index, (angle, physical_angle) in enumerate(
            zip(sweep_angles, physical_sweep_angles),
            start=1,
        ):
            if _is_stop_requested(should_stop):
                _emit_progress(
                    progress_callback,
                    "stopped",
                    f"Scan stopped at slice {slice_index}/{len(sweep_angles)}",
                    slice_index=slice_index,
                    total_slices=len(sweep_angles),
                    angle=angle,
                    point_count=len(all_points_3d),
                )
                return {
                    "success": False,
                    "stopped": True,
                    "point_count": len(all_points_3d),
                    "files": [],
                    "error": None,
                    "message": "Scan stopped by user",
                    "scan_quality": {
                        "successful_slices": successful_slices,
                        "failed_slices": failed_slices,
                        "total_slices": len(sweep_angles),
                    },
                }

            if (
                slice_index == 1
                or slice_index % progress_every_slices == 0
                or slice_index == len(sweep_angles)
            ):
                _emit_progress(
                    progress_callback,
                    "slice_start",
                    (
                        f"Scanning slice {slice_index}/{len(sweep_angles)} "
                        f"at azimuth {angle}° (servo {physical_angle:.1f}°)"
                    ),
                    slice_index=slice_index,
                    total_slices=len(sweep_angles),
                    angle=angle,
                    servo_angle=physical_angle,
                    point_count=len(all_points_3d),
                )

            slice_result = perform_scan_at_angle(
                scan_iterator=scan_iterator,
                min_angle_resolution=min_angle_resolution,
                min_coverage=min_coverage,
                relaxed_max_gap=relaxed_max_gap,
                max_scans_to_merge=max_scans_to_merge,
                plateau_patience=plateau_patience,
                slice_timeout=slice_timeout,
                should_stop=should_stop,
            )

            if slice_result.get("stopped"):
                _emit_progress(
                    progress_callback,
                    "stopped",
                    f"Scan stopped at slice {slice_index}/{len(sweep_angles)}",
                    slice_index=slice_index,
                    total_slices=len(sweep_angles),
                    angle=angle,
                    point_count=len(all_points_3d),
                )
                return {
                    "success": False,
                    "stopped": True,
                    "point_count": len(all_points_3d),
                    "files": [],
                    "error": None,
                    "message": "Scan stopped by user",
                    "scan_quality": {
                        "successful_slices": successful_slices,
                        "failed_slices": failed_slices,
                        "total_slices": len(sweep_angles),
                    },
                }

            slice_succeeded = bool(slice_result.get("success"))
            if not slice_succeeded:
                failed_slices += 1
                if (
                    slice_index == 1
                    or slice_index % progress_every_slices == 0
                    or slice_index == len(sweep_angles)
                ):
                    _emit_progress(
                        progress_callback,
                        "slice_failed",
                        (
                            f"Slice {slice_index}/{len(sweep_angles)} failed "
                            f"at azimuth {angle}° (servo {physical_angle:.1f}°)"
                        ),
                        slice_index=slice_index,
                        total_slices=len(sweep_angles),
                        angle=angle,
                        servo_angle=physical_angle,
                        point_count=len(all_points_3d),
                    )
            else:
                successful_slices += 1
                scans_merged_per_slice.append(slice_result["scans_merged"])
                slice_coverages.append(slice_result["coverage"] * 100.0)

                all_points_3d.extend(
                    compute_3d_coordinates(slice_result["scan"], servo_angle_deg=angle)
                )

                if (
                    slice_index == 1
                    or slice_index % progress_every_slices == 0
                    or slice_index == len(sweep_angles)
                ):
                    _emit_progress(
                        progress_callback,
                        "slice_done",
                        (
                            f"Slice {slice_index}/{len(sweep_angles)} done "
                            f"(az={angle}°, servo={physical_angle:.1f}°, "
                            f"cov={slice_result['coverage'] * 100:.1f}%, "
                            f"total_pts={len(all_points_3d)})"
                        ),
                        slice_index=slice_index,
                        total_slices=len(sweep_angles),
                        angle=angle,
                        servo_angle=physical_angle,
                        point_count=len(all_points_3d),
                    )

            if slice_index >= len(sweep_angles):
                continue

            next_angle = sweep_angles[slice_index]
            next_physical_angle = physical_sweep_angles[slice_index]

            if inter_slice_sleep > 0:
                _emit_progress(
                    progress_callback,
                    "slice_pause",
                    (
                        f"Slice {slice_index}/{len(sweep_angles)} complete. "
                        f"Pausing {inter_slice_sleep:.1f}s before step."
                    ),
                    slice_index=slice_index,
                    total_slices=len(sweep_angles),
                    angle=angle,
                    servo_angle=physical_angle,
                    next_angle=next_angle,
                    next_servo_angle=next_physical_angle,
                    slice_success=slice_succeeded,
                    point_count=len(all_points_3d),
                )
                if _sleep_with_stop(inter_slice_sleep, should_stop=should_stop):
                    _emit_progress(
                        progress_callback,
                        "stopped",
                        f"Scan stopped at slice {slice_index}/{len(sweep_angles)}",
                        slice_index=slice_index,
                        total_slices=len(sweep_angles),
                        angle=angle,
                        point_count=len(all_points_3d),
                    )
                    return {
                        "success": False,
                        "stopped": True,
                        "point_count": len(all_points_3d),
                        "files": [],
                        "error": None,
                        "message": "Scan stopped by user",
                        "scan_quality": {
                            "successful_slices": successful_slices,
                            "failed_slices": failed_slices,
                            "total_slices": len(sweep_angles),
                        },
                    }

            if step_permission_required:
                _emit_progress(
                    progress_callback,
                    "step_wait",
                    (
                        f"Awaiting step permission for slice {slice_index + 1}/{len(sweep_angles)} "
                        f"(az={next_angle}°, servo={next_physical_angle:.1f}°)"
                    ),
                    slice_index=slice_index,
                    total_slices=len(sweep_angles),
                    angle=angle,
                    servo_angle=physical_angle,
                    next_angle=next_angle,
                    next_servo_angle=next_physical_angle,
                    point_count=len(all_points_3d),
                )
                if wait_for_step is None:
                    return {
                        "success": False,
                        "stopped": False,
                        "point_count": len(all_points_3d),
                        "files": [],
                        "error": "Step permission is required but no wait_for_step callback was provided",
                        "message": "3D scan failed: step permission callback missing",
                        "scan_quality": {
                            "successful_slices": successful_slices,
                            "failed_slices": failed_slices,
                            "total_slices": len(sweep_angles),
                        },
                    }
                decision = str(
                    wait_for_step(
                        {
                            "slice_index": slice_index,
                            "total_slices": len(sweep_angles),
                            "angle": angle,
                            "servo_angle": physical_angle,
                            "next_angle": next_angle,
                            "next_servo_angle": next_physical_angle,
                            "point_count": len(all_points_3d),
                        }
                    )
                ).strip().lower()
                if decision == "stop":
                    _emit_progress(
                        progress_callback,
                        "stopped",
                        f"Scan stopped at slice {slice_index}/{len(sweep_angles)}",
                        slice_index=slice_index,
                        total_slices=len(sweep_angles),
                        angle=angle,
                        point_count=len(all_points_3d),
                    )
                    return {
                        "success": False,
                        "stopped": True,
                        "point_count": len(all_points_3d),
                        "files": [],
                        "error": None,
                        "message": "Scan stopped by user",
                        "scan_quality": {
                            "successful_slices": successful_slices,
                            "failed_slices": failed_slices,
                            "total_slices": len(sweep_angles),
                        },
                    }
                if decision != "continue":
                    _emit_progress(
                        progress_callback,
                        "stopped",
                        f"Scan stopped at slice {slice_index}/{len(sweep_angles)}",
                        slice_index=slice_index,
                        total_slices=len(sweep_angles),
                        angle=angle,
                        point_count=len(all_points_3d),
                    )
                    return {
                        "success": False,
                        "stopped": True,
                        "point_count": len(all_points_3d),
                        "files": [],
                        "error": None,
                        "message": "Scan stopped by user",
                        "scan_quality": {
                            "successful_slices": successful_slices,
                            "failed_slices": failed_slices,
                            "total_slices": len(sweep_angles),
                        },
                    }
                _emit_progress(
                    progress_callback,
                    "step_granted",
                    (
                        f"Step permission granted for slice {slice_index + 1}/{len(sweep_angles)} "
                        f"(az={next_angle}°)"
                    ),
                    slice_index=slice_index,
                    total_slices=len(sweep_angles),
                    angle=angle,
                    servo_angle=physical_angle,
                    next_angle=next_angle,
                    next_servo_angle=next_physical_angle,
                    point_count=len(all_points_3d),
                )
            else:
                _emit_progress(
                    progress_callback,
                    "step_auto",
                    (
                        f"Step permission disabled; auto-stepping to slice "
                        f"{slice_index + 1}/{len(sweep_angles)}."
                    ),
                    slice_index=slice_index,
                    total_slices=len(sweep_angles),
                    angle=angle,
                    servo_angle=physical_angle,
                    next_angle=next_angle,
                    next_servo_angle=next_physical_angle,
                    point_count=len(all_points_3d),
                )

            if _is_stop_requested(should_stop):
                _emit_progress(
                    progress_callback,
                    "stopped",
                    f"Scan stopped at slice {slice_index}/{len(sweep_angles)}",
                    slice_index=slice_index,
                    total_slices=len(sweep_angles),
                    angle=angle,
                    point_count=len(all_points_3d),
                )
                return {
                    "success": False,
                    "stopped": True,
                    "point_count": len(all_points_3d),
                    "files": [],
                    "error": None,
                    "message": "Scan stopped by user",
                    "scan_quality": {
                        "successful_slices": successful_slices,
                        "failed_slices": failed_slices,
                        "total_slices": len(sweep_angles),
                    },
                }

            _emit_progress(
                progress_callback,
                "step_move",
                (
                    f"Stepping to slice {slice_index + 1}/{len(sweep_angles)}: "
                    f"az={next_angle}° (servo {next_physical_angle:.1f}°)"
                ),
                slice_index=slice_index,
                total_slices=len(sweep_angles),
                angle=angle,
                servo_angle=physical_angle,
                next_angle=next_angle,
                next_servo_angle=next_physical_angle,
                point_count=len(all_points_3d),
            )
            servo.set_angle(next_physical_angle)
            current_angle = next_physical_angle
            if _sleep_with_stop(servo_settle_time, should_stop=should_stop):
                _emit_progress(
                    progress_callback,
                    "stopped",
                    f"Scan stopped at slice {slice_index}/{len(sweep_angles)}",
                    slice_index=slice_index,
                    total_slices=len(sweep_angles),
                    angle=angle,
                    point_count=len(all_points_3d),
                )
                return {
                    "success": False,
                    "stopped": True,
                    "point_count": len(all_points_3d),
                    "files": [],
                    "error": None,
                    "message": "Scan stopped by user",
                    "scan_quality": {
                        "successful_slices": successful_slices,
                        "failed_slices": failed_slices,
                        "total_slices": len(sweep_angles),
                    },
                }
            if release_after_move:
                servo.release()

        if not all_points_3d:
            result["error"] = "No valid 3D points collected"
            result["message"] = "3D scan failed: no valid points collected"
            result["scan_quality"] = {
                "successful_slices": successful_slices,
                "failed_slices": failed_slices,
                "total_slices": len(sweep_angles),
            }
            return result

        os.makedirs(output_dir, exist_ok=True)
        csv_file = os.path.join(output_dir, "scan.csv")
        ply_file = os.path.join(output_dir, "scan.ply")

        _emit_progress(
            progress_callback,
            "saving",
            f"Saving output files ({len(all_points_3d)} points)",
            point_count=len(all_points_3d),
        )

        with open(csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "quality",
                    "angle_deg",
                    "distance_mm",
                    "servo_angle_deg",
                    "x_m",
                    "y_m",
                    "z_m",
                ]
            )
            writer.writerows(all_points_3d)

        with open(ply_file, "w") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(all_points_3d)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            for _, _, _, _, x, y, z in all_points_3d:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        avg_coverage = sum(slice_coverages) / len(slice_coverages) if slice_coverages else 0.0
        avg_scans_merged = (
            sum(scans_merged_per_slice) / len(scans_merged_per_slice)
            if scans_merged_per_slice
            else 0.0
        )

        result = {
            "success": True,
            "stopped": False,
            "point_count": len(all_points_3d),
            "files": [csv_file, ply_file],
            "error": None,
            "message": (
                f"3D scan completed: {len(all_points_3d)} points "
                f"from {successful_slices}/{len(sweep_angles)} slices"
                + (f" ({failed_slices} failed)" if failed_slices else "")
            ),
            "scan_quality": {
                "successful_slices": successful_slices,
                "failed_slices": failed_slices,
                "total_slices": len(sweep_angles),
                "avg_slice_coverage_percent": avg_coverage,
                "avg_scans_merged_per_slice": avg_scans_merged,
                "azimuth_start": azimuth_start,
                "azimuth_end": azimuth_end,
                "azimuth_step": azimuth_step,
                "servo_scan_start_angle": physical_sweep_angles[0],
                "servo_scan_end_angle": physical_sweep_angles[-1],
            },
        }

    except Exception as e:
        _emit_progress(progress_callback, "error", f"3D scan error: {str(e)}")
        result = {
            "success": False,
            "stopped": False,
            "point_count": 0,
            "files": [],
            "error": str(e),
            "message": f"3D scan failed: {str(e)}",
            "scan_quality": {
                "successful_slices": successful_slices,
                "failed_slices": failed_slices,
                "total_slices": len(sweep_angles),
            },
        }

    finally:
        if servo is not None:
            try:
                # Reset opposite the sweep direction to return to the initial azimuth.
                reset_servo_reverse(
                    servo=servo,
                    current_angle=current_angle,
                    start_angle=physical_sweep_angles[0] if physical_sweep_angles else servo_scan_start_angle,
                    step_angle=abs(servo_scan_end_angle - servo_scan_start_angle) / max(1, len(sweep_angles) - 1),
                    settle_time=reset_settle_time,
                )
                reset_completed = True
            except Exception as e:
                reset_error = str(e)
            finally:
                try:
                    servo.release()
                except Exception:
                    pass
                try:
                    servo.cleanup()
                except Exception:
                    pass

        if lidar is not None:
            try:
                lidar.stop()
            except Exception:
                pass
            try:
                lidar.stop_motor()
            except Exception:
                pass
            try:
                lidar.disconnect()
            except Exception:
                pass

        if hardware_callback:
            try:
                hardware_callback("lidar", None)
            except Exception:
                pass
            try:
                hardware_callback("servo", None)
            except Exception:
                pass

        if "scan_quality" not in result or result["scan_quality"] is None:
            result["scan_quality"] = {}
        result["scan_quality"]["reset_completed"] = reset_completed
        if reset_error:
            result["scan_quality"]["reset_error"] = reset_error

        if result.get("success"):
            _emit_progress(
                progress_callback,
                "completed",
                f"3D scan complete: {result.get('point_count', 0)} points",
                point_count=result.get("point_count", 0),
            )
        elif result.get("stopped"):
            _emit_progress(
                progress_callback,
                "stopped",
                result.get("message", "Scan stopped by user"),
                point_count=result.get("point_count", 0),
            )

    return result


def main():
    """CLI entrypoint."""
    port = "auto"
    output_dir = "data"

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        output_dir = sys.argv[2]

    print("=" * 70, flush=True)
    print("Automated RPLidar 3D Scanner (Servo Sweep)", flush=True)
    print("=" * 70, flush=True)
    print(f"Port: {port}", flush=True)
    print(f"Output: {output_dir}", flush=True)
    print("Sweep: azimuth 0° -> 180° in 1° steps", flush=True)
    print(flush=True)

    result = run_scan(port=port, output_dir=output_dir)

    if result["success"]:
        print(f"SUCCESS: {result['message']}", flush=True)
        print(f"Points: {result['point_count']}", flush=True)
        for path in result["files"]:
            print(f"  - {path}", flush=True)
        sys.exit(0)

    print(f"FAILED: {result['message']}", flush=True)
    if result.get("error"):
        print(f"Error: {result['error']}", flush=True)
    sys.exit(1)


if __name__ == "__main__":
    main()

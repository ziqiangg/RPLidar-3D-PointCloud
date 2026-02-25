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
from typing import Dict, List, Optional, Tuple

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

# Azimuth sweep defaults
AZIMUTH_START = 0
AZIMUTH_END = 180
AZIMUTH_STEP = 1


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

    def set_angle(self, angle: int):
        """Set servo angle, clamped to [0, 180]."""
        clamped = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, int(angle)))
        value = (clamped - 90) / 90.0  # 0° -> -1, 90° -> 0, 180° -> 1
        self.servo.value = value

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
    lidar: RPLidar,
    min_angle_resolution: float,
    min_coverage: float,
    relaxed_max_gap: float,
    max_scans_to_merge: int,
    plateau_patience: int,
    slice_timeout: float,
    buffer_size: int,
) -> Dict:
    """
    Capture one vertical slice, merging multiple revolutions if needed.
    """
    start_time = time.time()
    collected_scans: List[List[Tuple[int, float, float]]] = []
    merged_slice: Optional[List[Tuple[int, float, float]]] = None
    last_coverage = 0.0
    plateau_count = 0

    for scan in lidar.iter_scans(max_buf_meas=buffer_size, min_len=50):
        elapsed = time.time() - start_time

        if not scan or len(scan) < 50:
            if elapsed > slice_timeout:
                break
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


def reset_servo_reverse(
    servo: ServoFS90,
    current_angle: int,
    start_angle: int,
    step_angle: int,
    settle_time: float,
):
    """
    Reset servo to start angle by moving opposite the sweep direction.

    Example: sweep 0 -> 180 (increasing) resets via decreasing angles.
    """
    step = max(1, abs(step_angle))

    if current_angle > start_angle:
        for angle in range(current_angle - step, start_angle - 1, -step):
            servo.set_angle(angle)
            time.sleep(settle_time)
    elif current_angle < start_angle:
        for angle in range(current_angle + step, start_angle + 1, step):
            servo.set_angle(angle)
            time.sleep(settle_time)

    servo.set_angle(start_angle)
    time.sleep(settle_time)


def run_scan(
    port: str = "auto",
    output_dir: str = "data",
    servo_config: Optional[Dict] = None,
    scan_config: Optional[Dict] = None,
) -> Dict:
    """
    Execute automated 3D scan and return structured result.

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

    azimuth_start = _safe_int(scan_config.get("azimuth_start", AZIMUTH_START), AZIMUTH_START)
    azimuth_end = _safe_int(scan_config.get("azimuth_end", AZIMUTH_END), AZIMUTH_END)
    azimuth_step = max(1, abs(_safe_int(scan_config.get("azimuth_step", AZIMUTH_STEP), AZIMUTH_STEP)))
    azimuth_start = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, azimuth_start))
    azimuth_end = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, azimuth_end))

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

    result = {
        "success": False,
        "point_count": 0,
        "files": [],
        "error": "Scan did not start",
        "message": "3D scan failed",
        "scan_quality": {},
    }

    if port == "auto":
        port = get_default_port()

    lidar = None
    servo = None
    sweep_angles: List[int] = []
    current_angle = azimuth_start
    reset_completed = False
    reset_error = None

    all_points_3d: List[Tuple] = []
    successful_slices = 0
    failed_slices = 0
    slice_coverages: List[float] = []
    scans_merged_per_slice: List[int] = []

    try:
        servo = ServoFS90(
            pin=servo_pin,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
        )

        lidar = RPLidar(port, baudrate=BAUD, timeout=3)
        lidar.get_info()
        lidar.get_health()
        lidar.start_motor()
        time.sleep(2)

        sweep_angles = build_sweep_angles(azimuth_start, azimuth_end, azimuth_step)
        if not sweep_angles:
            result["error"] = "No sweep angles generated"
            result["message"] = "3D scan failed: no sweep angles generated"
            return result

        # Set the scan reference start azimuth (0° by default).
        servo.set_angle(sweep_angles[0])
        current_angle = sweep_angles[0]
        time.sleep(servo_settle_time)

        for angle in sweep_angles:
            servo.set_angle(angle)
            current_angle = angle
            time.sleep(servo_settle_time)

            slice_result = perform_scan_at_angle(
                lidar=lidar,
                min_angle_resolution=min_angle_resolution,
                min_coverage=min_coverage,
                relaxed_max_gap=relaxed_max_gap,
                max_scans_to_merge=max_scans_to_merge,
                plateau_patience=plateau_patience,
                slice_timeout=slice_timeout,
                buffer_size=buffer_size,
            )

            if not slice_result["success"]:
                failed_slices += 1
                continue

            successful_slices += 1
            scans_merged_per_slice.append(slice_result["scans_merged"])
            slice_coverages.append(slice_result["coverage"] * 100.0)

            all_points_3d.extend(
                compute_3d_coordinates(slice_result["scan"], servo_angle_deg=angle)
            )

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
            },
        }

    except Exception as e:
        result = {
            "success": False,
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
                    start_angle=azimuth_start,
                    step_angle=azimuth_step,
                    settle_time=reset_settle_time,
                )
                reset_completed = True
            except Exception as e:
                reset_error = str(e)
            finally:
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

        if "scan_quality" not in result or result["scan_quality"] is None:
            result["scan_quality"] = {}
        result["scan_quality"]["reset_completed"] = reset_completed
        if reset_error:
            result["scan_quality"]["reset_error"] = reset_error

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

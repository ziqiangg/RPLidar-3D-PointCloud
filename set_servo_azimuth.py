#!/usr/bin/env python3
"""
Move FS90 servo to a requested azimuth.

By default, the provided azimuth is a logical scan azimuth (0..180) and is
mapped to the configured physical servo range:
    servo.scan_start_angle -> servo.scan_end_angle

Use --physical to bypass mapping and command the given angle directly.
"""

import argparse
import json
import os
import time
from typing import Any, Dict

import yaml

try:
    from gpiozero import Servo
    from gpiozero.pins.lgpio import LGPIOFactory
except Exception:  # pragma: no cover - expected outside Raspberry Pi
    Servo = None
    LGPIOFactory = None


SERVO_MIN_ANGLE = 0.0
SERVO_MAX_ANGLE = 180.0
DEFAULT_HOLD_SECONDS = 1.0
DEFAULT_STATE_FILE = "/tmp/servo_azimuth_state.json"
DEFAULT_NEUTRAL_VALUE = 0.0
DEFAULT_CW_VALUE = 0.35
DEFAULT_CCW_VALUE = -0.35
DEFAULT_DEG_PER_SEC_CW = 120.0
DEFAULT_DEG_PER_SEC_CCW = 120.0


def _safe_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _safe_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _load_config(config_path: str) -> Dict[str, Any]:
    with open(config_path, "r") as f:
        return yaml.safe_load(f) or {}


def _safe_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _map_logical_to_physical(logical_azimuth: float, servo_cfg: Dict[str, Any]) -> float:
    logical = _clamp(float(logical_azimuth), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
    physical_start = _safe_float(servo_cfg.get("scan_start_angle", 0.0), 0.0)
    physical_end = _safe_float(servo_cfg.get("scan_end_angle", 180.0), 180.0)
    physical_start = _clamp(physical_start, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
    physical_end = _clamp(physical_end, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)

    ratio = (logical - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)
    physical = physical_start + (physical_end - physical_start) * ratio
    return _clamp(physical, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)


def _angle_to_servo_value(angle_deg: float) -> float:
    # gpiozero Servo value range: -1 .. +1
    return (angle_deg - 90.0) / 90.0


def _wrap_360(deg: float) -> float:
    wrapped = float(deg) % 360.0
    # Keep 360 represented as 0 to avoid duplicate state semantics.
    if abs(wrapped - 360.0) < 1e-9:
        return 0.0
    return wrapped


def _shortest_delta_deg(current: float, target: float) -> float:
    """
    Signed shortest delta from current -> target in degrees.

    Positive: clockwise
    Negative: anticlockwise
    """
    delta = (target - current + 540.0) % 360.0 - 180.0
    if abs(delta + 180.0) < 1e-9:
        return 180.0
    return delta


def _load_state_angle(state_file: str) -> float | None:
    try:
        with open(state_file, "r") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return None
        angle = data.get("azimuth_deg")
        if angle is None:
            return None
        return _wrap_360(float(angle))
    except Exception:
        return None


def _save_state_angle(state_file: str, azimuth_deg: float):
    try:
        state_dir = os.path.dirname(state_file)
        if state_dir:
            os.makedirs(state_dir, exist_ok=True)
        with open(state_file, "w") as f:
            json.dump({"azimuth_deg": _wrap_360(azimuth_deg)}, f)
    except Exception:
        pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move servo to a target azimuth using config_rpi.yaml settings."
    )
    parser.add_argument(
        "azimuth",
        type=float,
        help="Target azimuth in degrees (default: logical azimuth 0..180).",
    )
    parser.add_argument(
        "--physical",
        action="store_true",
        help="Treat azimuth as direct physical servo angle (skip logical mapping).",
    )
    parser.add_argument(
        "--continuous",
        action="store_true",
        help=(
            "Use continuous-rotation mode (0..360 target). "
            "This is timed movement using calibration, not true encoder positioning."
        ),
    )
    parser.add_argument(
        "--config",
        default="config_rpi.yaml",
        help="Path to Raspberry Pi config file (default: config_rpi.yaml).",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=None,
        help=(
            "Time to hold before optional release. "
            "Default: max(servo.settle_time, 1.0s)."
        ),
    )
    parser.add_argument(
        "--release-after-move",
        action="store_true",
        help="Release PWM after hold (default keeps PWM active until script exits).",
    )
    parser.add_argument(
        "--current-azimuth",
        type=float,
        default=None,
        help=(
            "Current azimuth estimate (degrees, continuous mode). "
            "If omitted, uses saved state, else assumes 0°."
        ),
    )
    parser.add_argument(
        "--state-file",
        default=DEFAULT_STATE_FILE,
        help=f"Path for saved azimuth state in continuous mode (default: {DEFAULT_STATE_FILE}).",
    )
    parser.add_argument(
        "--wait-for-enter",
        action="store_true",
        help="Wait for Enter before cleanup so position can be inspected.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if Servo is None or LGPIOFactory is None:
        print("ERROR: gpiozero/lgpio not available. Run this on Raspberry Pi with dependencies installed.")
        return 1

    cfg = _load_config(args.config)
    servo_cfg = cfg.get("servo", {}) if isinstance(cfg, dict) else {}
    servo_cfg = _safe_dict(servo_cfg)
    continuous_cfg = _safe_dict(servo_cfg.get("continuous", {}))

    pin = _safe_int(servo_cfg.get("pin", 18), 18)
    min_pulse = _safe_float(servo_cfg.get("min_pulse_width", 0.001), 0.001)
    max_pulse = _safe_float(servo_cfg.get("max_pulse_width", 0.002), 0.002)
    settle_time = max(0.0, _safe_float(servo_cfg.get("settle_time", 0.08), 0.08))
    hold_seconds = (
        max(settle_time, DEFAULT_HOLD_SECONDS)
        if args.hold_seconds is None
        else max(0.0, float(args.hold_seconds))
    )
    release_after_move = bool(args.release_after_move)

    requested = float(args.azimuth)

    print("Servo move request")
    print(f"  pin: {pin}")
    print(f"  mode: {'continuous' if args.continuous else 'positional'}")

    servo = None
    try:
        factory = LGPIOFactory()
        servo = Servo(
            pin,
            min_pulse_width=min_pulse,
            max_pulse_width=max_pulse,
            pin_factory=factory,
        )

        if args.continuous:
            neutral_value = _safe_float(
                continuous_cfg.get("neutral_value", DEFAULT_NEUTRAL_VALUE),
                DEFAULT_NEUTRAL_VALUE,
            )
            cw_value = _safe_float(
                continuous_cfg.get("cw_value", DEFAULT_CW_VALUE),
                DEFAULT_CW_VALUE,
            )
            ccw_value = _safe_float(
                continuous_cfg.get("ccw_value", DEFAULT_CCW_VALUE),
                DEFAULT_CCW_VALUE,
            )
            deg_per_sec_cw = max(
                0.1,
                _safe_float(
                    continuous_cfg.get("deg_per_sec_cw", DEFAULT_DEG_PER_SEC_CW),
                    DEFAULT_DEG_PER_SEC_CW,
                ),
            )
            deg_per_sec_ccw = max(
                0.1,
                _safe_float(
                    continuous_cfg.get("deg_per_sec_ccw", DEFAULT_DEG_PER_SEC_CCW),
                    DEFAULT_DEG_PER_SEC_CCW,
                ),
            )

            target_az = _wrap_360(requested)
            if args.current_azimuth is not None:
                current_az = _wrap_360(float(args.current_azimuth))
                current_source = "--current-azimuth"
            else:
                saved = _load_state_angle(args.state_file)
                if saved is not None:
                    current_az = saved
                    current_source = f"state file ({args.state_file})"
                else:
                    current_az = 0.0
                    current_source = "default 0.0°"

            delta = _shortest_delta_deg(current_az, target_az)
            rotate_deg = abs(delta)
            if rotate_deg < 0.01:
                direction = "none"
                drive_value = neutral_value
                duration = 0.0
            elif delta > 0:
                direction = "clockwise"
                drive_value = cw_value
                duration = rotate_deg / deg_per_sec_cw
            else:
                direction = "anticlockwise"
                drive_value = ccw_value
                duration = rotate_deg / deg_per_sec_ccw

            print(f"  target azimuth: {target_az:.1f}° (0..360)")
            print(f"  current azimuth estimate: {current_az:.1f}° ({current_source})")
            print(f"  planned direction: {direction}")
            print(f"  planned rotation: {rotate_deg:.1f}°")
            print(f"  drive value: {drive_value:.3f}")
            print(f"  drive duration: {duration:.3f}s")
            print(f"  release after move: {release_after_move}")
            print("  note: this is timed control; calibrate deg_per_sec_* for your servo.")

            # Stop first, then drive, then stop.
            servo.value = neutral_value
            time.sleep(0.1)
            if duration > 0:
                servo.value = drive_value
                time.sleep(duration)
                servo.value = neutral_value
                time.sleep(0.1)
            _save_state_angle(args.state_file, target_az)

            if args.wait_for_enter:
                input("Press Enter to release/exit...")
        else:
            requested = _clamp(requested, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
            if args.physical:
                physical = requested
                logical_info = "n/a (physical mode)"
            else:
                physical = _map_logical_to_physical(requested, servo_cfg)
                logical_info = f"{requested:.1f}°"

            print(f"  logical azimuth: {logical_info}")
            print(f"  physical angle: {physical:.1f}°")
            print(f"  hold seconds: {hold_seconds:.2f}")
            print(f"  release after move: {release_after_move}")
            if abs(physical - 90.0) < 0.01:
                print("  note: 90° is center; if servo is already centered, movement may be minimal.")
            print("  note: positional mode assumes 0..180 degree servo response.")

            servo.value = _angle_to_servo_value(physical)
            time.sleep(hold_seconds)

            if args.wait_for_enter:
                input("Press Enter to release/exit...")

        if release_after_move:
            servo.value = None

        print("Done.")
        return 0

    except KeyboardInterrupt:
        print("Interrupted.")
        return 130
    except Exception as e:
        print(f"ERROR: {e}")
        return 1
    finally:
        if servo is not None:
            try:
                servo.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())

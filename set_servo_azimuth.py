#!/usr/bin/env python3
"""
Move FS90 servo to a requested azimuth.

By default, the provided azimuth is a logical scan azimuth (0..180) and is
mapped to the configured physical servo range:
    servo.scan_start_angle -> servo.scan_end_angle

Use --physical to bypass mapping and command the given angle directly.
"""

import argparse
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
            "Default: use servo.settle_time from config."
        ),
    )
    parser.add_argument(
        "--keep-holding",
        action="store_true",
        help="Do not release PWM after move.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if Servo is None or LGPIOFactory is None:
        print("ERROR: gpiozero/lgpio not available. Run this on Raspberry Pi with dependencies installed.")
        return 1

    cfg = _load_config(args.config)
    servo_cfg = cfg.get("servo", {}) if isinstance(cfg, dict) else {}

    pin = _safe_int(servo_cfg.get("pin", 18), 18)
    min_pulse = _safe_float(servo_cfg.get("min_pulse_width", 0.001), 0.001)
    max_pulse = _safe_float(servo_cfg.get("max_pulse_width", 0.002), 0.002)
    settle_time = max(0.0, _safe_float(servo_cfg.get("settle_time", 0.08), 0.08))
    hold_seconds = settle_time if args.hold_seconds is None else max(0.0, float(args.hold_seconds))

    requested = _clamp(float(args.azimuth), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
    if args.physical:
        physical = requested
        logical_info = "n/a (physical mode)"
    else:
        physical = _map_logical_to_physical(requested, servo_cfg)
        logical_info = f"{requested:.1f}°"

    print("Servo move request")
    print(f"  pin: {pin}")
    print(f"  logical azimuth: {logical_info}")
    print(f"  physical angle: {physical:.1f}°")
    print(f"  hold seconds: {hold_seconds:.2f}")
    print(f"  release after move: {not args.keep_holding}")

    servo = None
    try:
        factory = LGPIOFactory()
        servo = Servo(
            pin,
            min_pulse_width=min_pulse,
            max_pulse_width=max_pulse,
            pin_factory=factory,
        )

        servo.value = _angle_to_servo_value(physical)
        time.sleep(hold_seconds)

        if not args.keep_holding:
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

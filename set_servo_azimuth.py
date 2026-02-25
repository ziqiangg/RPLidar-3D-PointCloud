#!/usr/bin/env python3
"""
Continuous-servo azimuth calibration helper.

This script keeps a software azimuth estimate and lets you:
1) Set current physical position as 0 degrees
2) Step by N degrees in a chosen direction (cw/ccw)

State is stored in a small JSON file (default: /tmp/servo_azimuth_state.json).
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


DEFAULT_CONFIG_PATH = "config_rpi.yaml"
DEFAULT_STATE_FILE = "/tmp/servo_azimuth_state.json"

DEFAULT_NEUTRAL_VALUE = 0.0
DEFAULT_CW_VALUE = 0.35
DEFAULT_CCW_VALUE = -0.35
DEFAULT_DEG_PER_SEC_CW = 120.0
DEFAULT_DEG_PER_SEC_CCW = 120.0
DEFAULT_STOP_SETTLE_SECONDS = 0.10


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


def _safe_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _wrap_360(deg: float) -> float:
    wrapped = float(deg) % 360.0
    if abs(wrapped - 360.0) < 1e-9:
        return 0.0
    return wrapped


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _load_state(path: str) -> float:
    try:
        with open(path, "r") as f:
            data = json.load(f)
        if isinstance(data, dict) and "azimuth_deg" in data:
            return _wrap_360(float(data["azimuth_deg"]))
    except Exception:
        pass
    return 0.0


def _save_state(path: str, azimuth_deg: float):
    state_dir = os.path.dirname(path)
    if state_dir:
        os.makedirs(state_dir, exist_ok=True)
    with open(path, "w") as f:
        json.dump({"azimuth_deg": _wrap_360(azimuth_deg)}, f)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibrate continuous-servo azimuth (zero + directional steps)."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_PATH,
        help=f"Path to config file (default: {DEFAULT_CONFIG_PATH})",
    )
    parser.add_argument(
        "--state-file",
        default=DEFAULT_STATE_FILE,
        help=f"Path to azimuth state file (default: {DEFAULT_STATE_FILE})",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser(
        "zero",
        help="Set current physical position as azimuth 0° (no rotation).",
    )

    p_step = sub.add_parser(
        "step",
        help="Rotate by N degrees in chosen direction and update azimuth estimate.",
    )
    p_step.add_argument("direction", choices=["cw", "ccw"], help="Rotation direction.")
    p_step.add_argument("degrees", type=float, help="Step size in degrees (positive).")

    p_drive = sub.add_parser(
        "drive",
        help="Drive servo with a raw gpiozero value for direct direction testing.",
    )
    p_drive.add_argument(
        "value",
        type=float,
        help="Raw Servo value (-1.0..1.0). Positive and negative should spin opposite directions.",
    )
    p_drive.add_argument(
        "seconds",
        type=float,
        help="Drive duration in seconds (>0).",
    )

    sub.add_parser(
        "status",
        help="Print current stored azimuth estimate.",
    )

    return parser


def main() -> int:
    args = _build_parser().parse_args()

    if args.command == "status":
        current = _load_state(args.state_file)
        print(f"Current azimuth estimate: {current:.1f}°")
        print(f"State file: {args.state_file}")
        return 0

    if Servo is None or LGPIOFactory is None:
        print("ERROR: gpiozero/lgpio not available. Run this on Raspberry Pi.")
        return 1

    cfg = _load_yaml(args.config)
    servo_cfg = _safe_dict(cfg.get("servo"))
    continuous_cfg = _safe_dict(servo_cfg.get("continuous"))

    pin = _safe_int(servo_cfg.get("pin", 18), 18)
    min_pulse = _safe_float(servo_cfg.get("min_pulse_width", 0.001), 0.001)
    max_pulse = _safe_float(servo_cfg.get("max_pulse_width", 0.002), 0.002)

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
    stop_settle = max(
        0.0,
        _safe_float(
            continuous_cfg.get("stop_settle_seconds", DEFAULT_STOP_SETTLE_SECONDS),
            DEFAULT_STOP_SETTLE_SECONDS,
        ),
    )

    current_az = _load_state(args.state_file)
    servo = None
    try:
        factory = LGPIOFactory()
        servo = Servo(
            pin,
            min_pulse_width=min_pulse,
            max_pulse_width=max_pulse,
            pin_factory=factory,
        )

        # Force neutral before any calibration action.
        servo.value = neutral_value
        if stop_settle > 0:
            time.sleep(stop_settle)

        if args.command == "zero":
            _save_state(args.state_file, 0.0)
            print("Zero reference set.")
            print("  current physical position is now treated as 0.0°")
            print(f"  state file: {args.state_file}")
            return 0

        # raw drive command (direct calibration/debug)
        if args.command == "drive":
            drive_value = max(-1.0, min(1.0, float(args.value)))
            seconds = float(args.seconds)
            if seconds <= 0:
                print("ERROR: drive seconds must be > 0")
                return 2

            print("Drive command")
            print(f"  value: {drive_value:.3f}")
            print(f"  seconds: {seconds:.3f}s")
            print(f"  neutral: {neutral_value:.3f}")

            servo.value = drive_value
            time.sleep(seconds)
            servo.value = neutral_value
            if stop_settle > 0:
                time.sleep(stop_settle)

            print("Done.")
            return 0

        # step command
        step_deg = float(args.degrees)
        if step_deg <= 0:
            print("ERROR: step degrees must be > 0")
            return 2

        direction = str(args.direction)
        if direction == "cw":
            drive_value = cw_value
            duration = step_deg / deg_per_sec_cw
            next_az = _wrap_360(current_az + step_deg)
        else:
            drive_value = ccw_value
            duration = step_deg / deg_per_sec_ccw
            next_az = _wrap_360(current_az - step_deg)

        print("Step command")
        print(f"  direction: {direction}")
        print(f"  step: {step_deg:.1f}°")
        print(f"  current estimate: {current_az:.1f}°")
        print(f"  next estimate: {next_az:.1f}°")
        print(f"  drive value: {drive_value:.3f}")
        print(f"  drive duration: {duration:.3f}s")

        servo.value = drive_value
        time.sleep(duration)
        servo.value = neutral_value
        if stop_settle > 0:
            time.sleep(stop_settle)

        _save_state(args.state_file, next_az)
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
                servo.value = None
            except Exception:
                pass
            try:
                servo.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())

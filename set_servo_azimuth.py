#!/usr/bin/env python3
"""
Minimal continuous-servo calibration helper.

Commands:
  zero                     Set current position as azimuth 0°
  step <cw|ccw> <degrees>  Rotate by N degrees in chosen direction
  status                   Show current stored azimuth estimate
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


CONFIG_PATH = "config_rpi.yaml"
STATE_FILE = "/tmp/servo_azimuth_state.json"

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


def _load_calibration() -> Dict[str, float]:
    cfg = _load_yaml(CONFIG_PATH)
    servo_cfg = _safe_dict(cfg.get("servo"))
    continuous_cfg = _safe_dict(servo_cfg.get("continuous"))

    pin = _safe_int(servo_cfg.get("pin", 18), 18)
    min_pulse = _safe_float(servo_cfg.get("min_pulse_width", 0.001), 0.001)
    max_pulse = _safe_float(servo_cfg.get("max_pulse_width", 0.002), 0.002)

    neutral = _safe_float(
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

    # Guardrail: if both configured values are on the same side of neutral,
    # force CCW to mirrored opposite side so directions are not identical.
    if (cw_value - neutral) * (ccw_value - neutral) >= 0:
        cw_offset = cw_value - neutral
        if abs(cw_offset) < 0.05:
            cw_offset = 0.35
        ccw_value = neutral - cw_offset
        print("WARNING: cw/ccw values were on same side of neutral; using mirrored ccw value.")
        print(f"         effective ccw_value={ccw_value:.3f}")

    return {
        "pin": float(pin),
        "min_pulse": min_pulse,
        "max_pulse": max_pulse,
        "neutral": neutral,
        "cw_value": cw_value,
        "ccw_value": ccw_value,
        "deg_per_sec_cw": deg_per_sec_cw,
        "deg_per_sec_ccw": deg_per_sec_ccw,
        "stop_settle": stop_settle,
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibrate continuous-servo azimuth with simple zero/step commands."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("zero", help="Set current physical position as azimuth 0°.")
    sub.add_parser("status", help="Show current stored azimuth estimate.")

    p_step = sub.add_parser("step", help="Rotate by N degrees in a direction.")
    p_step.add_argument("direction", choices=["cw", "ccw"], help="Rotation direction.")
    p_step.add_argument("degrees", type=float, help="Step size in degrees (>0).")
    return parser


def main() -> int:
    args = _build_parser().parse_args()

    if args.command == "status":
        current = _load_state(STATE_FILE)
        print(f"Azimuth estimate: {current:.1f}°")
        print(f"State file: {STATE_FILE}")
        return 0

    if args.command == "zero":
        _save_state(STATE_FILE, 0.0)
        print("Zero set. Current physical position is now 0.0°.")
        print(f"State file: {STATE_FILE}")
        return 0

    # step command
    if Servo is None or LGPIOFactory is None:
        print("ERROR: gpiozero/lgpio not available. Run this on Raspberry Pi.")
        return 1

    step_deg = float(args.degrees)
    if step_deg <= 0:
        print("ERROR: step degrees must be > 0")
        return 2

    calib = _load_calibration()
    pin = int(calib["pin"])
    min_pulse = float(calib["min_pulse"])
    max_pulse = float(calib["max_pulse"])
    neutral = float(calib["neutral"])
    cw_value = float(calib["cw_value"])
    ccw_value = float(calib["ccw_value"])
    deg_per_sec_cw = float(calib["deg_per_sec_cw"])
    deg_per_sec_ccw = float(calib["deg_per_sec_ccw"])
    stop_settle = float(calib["stop_settle"])

    current_az = _load_state(STATE_FILE)
    direction = str(args.direction)
    if direction == "cw":
        drive_value = cw_value
        duration = step_deg / deg_per_sec_cw
        next_az = _wrap_360(current_az + step_deg)
    else:
        drive_value = ccw_value
        duration = step_deg / deg_per_sec_ccw
        next_az = _wrap_360(current_az - step_deg)

    servo = None
    try:
        factory = LGPIOFactory()
        servo = Servo(
            pin,
            min_pulse_width=min_pulse,
            max_pulse_width=max_pulse,
            pin_factory=factory,
        )

        # Stop -> drive -> stop
        servo.value = neutral
        if stop_settle > 0:
            time.sleep(stop_settle)
        servo.value = drive_value
        time.sleep(duration)
        servo.value = neutral
        if stop_settle > 0:
            time.sleep(stop_settle)

        _save_state(STATE_FILE, next_az)

        print("Step executed.")
        print(f"  direction: {direction}")
        print(f"  step: {step_deg:.1f}°")
        print(f"  current: {current_az:.1f}°")
        print(f"  next: {next_az:.1f}°")
        print(f"  drive value: {drive_value:.3f}")
        print(f"  duration: {duration:.3f}s")
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

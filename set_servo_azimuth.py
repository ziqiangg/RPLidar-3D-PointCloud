#!/usr/bin/env python3
"""
Servo calibration helper using positional angle commands (servotest-style).

Commands:
  zero                     Set current logical azimuth to 0°
  step <cw|ccw> <degrees>  Move by N degrees in direction
  status                   Show logical azimuth + command angle estimate
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

SERVO_MIN_ANGLE = 0.0
SERVO_MAX_ANGLE = 180.0
DEFAULT_COMMAND_ANGLE = 90.0
DEFAULT_SWEEP_STEP = 1.0
DEFAULT_SWEEP_DELAY_SEC = 0.01
DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG = 1.0


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


def _safe_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        s = value.strip().lower()
        if s in {"1", "true", "yes", "y", "on"}:
            return True
        if s in {"0", "false", "no", "n", "off"}:
            return False
    return bool(value)


def _safe_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _wrap_360(deg: float) -> float:
    wrapped = float(deg) % 360.0
    if abs(wrapped - 360.0) < 1e-9:
        return 0.0
    return wrapped


def _clamp_angle(deg: float) -> float:
    return max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, float(deg)))


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _load_state(path: str) -> Dict[str, float]:
    default = {"logical_azimuth_deg": 0.0, "command_angle_deg": DEFAULT_COMMAND_ANGLE}
    try:
        with open(path, "r") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return default
        logical = _wrap_360(_safe_float(data.get("logical_azimuth_deg"), 0.0))
        command = _clamp_angle(_safe_float(data.get("command_angle_deg"), DEFAULT_COMMAND_ANGLE))
        return {"logical_azimuth_deg": logical, "command_angle_deg": command}
    except Exception:
        return default


def _save_state(path: str, logical_azimuth_deg: float, command_angle_deg: float):
    state_dir = os.path.dirname(path)
    if state_dir:
        os.makedirs(state_dir, exist_ok=True)
    with open(path, "w") as f:
        json.dump(
            {
                "logical_azimuth_deg": _wrap_360(logical_azimuth_deg),
                "command_angle_deg": _clamp_angle(command_angle_deg),
            },
            f,
        )


def _angle_to_servo_value(angle_deg: float) -> float:
    # 0° -> -1, 90° -> 0, 180° -> 1
    return (_clamp_angle(angle_deg) - 90.0) / 90.0


def _resolve_pulse_widths(servo_cfg: Dict[str, Any]) -> tuple[float, float]:
    """
    Resolve pulse-width range for set_servo_azimuth calibration moves.

    Preference order:
    1) servo.continuous.{min,max}_pulse_width (explicit calibration range)
    2) servo.{min,max}_pulse_width
    """
    continuous_cfg = _safe_dict(servo_cfg.get("continuous"))
    min_pulse = _safe_float(
        continuous_cfg.get("min_pulse_width", servo_cfg.get("min_pulse_width", 0.001)),
        0.001,
    )
    max_pulse = _safe_float(
        continuous_cfg.get("max_pulse_width", servo_cfg.get("max_pulse_width", 0.002)),
        0.002,
    )
    if min_pulse > max_pulse:
        min_pulse, max_pulse = max_pulse, min_pulse
    return min_pulse, max_pulse


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Simple servo step calibrator (servotest-style positional control)."
    )
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("zero", help="Set current logical azimuth to 0°.")
    sub.add_parser("status", help="Show current logical azimuth and command angle.")
    p_step = sub.add_parser("step", help="Move by N degrees in chosen direction.")
    p_step.add_argument("direction", choices=["cw", "ccw"], help="Move direction.")
    p_step.add_argument("degrees", type=float, help="Step size in degrees (>0).")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    state = _load_state(STATE_FILE)

    if args.command == "status":
        cfg = _load_yaml(CONFIG_PATH)
        servo_cfg = _safe_dict(cfg.get("servo"))
        calib_cfg = _safe_dict(servo_cfg.get("calibration"))
        min_pulse, max_pulse = _resolve_pulse_widths(servo_cfg)
        physical_deg_per_command_deg = _safe_float(
            calib_cfg.get("physical_deg_per_command_deg", DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG),
            DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG,
        )
        if physical_deg_per_command_deg <= 0:
            physical_deg_per_command_deg = DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG
        print(f"Logical azimuth: {state['logical_azimuth_deg']:.1f}°")
        print(f"Command angle: {state['command_angle_deg']:.1f}°")
        print(f"physical_deg_per_command_deg: {physical_deg_per_command_deg:.4f}")
        print(f"Pulse range: {min_pulse * 1e6:.0f}us -> {max_pulse * 1e6:.0f}us")
        print(f"State file: {STATE_FILE}")
        return 0

    if args.command == "zero":
        _save_state(STATE_FILE, 0.0, state["command_angle_deg"])
        print("Zero set.")
        print("Current physical position is now logical 0.0°.")
        print(f"State file: {STATE_FILE}")
        return 0

    if Servo is None or LGPIOFactory is None:
        print("ERROR: gpiozero/lgpio not available. Run this on Raspberry Pi.")
        return 1

    step_deg = _safe_float(getattr(args, "degrees", 0.0), 0.0)
    if step_deg <= 0:
        print("ERROR: step degrees must be > 0")
        return 2

    cfg = _load_yaml(CONFIG_PATH)
    servo_cfg = _safe_dict(cfg.get("servo"))
    pin = _safe_int(servo_cfg.get("pin", 18), 18)
    min_pulse, max_pulse = _resolve_pulse_widths(servo_cfg)

    calib_cfg = _safe_dict(servo_cfg.get("calibration"))
    cw_increases_angle = _safe_bool(calib_cfg.get("cw_increases_angle", True), True)
    sweep_step = max(0.1, _safe_float(calib_cfg.get("sweep_step_deg", DEFAULT_SWEEP_STEP), DEFAULT_SWEEP_STEP))
    sweep_delay = max(0.0, _safe_float(calib_cfg.get("sweep_delay_sec", DEFAULT_SWEEP_DELAY_SEC), DEFAULT_SWEEP_DELAY_SEC))
    physical_deg_per_command_deg = _safe_float(
        calib_cfg.get("physical_deg_per_command_deg", DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG),
        DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG,
    )
    if physical_deg_per_command_deg <= 0:
        physical_deg_per_command_deg = DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG

    current_cmd_angle = _clamp_angle(state["command_angle_deg"])
    current_logical = _wrap_360(state["logical_azimuth_deg"])

    direction = str(args.direction)
    if direction == "cw":
        cmd_sign = 1.0 if cw_increases_angle else -1.0
        logical_sign = 1.0
    else:
        cmd_sign = -1.0 if cw_increases_angle else 1.0
        logical_sign = -1.0

    requested_cmd_delta = step_deg / physical_deg_per_command_deg
    target_cmd_angle = _clamp_angle(current_cmd_angle + cmd_sign * requested_cmd_delta)
    actual_cmd_delta = target_cmd_angle - current_cmd_angle
    if abs(actual_cmd_delta) < 1e-9:
        print("No movement (already at command-angle limit).")
        print(f"Command angle remains {current_cmd_angle:.1f}°")
        return 0

    # Adjust logical delta to reflect clamp-limited actual movement.
    actual_logical_delta = abs(actual_cmd_delta) * physical_deg_per_command_deg * logical_sign
    next_logical = _wrap_360(current_logical + actual_logical_delta)

    servo = None
    try:
        factory = LGPIOFactory()
        servo = Servo(
            pin,
            min_pulse_width=min_pulse,
            max_pulse_width=max_pulse,
            pin_factory=factory,
        )

        # Smooth incremental move like servotest sweep.
        angle = current_cmd_angle
        step_dir = 1.0 if target_cmd_angle > current_cmd_angle else -1.0
        while (step_dir > 0 and angle < target_cmd_angle) or (step_dir < 0 and angle > target_cmd_angle):
            angle = _clamp_angle(angle + step_dir * sweep_step)
            if (step_dir > 0 and angle > target_cmd_angle) or (step_dir < 0 and angle < target_cmd_angle):
                angle = target_cmd_angle
            servo.value = _angle_to_servo_value(angle)
            if sweep_delay > 0:
                time.sleep(sweep_delay)

        _save_state(STATE_FILE, next_logical, target_cmd_angle)

        print("Step executed.")
        print(f"  direction: {direction}")
        print(f"  requested physical step: {step_deg:.1f}°")
        print(f"  logical: {current_logical:.1f}° -> {next_logical:.1f}°")
        print(f"  command angle: {current_cmd_angle:.1f}° -> {target_cmd_angle:.1f}°")
        print(f"  command delta applied: {actual_cmd_delta:+.1f}°")
        print(f"  physical_deg_per_command_deg: {physical_deg_per_command_deg:.4f}")
        print(f"  pulse range: {min_pulse * 1e6:.0f}us -> {max_pulse * 1e6:.0f}us")
        print(f"  cw_increases_angle: {cw_increases_angle}")
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

#!/usr/bin/env python3
"""
Servo calibration helper using positional angle commands (servotest-style).

Commands:
  zero                     Set current logical azimuth to 0°
  sync <command_angle>     Sync internal command reference to current physical position
  step <cw|ccw> <degrees>  Move by N degrees in direction
  status                   Show logical azimuth + command angle estimate
  pulse <min_us> <max_us>  Override pulse range for calibration
  pulse reset              Clear override and use config pulse range
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
DEFAULT_FINAL_SETTLE_SEC = 0.15
DEFAULT_MIN_COMMAND_STEP_DEG = 0.0


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
    default = {
        "logical_azimuth_deg": 0.0,
        "command_angle_deg": DEFAULT_COMMAND_ANGLE,
        "pulse_min_width_s": None,
        "pulse_max_width_s": None,
    }
    try:
        with open(path, "r") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return default
        logical = _wrap_360(_safe_float(data.get("logical_azimuth_deg"), 0.0))
        command = _clamp_angle(_safe_float(data.get("command_angle_deg"), DEFAULT_COMMAND_ANGLE))
        pulse_min = data.get("pulse_min_width_s")
        pulse_max = data.get("pulse_max_width_s")
        pulse_min_f = _safe_float(pulse_min, -1.0) if pulse_min is not None else -1.0
        pulse_max_f = _safe_float(pulse_max, -1.0) if pulse_max is not None else -1.0
        if pulse_min_f <= 0 or pulse_max_f <= 0 or pulse_min_f >= pulse_max_f:
            pulse_min_f = None
            pulse_max_f = None
        return {
            "logical_azimuth_deg": logical,
            "command_angle_deg": command,
            "pulse_min_width_s": pulse_min_f,
            "pulse_max_width_s": pulse_max_f,
        }
    except Exception:
        return default


def _save_state(
    path: str,
    logical_azimuth_deg: float,
    command_angle_deg: float,
    pulse_min_width_s: float | None,
    pulse_max_width_s: float | None,
):
    state_dir = os.path.dirname(path)
    if state_dir:
        os.makedirs(state_dir, exist_ok=True)
    payload = {
        "logical_azimuth_deg": _wrap_360(logical_azimuth_deg),
        "command_angle_deg": _clamp_angle(command_angle_deg),
    }
    if (
        pulse_min_width_s is not None
        and pulse_max_width_s is not None
        and pulse_min_width_s > 0
        and pulse_max_width_s > pulse_min_width_s
    ):
        payload["pulse_min_width_s"] = float(pulse_min_width_s)
        payload["pulse_max_width_s"] = float(pulse_max_width_s)
    with open(path, "w") as f:
        json.dump(payload, f)


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


def _resolve_active_pulse_widths(
    servo_cfg: Dict[str, Any],
    state: Dict[str, Any],
) -> tuple[float, float, str]:
    cfg_min, cfg_max = _resolve_pulse_widths(servo_cfg)
    state_min = state.get("pulse_min_width_s")
    state_max = state.get("pulse_max_width_s")
    state_min_f = _safe_float(state_min, -1.0) if state_min is not None else -1.0
    state_max_f = _safe_float(state_max, -1.0) if state_max is not None else -1.0
    if state_min_f > 0 and state_max_f > state_min_f:
        return state_min_f, state_max_f, "state override"
    return cfg_min, cfg_max, "config"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Simple servo step calibrator (servotest-style positional control)."
    )
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("zero", help="Set current logical azimuth to 0°.")
    p_sync = sub.add_parser("sync", help="Set internal command-angle reference (no motor move).")
    p_sync.add_argument("command_angle", type=float, help="Command angle in degrees (0..180).")
    sub.add_parser("status", help="Show current logical azimuth and command angle.")
    p_step = sub.add_parser("step", help="Move by N degrees in chosen direction.")
    p_step.add_argument("direction", choices=["cw", "ccw"], help="Move direction.")
    p_step.add_argument("degrees", type=float, help="Step size in degrees (>0).")
    p_pulse = sub.add_parser("pulse", help="Set or reset pulse-width override.")
    p_pulse.add_argument("value1", help="min_us or 'reset'")
    p_pulse.add_argument("value2", nargs="?", help="max_us (required unless reset)")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    state = _load_state(STATE_FILE)
    cfg = _load_yaml(CONFIG_PATH)
    servo_cfg = _safe_dict(cfg.get("servo"))

    if args.command == "status":
        calib_cfg = _safe_dict(servo_cfg.get("calibration"))
        min_pulse, max_pulse, pulse_source = _resolve_active_pulse_widths(servo_cfg, state)
        physical_deg_per_command_deg = _safe_float(
            calib_cfg.get("physical_deg_per_command_deg", DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG),
            DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG,
        )
        if physical_deg_per_command_deg <= 0:
            physical_deg_per_command_deg = DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG
        min_command_step_deg = max(
            0.0,
            _safe_float(
                calib_cfg.get("min_command_step_deg", DEFAULT_MIN_COMMAND_STEP_DEG),
                DEFAULT_MIN_COMMAND_STEP_DEG,
            ),
        )
        final_settle_sec = max(
            0.0,
            _safe_float(
                calib_cfg.get("final_settle_sec", DEFAULT_FINAL_SETTLE_SEC),
                DEFAULT_FINAL_SETTLE_SEC,
            ),
        )
        print(f"Logical azimuth: {state['logical_azimuth_deg']:.1f}°")
        print(f"Command angle: {state['command_angle_deg']:.1f}°")
        print(f"physical_deg_per_command_deg: {physical_deg_per_command_deg:.4f}")
        print(f"min_command_step_deg: {min_command_step_deg:.3f}°")
        print(f"final_settle_sec: {final_settle_sec:.3f}s")
        print(f"Pulse range ({pulse_source}): {min_pulse * 1e6:.0f}us -> {max_pulse * 1e6:.0f}us")
        print(f"State file: {STATE_FILE}")
        return 0

    if args.command == "zero":
        _save_state(
            STATE_FILE,
            0.0,
            state["command_angle_deg"],
            state.get("pulse_min_width_s"),
            state.get("pulse_max_width_s"),
        )
        print("Zero set.")
        print("Current physical position is now logical 0.0°.")
        print(f"Command reference kept at {state['command_angle_deg']:.1f}°.")
        print(f"State file: {STATE_FILE}")
        return 0

    if args.command == "sync":
        synced = _clamp_angle(_safe_float(getattr(args, "command_angle", DEFAULT_COMMAND_ANGLE), DEFAULT_COMMAND_ANGLE))
        _save_state(
            STATE_FILE,
            state["logical_azimuth_deg"],
            synced,
            state.get("pulse_min_width_s"),
            state.get("pulse_max_width_s"),
        )
        print(f"Command reference synced to {synced:.1f}°.")
        print("No motor movement was commanded.")
        print(f"State file: {STATE_FILE}")
        return 0

    if args.command == "pulse":
        raw = str(getattr(args, "value1", "")).strip().lower()
        if raw == "reset":
            _save_state(
                STATE_FILE,
                state["logical_azimuth_deg"],
                state["command_angle_deg"],
                None,
                None,
            )
            cfg_min, cfg_max = _resolve_pulse_widths(servo_cfg)
            print("Pulse override cleared.")
            print(f"Active pulse range (config): {cfg_min * 1e6:.0f}us -> {cfg_max * 1e6:.0f}us")
            print(f"State file: {STATE_FILE}")
            return 0

        if getattr(args, "value2", None) is None:
            print("ERROR: pulse requires two values: pulse <min_us> <max_us>")
            return 2

        min_us = _safe_float(getattr(args, "value1"), -1.0)
        max_us = _safe_float(getattr(args, "value2"), -1.0)
        if min_us <= 0 or max_us <= 0 or min_us >= max_us:
            print("ERROR: pulse values must be > 0 and min_us < max_us")
            return 2

        min_s = min_us / 1_000_000.0
        max_s = max_us / 1_000_000.0
        _save_state(
            STATE_FILE,
            state["logical_azimuth_deg"],
            state["command_angle_deg"],
            min_s,
            max_s,
        )
        print("Pulse override set.")
        print(f"Active pulse range (state override): {min_us:.0f}us -> {max_us:.0f}us")
        print(f"State file: {STATE_FILE}")
        return 0

    if Servo is None or LGPIOFactory is None:
        print("ERROR: gpiozero/lgpio not available. Run this on Raspberry Pi.")
        return 1

    step_deg = _safe_float(getattr(args, "degrees", 0.0), 0.0)
    if step_deg <= 0:
        print("ERROR: step degrees must be > 0")
        return 2

    pin = _safe_int(servo_cfg.get("pin", 18), 18)
    min_pulse, max_pulse, pulse_source = _resolve_active_pulse_widths(servo_cfg, state)

    calib_cfg = _safe_dict(servo_cfg.get("calibration"))
    cw_increases_angle = _safe_bool(calib_cfg.get("cw_increases_angle", True), True)
    sweep_step = max(0.1, _safe_float(calib_cfg.get("sweep_step_deg", DEFAULT_SWEEP_STEP), DEFAULT_SWEEP_STEP))
    sweep_delay = max(0.0, _safe_float(calib_cfg.get("sweep_delay_sec", DEFAULT_SWEEP_DELAY_SEC), DEFAULT_SWEEP_DELAY_SEC))
    final_settle_sec = max(
        0.0,
        _safe_float(calib_cfg.get("final_settle_sec", DEFAULT_FINAL_SETTLE_SEC), DEFAULT_FINAL_SETTLE_SEC),
    )
    physical_deg_per_command_deg = _safe_float(
        calib_cfg.get("physical_deg_per_command_deg", DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG),
        DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG,
    )
    if physical_deg_per_command_deg <= 0:
        physical_deg_per_command_deg = DEFAULT_PHYSICAL_DEG_PER_COMMAND_DEG
    min_command_step_deg = max(
        0.0,
        _safe_float(
            calib_cfg.get("min_command_step_deg", DEFAULT_MIN_COMMAND_STEP_DEG),
            DEFAULT_MIN_COMMAND_STEP_DEG,
        ),
    )

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
    effective_cmd_delta = requested_cmd_delta
    if 0 < abs(requested_cmd_delta) < min_command_step_deg:
        effective_cmd_delta = min_command_step_deg
    target_cmd_angle = _clamp_angle(current_cmd_angle + cmd_sign * effective_cmd_delta)
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
        if final_settle_sec > 0:
            time.sleep(final_settle_sec)

        _save_state(
            STATE_FILE,
            next_logical,
            target_cmd_angle,
            state.get("pulse_min_width_s"),
            state.get("pulse_max_width_s"),
        )

        print("Step executed.")
        print(f"  direction: {direction}")
        print(f"  requested physical step: {step_deg:.1f}°")
        print(f"  logical: {current_logical:.1f}° -> {next_logical:.1f}°")
        print(f"  command angle: {current_cmd_angle:.1f}° -> {target_cmd_angle:.1f}°")
        print(f"  command delta applied: {actual_cmd_delta:+.1f}°")
        print(f"  physical_deg_per_command_deg: {physical_deg_per_command_deg:.4f}")
        print(f"  min_command_step_deg: {min_command_step_deg:.3f}°")
        print(f"  final_settle_sec: {final_settle_sec:.3f}s")
        print(f"  pulse range ({pulse_source}): {min_pulse * 1e6:.0f}us -> {max_pulse * 1e6:.0f}us")
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

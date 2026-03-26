#!/usr/bin/env python3
"""
Two-pass scheduling experiment for robust 3D scanning.

This module keeps the existing slice capture logic intact, but changes how scan
time is allocated across the sweep:

1. Pass 1 captures every angle with a smaller baseline scan budget.
2. Pass 2 revisits only weak slices using a larger capture budget.

The goal is to test whether scheduling scan time adaptively across angles can
reduce total runtime without materially degrading point-cloud quality.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from collections import Counter
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import yaml

base_scan = None


def _require_base_scan():
    global base_scan
    if base_scan is None:
        import robust_3d_scan_module as imported_base_scan

        base_scan = imported_base_scan
    return base_scan


EXPERIMENT_DEFAULTS = {
    "policy": "two_pass_adaptive_revisit",
    "pass1_max_scans": None,
    "pass1_min_scans": None,
    "revisit_max_scans": 19,
    "revisit_min_scans": None,
    "revisit_if_coverage_below": 45.0,
    "revisit_if_point_count_below": 170,
    "revisit_top_n": 0,
    "revisit_order": "weakest_first",
}


def _resolve_experiment_config(
    scan_config: Optional[Dict[str, Any]],
    experiment_config: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    bs = _require_base_scan()
    scan_config = scan_config or {}
    merged = dict(EXPERIMENT_DEFAULTS)
    merged.update(experiment_config or {})

    baseline_min = int(scan_config.get("min_scans", bs.DEFAULTS["min_scans"]))
    baseline_max = int(scan_config.get("max_scans", bs.DEFAULTS["max_scans"]))

    pass1_max = merged.get("pass1_max_scans")
    if pass1_max is None:
        pass1_max = baseline_min
    pass1_max = int(pass1_max)

    pass1_min = merged.get("pass1_min_scans")
    if pass1_min is None:
        pass1_min = min(baseline_min, pass1_max)
    pass1_min = int(pass1_min)

    revisit_max = merged.get("revisit_max_scans")
    if revisit_max is None:
        revisit_max = min(baseline_max, max(baseline_min, 19))
    revisit_max = int(revisit_max)

    revisit_min = merged.get("revisit_min_scans")
    if revisit_min is None:
        revisit_min = baseline_min
    revisit_min = int(revisit_min)

    merged["pass1_max_scans"] = max(1, pass1_max)
    merged["pass1_min_scans"] = max(1, min(pass1_min, merged["pass1_max_scans"]))
    merged["revisit_max_scans"] = max(1, revisit_max)
    merged["revisit_min_scans"] = max(1, min(revisit_min, merged["revisit_max_scans"]))
    merged["revisit_if_coverage_below"] = float(merged["revisit_if_coverage_below"])
    merged["revisit_if_point_count_below"] = int(merged["revisit_if_point_count_below"])
    merged["revisit_top_n"] = max(0, int(merged["revisit_top_n"]))
    merged["revisit_order"] = str(merged["revisit_order"]).strip().lower() or "weakest_first"
    return merged


def _build_capture_config(
    scan_config: Dict[str, Any],
    *,
    max_scans: int,
    min_scans: int,
) -> Dict[str, Any]:
    bs = _require_base_scan()
    return {
        "max_scans": max_scans,
        "min_scans": min_scans,
        "plateau_iters": scan_config.get("plateau_iters", bs.DEFAULTS["plateau_iters"]),
        "plateau_tol": scan_config.get("plateau_tol", bs.DEFAULTS["plateau_tol"]),
        "bin_deg": scan_config.get("bin_deg", bs.DEFAULTS["bin_deg"]),
        "min_dist": scan_config.get("min_dist", bs.DEFAULTS["min_dist"]),
        "max_dist": scan_config.get("max_dist", bs.DEFAULTS["max_dist"]),
    }


def _reset_lidar_for_slice(lidar: Any) -> None:
    try:
        lidar.stop()
        time.sleep(0.5)
        if hasattr(lidar, "clean_input"):
            lidar.clean_input()
        elif hasattr(lidar, "_serial_port"):
            lidar._serial_port.reset_input_buffer()
        time.sleep(0.5)
    except Exception as exc:
        print(f"Lidar reset warning: {exc}")


def _project_slice_points(
    slice_points_2d: List[Tuple[float, float, float]],
    servo_angle_deg: float,
) -> List[Tuple[float, float, float, float, float]]:
    points_3d: List[Tuple[float, float, float, float, float]] = []
    beta = math.radians(-float(servo_angle_deg))

    for qual, lidar_angle_deg, r_mm in slice_points_2d:
        r_m = r_mm / 1000.0
        alpha = math.radians(lidar_angle_deg)

        x = r_m * math.cos(alpha)
        z = r_m * math.sin(alpha)
        y = 0.0

        y2 = y * math.cos(beta) - z * math.sin(beta)
        z2 = y * math.sin(beta) + z * math.cos(beta)
        points_3d.append((x, y2, z2, r_m, qual))

    return points_3d


def _save_cloud_files(
    output_dir: str,
    points: List[Tuple[float, float, float, float, float]],
    prefix: str,
    file_callback: Callable[[str], None],
) -> List[str]:
    generated: List[str] = []

    ply_path = os.path.join(output_dir, f"{prefix}.ply")
    with open(ply_path, "w", encoding="utf-8") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {len(points)}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property float intensity\n")
        handle.write("end_header\n")
        for p in points:
            handle.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[4]}\n")
    generated.append(ply_path)
    file_callback(ply_path)

    csv_path = os.path.join(output_dir, f"{prefix}.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["x", "y", "z", "distance", "quality"])
        for p in points:
            writer.writerow(
                [f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}", f"{p[3]:.4f}", int(p[4])]
            )
    generated.append(csv_path)
    file_callback(csv_path)
    return generated


def _revisit_sort_key(result: Dict[str, Any]) -> Tuple[float, int, float, float]:
    analysis = result["analysis"]
    return (
        float(analysis.get("coverage_percent", 0.0)),
        int(analysis.get("point_count", 0)),
        -float(analysis.get("scan_count", 0)),
        -float(analysis.get("elapsed_ms", 0.0)),
    )


def _select_revisit_indices(
    pass1_results: Dict[int, Dict[str, Any]],
    experiment_config: Dict[str, Any],
) -> Tuple[List[int], Dict[int, List[str]], Dict[str, int]]:
    selected: set[int] = set()
    reasons_by_index: Dict[int, List[str]] = {}

    coverage_threshold = float(experiment_config["revisit_if_coverage_below"])
    point_threshold = int(experiment_config["revisit_if_point_count_below"])

    for index, result in pass1_results.items():
        analysis = result["analysis"]
        reasons: List[str] = []
        if float(analysis.get("coverage_percent", 0.0)) < coverage_threshold:
            reasons.append("low_coverage")
        if int(analysis.get("point_count", 0)) < point_threshold:
            reasons.append("low_point_count")
        if reasons:
            selected.add(index)
            reasons_by_index.setdefault(index, []).extend(reasons)

    revisit_top_n = int(experiment_config["revisit_top_n"])
    if revisit_top_n > 0:
        ranked = sorted(pass1_results.items(), key=lambda item: _revisit_sort_key(item[1]))
        for index, _ in ranked[:revisit_top_n]:
            selected.add(index)
            reasons_by_index.setdefault(index, []).append("top_n_weakest")

    order = str(experiment_config["revisit_order"]).strip().lower()
    if order == "original_order":
        ordered = sorted(selected)
    else:
        ordered = [
            index
            for index, _ in sorted(
                ((index, pass1_results[index]) for index in selected),
                key=lambda item: _revisit_sort_key(item[1]),
            )
        ]

    reason_counts = Counter()
    for reasons in reasons_by_index.values():
        reason_counts.update(reasons)
    return ordered, reasons_by_index, dict(reason_counts)


def _capture_slice_at_angle(
    *,
    servo: Any,
    lidar: Any,
    lidar_port: str,
    motor_pwm: int,
    servo_angle: float,
    slice_index: int,
    total_slices: int,
    pass_label: str,
    capture_config: Dict[str, Any],
    retry_budget: int,
    reinit_spinup_s: float,
    servo_settle: float,
    progress_callback: Callable[[Dict[str, Any]], None],
) -> Tuple[Any, List[Tuple[float, float, float]], Dict[str, Any]]:
    bs = _require_base_scan()
    progress_callback(
        {
            "stage": "scanning",
            "pass_label": pass_label,
            "message": f"{pass_label}: scanning slice {slice_index + 1}/{total_slices} at {servo_angle:.1f}°",
            "slice_index": slice_index,
            "total_slices": total_slices,
        }
    )

    servo.set_angle(servo_angle)
    time.sleep(servo_settle)
    _reset_lidar_for_slice(lidar)

    attempt = 0
    slice_attempts: List[Dict[str, Any]] = []
    slice_points_2d: List[Tuple[float, float, float]] = []
    while True:
        slice_points_2d, had_io_error, slice_stats = bs.capture_robust_slice(
            lidar,
            f"{pass_label}_slice_{slice_index}",
            capture_config,
        )
        slice_attempts.append(slice_stats)
        if not had_io_error:
            break

        if attempt >= retry_budget:
            raise RuntimeError(
                f"LiDAR I/O error persisted in {pass_label} slice {slice_index + 1} after {attempt} retries"
            )

        attempt += 1
        progress_callback(
            {
                "stage": "recovering",
                "pass_label": pass_label,
                "message": (
                    f"{pass_label}: LiDAR I/O error in slice {slice_index + 1}; "
                    f"reinitializing and retrying ({attempt}/{retry_budget})"
                ),
                "slice_index": slice_index,
                "total_slices": total_slices,
            }
        )
        bs._safe_lidar_teardown(lidar)
        lidar = bs._init_lidar(
            lidar_port=lidar_port,
            baudrate=bs.DEFAULTS["lidar_baudrate"],
            motor_pwm=motor_pwm,
            spinup_s=reinit_spinup_s,
        )

    analysis = dict(slice_attempts[-1]) if slice_attempts else {}
    analysis.update(
        {
            "slice_index": int(slice_index),
            "servo_angle_deg": round(float(servo_angle), 3),
            "pass_label": pass_label,
            "attempt_count": int(len(slice_attempts)),
            "retry_count": int(max(0, len(slice_attempts) - 1)),
            "attempts": slice_attempts,
        }
    )

    progress_callback(
        {
            "stage": "slice_analysis",
            "pass_label": pass_label,
            "message": (
                f"{pass_label}: slice {slice_index + 1}/{total_slices} at {servo_angle:.1f}° -> "
                f"{analysis.get('scan_count', 0)} scans, "
                f"{analysis.get('coverage_percent', 0.0):.1f}% coverage, "
                f"stop={analysis.get('termination_reason', 'unknown')}"
            ),
            "slice_index": slice_index,
            "total_slices": total_slices,
            "slice_analysis": analysis,
        }
    )
    return lidar, slice_points_2d, analysis


def run_scan(
    port: str = "auto",
    output_dir: str = "data",
    servo_config: Optional[Dict[str, Any]] = None,
    scan_config: Optional[Dict[str, Any]] = None,
    experiment_config: Optional[Dict[str, Any]] = None,
    should_stop: Optional[Callable[[], bool]] = None,
    progress_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    file_callback: Optional[Callable[[str], None]] = None,
) -> Dict[str, Any]:
    bs = _require_base_scan()
    if should_stop is None:
        should_stop = lambda: False
    if progress_callback is None:
        progress_callback = lambda payload: None
    if file_callback is None:
        file_callback = lambda path: None

    servo_config = servo_config or {}
    scan_config = scan_config or {}
    experiment_config = _resolve_experiment_config(scan_config, experiment_config)

    lidar_port = port if port != "auto" else bs.get_default_port()
    servo_port_cfg = servo_config.get("serial_port", bs.DEFAULTS["servo_serial_port"])
    servo_port = bs.get_default_servo_port(servo_port_cfg)

    servo_baud = int(servo_config.get("baudrate", bs.DEFAULTS["servo_baudrate"]))
    servo_timeout = float(servo_config.get("timeout", bs.DEFAULTS["servo_timeout"]))
    servo_settle = float(servo_config.get("settle_time", bs.DEFAULTS["servo_settle_time"]))
    base_io_retries = int(scan_config.get("slice_io_retries", 1))
    first_slice_extra_retries = int(scan_config.get("first_slice_extra_io_retries", 1))
    reinit_spinup_s = float(scan_config.get("io_reinit_spinup_s", 2.0))
    motor_pwm = int(scan_config.get("motor_pwm", bs.DEFAULTS["lidar_pwm"]))

    sweep_start = float(scan_config.get("sweep_start", bs.DEFAULTS["sweep_start"]))
    sweep_end = float(scan_config.get("sweep_end", bs.DEFAULTS["sweep_end"]))
    num_steps = int(scan_config.get("num_steps", bs.DEFAULTS["num_steps"]))
    park_after_scan = bool(scan_config.get("park_after_scan", bs.DEFAULTS["park_after_scan"]))
    park_target_raw = scan_config.get("park_target_deg", bs.DEFAULTS["park_target_deg"])
    park_target_deg = None if park_target_raw in (None, "") else float(park_target_raw)
    park_backoff_deg = abs(float(scan_config.get("park_backoff_deg", bs.DEFAULTS["park_backoff_deg"])))
    park_settle_time = float(scan_config.get("park_settle_time", bs.DEFAULTS["park_settle_time"]))

    pass1_capture_config = _build_capture_config(
        scan_config,
        max_scans=int(experiment_config["pass1_max_scans"]),
        min_scans=int(experiment_config["pass1_min_scans"]),
    )
    revisit_capture_config = _build_capture_config(
        scan_config,
        max_scans=int(experiment_config["revisit_max_scans"]),
        min_scans=int(experiment_config["revisit_min_scans"]),
    )

    os.makedirs(output_dir, exist_ok=True)
    steps = np.linspace(sweep_start, sweep_end, num_steps)

    servo = None
    lidar = None
    generated_files: List[str] = []
    pass1_results: Dict[int, Dict[str, Any]] = {}
    pass2_results: Dict[int, Dict[str, Any]] = {}
    stopped = False
    processed_points: List[Tuple[float, float, float, float, float]] = []
    last_servo_angle: Optional[float] = None
    pass1_elapsed_ms = 0.0
    pass2_elapsed_ms = 0.0
    selection_reason_counts: Dict[str, int] = {}
    revisit_indices: List[int] = []
    reasons_by_index: Dict[int, List[str]] = {}

    try:
        progress_callback(
            {
                "stage": "init",
                "message": "Initializing scheduling experiment hardware...",
                "policy": experiment_config["policy"],
            }
        )

        print(f"Connecting to servo on {servo_port} @ {servo_baud}...")
        servo = bs.PicoServoController(servo_port, baudrate=servo_baud, timeout=servo_timeout)
        servo.set_policy("ROBUST_3D")

        lidar = bs._init_lidar(
            lidar_port=lidar_port,
            baudrate=bs.DEFAULTS["lidar_baudrate"],
            motor_pwm=motor_pwm,
            spinup_s=3.0,
        )

        progress_callback(
            {
                "stage": "schedule_policy",
                "message": (
                    "Running two-pass adaptive revisit scheduling: "
                    f"pass1 max_scans={experiment_config['pass1_max_scans']}, "
                    f"revisit max_scans={experiment_config['revisit_max_scans']}"
                ),
                "experiment_config": experiment_config,
            }
        )

        pass1_started = time.perf_counter()
        for index, servo_angle in enumerate(steps):
            if should_stop():
                stopped = True
                break
            retry_budget = base_io_retries + (first_slice_extra_retries if index == 0 else 0)
            lidar, slice_points_2d, analysis = _capture_slice_at_angle(
                servo=servo,
                lidar=lidar,
                lidar_port=lidar_port,
                motor_pwm=motor_pwm,
                servo_angle=float(servo_angle),
                slice_index=index,
                total_slices=len(steps),
                pass_label="pass1",
                capture_config=pass1_capture_config,
                retry_budget=retry_budget,
                reinit_spinup_s=reinit_spinup_s,
                servo_settle=servo_settle,
                progress_callback=progress_callback,
            )
            last_servo_angle = float(servo_angle)
            pass1_results[index] = {
                "angle_deg": float(servo_angle),
                "points_2d": slice_points_2d,
                "analysis": analysis,
                "pass_label": "pass1",
            }
        pass1_elapsed_ms = round((time.perf_counter() - pass1_started) * 1000.0, 3)

        if not stopped:
            revisit_indices, reasons_by_index, selection_reason_counts = _select_revisit_indices(
                pass1_results,
                experiment_config,
            )
            progress_callback(
                {
                    "stage": "revisit_selection",
                    "message": (
                        f"Selected {len(revisit_indices)} slices for revisit out of {len(pass1_results)}"
                    ),
                    "selected_slice_indices": revisit_indices,
                    "selection_reason_counts": selection_reason_counts,
                }
            )

            pass2_started = time.perf_counter()
            for index in revisit_indices:
                if should_stop():
                    stopped = True
                    break
                servo_angle = float(pass1_results[index]["angle_deg"])
                retry_budget = base_io_retries
                lidar, slice_points_2d, analysis = _capture_slice_at_angle(
                    servo=servo,
                    lidar=lidar,
                    lidar_port=lidar_port,
                    motor_pwm=motor_pwm,
                    servo_angle=servo_angle,
                    slice_index=index,
                    total_slices=len(steps),
                    pass_label="pass2",
                    capture_config=revisit_capture_config,
                    retry_budget=retry_budget,
                    reinit_spinup_s=reinit_spinup_s,
                    servo_settle=servo_settle,
                    progress_callback=progress_callback,
                )
                last_servo_angle = servo_angle
                pass2_results[index] = {
                    "angle_deg": servo_angle,
                    "points_2d": slice_points_2d,
                    "analysis": analysis,
                    "pass_label": "pass2",
                }
            pass2_elapsed_ms = round((time.perf_counter() - pass2_started) * 1000.0, 3)

        all_points_3d: List[Tuple[float, float, float, float, float]] = []
        final_slice_analyses: List[Dict[str, Any]] = []

        for index, servo_angle in enumerate(steps):
            if index not in pass1_results:
                continue
            final_result = pass2_results.get(index, pass1_results[index])
            points_3d = _project_slice_points(final_result["points_2d"], float(servo_angle))
            all_points_3d.extend(points_3d)

            final_analysis = dict(final_result["analysis"])
            final_analysis["selected_for_revisit"] = bool(index in pass2_results)
            final_analysis["selection_reasons"] = list(reasons_by_index.get(index, []))
            final_analysis["final_pass"] = final_result["pass_label"]
            final_analysis["pass1_analysis"] = pass1_results[index]["analysis"]
            if index in pass2_results:
                final_analysis["pass2_analysis"] = pass2_results[index]["analysis"]
            final_slice_analyses.append(final_analysis)

        if all_points_3d:
            raw_points = all_points_3d
            processed_points = raw_points

            voxel_size_m = float(scan_config.get("voxel_size_m", 0.0))
            if voxel_size_m > 0.0:
                before = len(processed_points)
                processed_points = bs._voxel_downsample_points(processed_points, voxel_size_m)
                progress_callback(
                    {
                        "stage": "filtering",
                        "message": (
                            f"Voxel downsample ({voxel_size_m:.4f}m): "
                            f"{before} -> {len(processed_points)} points"
                        ),
                        "point_count": len(processed_points),
                    }
                )

            sor_neighbors = int(scan_config.get("sor_neighbors", 0))
            sor_std_ratio = float(scan_config.get("sor_std_ratio", 0.0))
            sor_radius_m = float(scan_config.get("sor_radius_m", 0.0))
            sor_max_points = int(scan_config.get("sor_max_points", 12000))
            sor_max_runtime_s = float(scan_config.get("sor_max_runtime_s", 2.5))
            sor_note = "sor_disabled"
            if sor_neighbors > 0 and sor_std_ratio > 0.0 and sor_radius_m > 0.0:
                before = len(processed_points)
                processed_points, sor_note = bs._sor_filter_points(
                    processed_points,
                    neighbors=sor_neighbors,
                    std_ratio=sor_std_ratio,
                    radius_m=sor_radius_m,
                    max_points=sor_max_points,
                    max_runtime_s=sor_max_runtime_s,
                )
                progress_callback(
                    {
                        "stage": "filtering",
                        "message": (
                            f"SOR ({sor_note}): {before} -> {len(processed_points)} points "
                            f"(k={sor_neighbors}, std={sor_std_ratio:.2f}, r={sor_radius_m:.3f}m)"
                        ),
                        "point_count": len(processed_points),
                    }
                )

            progress_callback({"stage": "saving", "message": "Saving scheduling experiment point cloud..."})
            generated_files.extend(
                _save_cloud_files(
                    output_dir=output_dir,
                    points=processed_points,
                    prefix="schedule_experiment_full",
                    file_callback=file_callback,
                )
            )

        return {
            "success": True,
            "stopped": stopped,
            "point_count": len(processed_points) if all_points_3d else 0,
            "files": generated_files,
            "error": None,
            "message": "Scheduling experiment completed successfully",
            "scan_quality": {
                "raw_points": len(all_points_3d),
                "final_points": len(processed_points) if all_points_3d else 0,
                "voxel_size_m": float(scan_config.get("voxel_size_m", 0.0)),
                "sor_neighbors": int(scan_config.get("sor_neighbors", 0)),
                "sor_std_ratio": float(scan_config.get("sor_std_ratio", 0.0)),
                "sor_radius_m": float(scan_config.get("sor_radius_m", 0.0)),
                "slice_analysis": final_slice_analyses,
                "schedule_experiment": {
                    "policy": experiment_config["policy"],
                    "pass1_elapsed_ms": pass1_elapsed_ms,
                    "pass2_elapsed_ms": pass2_elapsed_ms,
                    "pass1_capture_config": pass1_capture_config,
                    "revisit_capture_config": revisit_capture_config,
                    "selected_for_revisit_count": len(revisit_indices),
                    "selected_for_revisit_indices": revisit_indices,
                    "selection_reason_counts": selection_reason_counts,
                },
            },
        }
    except Exception as exc:
        return {
            "success": False,
            "stopped": stopped,
            "point_count": len(processed_points),
            "files": generated_files,
            "error": str(exc),
            "message": f"Scheduling experiment failed: {exc}",
            "scan_quality": {
                "slice_analysis": [
                    result["analysis"] for _, result in sorted(pass1_results.items(), key=lambda item: item[0])
                ],
                "schedule_experiment": {
                    "policy": experiment_config["policy"],
                    "pass1_elapsed_ms": pass1_elapsed_ms,
                    "pass2_elapsed_ms": pass2_elapsed_ms,
                    "selected_for_revisit_count": len(revisit_indices),
                    "selected_for_revisit_indices": revisit_indices,
                    "selection_reason_counts": selection_reason_counts,
                },
            },
        }
    finally:
        if (
            servo
            and park_after_scan
            and last_servo_angle is not None
            and (park_target_deg is not None or park_backoff_deg > 0.0)
            and len(steps) >= 1
        ):
            try:
                lower = min(sweep_start, sweep_end)
                upper = max(sweep_start, sweep_end)
                if park_target_deg is not None:
                    park_target = max(lower, min(upper, park_target_deg))
                else:
                    direction = 0.0
                    if len(steps) >= 2:
                        direction = float(steps[-1] - steps[0])
                    if direction > 0.0:
                        park_target = max(lower, min(upper, last_servo_angle - park_backoff_deg))
                    elif direction < 0.0:
                        park_target = max(lower, min(upper, last_servo_angle + park_backoff_deg))
                    else:
                        park_target = last_servo_angle

                if abs(park_target - last_servo_angle) > 1e-6:
                    progress_callback(
                        {
                            "stage": "parking",
                            "message": f"Parking servo at {park_target:.1f}° after scheduling experiment",
                        }
                    )
                    servo.set_angle(park_target)
                    if park_settle_time > 0.0:
                        time.sleep(park_settle_time)
            except Exception as exc:
                print(f"Servo park warning: {exc}")

        bs._safe_lidar_teardown(lidar)
        if servo:
            servo.detach()


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the two-pass scheduling experiment scan.")
    parser.add_argument("--config", default="config_rpi.yaml", help="Path to the Pi config file.")
    parser.add_argument("--port", default="auto", help='Serial port to use, or "auto".')
    parser.add_argument("--output-dir", default=None, help="Override the output directory.")
    parser.add_argument("--json", action="store_true", help="Print the full result as JSON.")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    config = _load_yaml(Path(args.config))
    output_dir = args.output_dir or config.get("data", {}).get("output_dir", "data")
    result = run_scan(
        port=args.port,
        output_dir=output_dir,
        servo_config=config.get("servo", {}),
        scan_config=config.get("scan3d", {}),
        experiment_config=config.get("schedule_experiment", {}),
    )
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(result.get("message"))
        summary = result.get("scan_quality", {}).get("schedule_experiment", {})
        print(f"Revisited slices: {summary.get('selected_for_revisit_count', 0)}")
        print(f"Pass 1 elapsed ms: {summary.get('pass1_elapsed_ms')}")
        print(f"Pass 2 elapsed ms: {summary.get('pass2_elapsed_ms')}")
        print(f"Files: {result.get('files', [])}")
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())

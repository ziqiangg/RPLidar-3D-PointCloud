#!/usr/bin/env python3
"""
Profile local scan execution on the Raspberry Pi or end-to-end scan flow over MQTT.

Two profiling modes are supported:

1. local
   Runs the scan module directly and records repeatable timing reports.
   This is the mode to use on the Raspberry Pi when profiling edge execution.

2. mqtt
   Sends a scan request over MQTT and records end-to-end timing from the laptop.
   This is the mode to use when profiling command, transfer, and receive latency.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
import traceback
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import yaml


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def _max_rss_mb() -> Optional[float]:
    try:
        import resource
    except Exception:
        return None

    value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    if sys.platform == "darwin":
        return round(float(value) / (1024.0 * 1024.0), 3)
    return round(float(value) / 1024.0, 3)


def _stat_summary(values: List[float]) -> Dict[str, Any]:
    if not values:
        return {"count": 0}

    ordered = sorted(float(v) for v in values)
    return {
        "count": len(ordered),
        "min_ms": round(ordered[0], 3),
        "median_ms": round(statistics.median(ordered), 3),
        "mean_ms": round(statistics.fmean(ordered), 3),
        "max_ms": round(ordered[-1], 3),
    }


def _summarize_named_events(events: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    grouped: Dict[str, List[float]] = {}
    for event in events:
        name = str(event.get("name", "") or "")
        elapsed_ms = event.get("elapsed_ms")
        if not name or elapsed_ms is None:
            continue
        grouped.setdefault(name, []).append(float(elapsed_ms))
    return {name: _stat_summary(values) for name, values in sorted(grouped.items())}


def _file_info(path_str: str) -> Dict[str, Any]:
    path = Path(path_str)
    info = {
        "path": str(path),
        "exists": path.exists(),
    }
    if path.exists():
        info["size_bytes"] = path.stat().st_size
    return info


def _write_report(profile_dir: Path, prefix: str, payload: Dict[str, Any]) -> Path:
    _ensure_dir(profile_dir)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = profile_dir / f"{prefix}_{stamp}.json"
    with output.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return output


@dataclass
class RunContext:
    started_at_wall: str
    started_at_perf: float
    started_cpu: float
    started_rss_mb: Optional[float]


def _start_run_context() -> RunContext:
    return RunContext(
        started_at_wall=_utc_now_iso(),
        started_at_perf=time.perf_counter(),
        started_cpu=time.process_time(),
        started_rss_mb=_max_rss_mb(),
    )


def _finish_run_context(ctx: RunContext) -> Dict[str, Any]:
    ended_perf = time.perf_counter()
    return {
        "started_at": ctx.started_at_wall,
        "ended_at": _utc_now_iso(),
        "elapsed_ms": round((ended_perf - ctx.started_at_perf) * 1000.0, 3),
        "cpu_time_ms": round((time.process_time() - ctx.started_cpu) * 1000.0, 3),
        "max_rss_mb": _max_rss_mb(),
    }


def _patch_attr(target: Any, attr: str, replacement: Any, originals: List[Any]) -> None:
    originals.append((target, attr, getattr(target, attr)))
    setattr(target, attr, replacement)


def _restore_patches(originals: List[Any]) -> None:
    for target, attr, original in reversed(originals):
        setattr(target, attr, original)


def _record_call(
    name: str,
    run_perf_start: float,
    events: List[Dict[str, Any]],
    fn: Callable[..., Any],
    meta_factory: Optional[Callable[[Any, tuple, dict], Dict[str, Any]]] = None,
) -> Callable[..., Any]:
    def wrapped(*args: Any, **kwargs: Any) -> Any:
        event: Dict[str, Any] = {
            "name": name,
            "offset_ms": round((time.perf_counter() - run_perf_start) * 1000.0, 3),
        }
        t0 = time.perf_counter()
        try:
            result = fn(*args, **kwargs)
            if meta_factory is not None:
                try:
                    event.update(meta_factory(result, args, kwargs))
                except Exception as exc:
                    event["meta_error"] = str(exc)
            return result
        except Exception as exc:
            event["error"] = str(exc)
            raise
        finally:
            event["elapsed_ms"] = round((time.perf_counter() - t0) * 1000.0, 3)
            events.append(event)

    return wrapped


def _run_local_once(
    scan_type: str,
    port: str,
    config_path: Path,
    output_dir_override: Optional[Path],
) -> Dict[str, Any]:
    config = _load_yaml(config_path)
    data_dir = Path(
        output_dir_override
        if output_dir_override is not None
        else config.get("data", {}).get("output_dir", "data")
    )
    _ensure_dir(data_dir)

    run_ctx = _start_run_context()
    originals: List[Any] = []
    wrapped_events: List[Dict[str, Any]] = []
    progress_events: List[Dict[str, Any]] = []

    try:
        if scan_type == "2d":
            import dump_one_scan as scan_module

            _patch_attr(
                scan_module,
                "merge_scans_robust",
                _record_call(
                    "merge_scans_robust",
                    run_ctx.started_at_perf,
                    wrapped_events,
                    scan_module.merge_scans_robust,
                    meta_factory=lambda result, args, kwargs: {
                        "merged_points": len(result[0]),
                        "hit_bins": len(result[1]),
                    },
                ),
                originals,
            )
            _patch_attr(
                scan_module,
                "validate_scan_quality_bins",
                _record_call(
                    "validate_scan_quality_bins",
                    run_ctx.started_at_perf,
                    wrapped_events,
                    scan_module.validate_scan_quality_bins,
                    meta_factory=lambda result, args, kwargs: {
                        "is_valid": bool(result[0]),
                        "coverage_percent": round(float(result[1]) * 100.0, 3),
                        "max_gap_degrees": round(float(result[2]), 3),
                        "point_count": int(result[3]),
                    },
                ),
                originals,
            )
            _patch_attr(
                scan_module,
                "fill_small_gaps_for_visualization",
                _record_call(
                    "fill_small_gaps_for_visualization",
                    run_ctx.started_at_perf,
                    wrapped_events,
                    scan_module.fill_small_gaps_for_visualization,
                    meta_factory=lambda result, args, kwargs: {
                        "filled_points": len(result),
                    },
                ),
                originals,
            )

            result = scan_module.run_scan(port=port, output_dir=str(data_dir))

        elif scan_type == "robust_3d":
            import robust_3d_scan_module as scan_module

            _patch_attr(
                scan_module,
                "_init_lidar",
                _record_call(
                    "init_lidar",
                    run_ctx.started_at_perf,
                    wrapped_events,
                    scan_module._init_lidar,
                ),
                originals,
            )
            _patch_attr(
                scan_module.PicoServoController,
                "set_angle",
                _record_call(
                    "servo_set_angle",
                    run_ctx.started_at_perf,
                    wrapped_events,
                    scan_module.PicoServoController.set_angle,
                    meta_factory=lambda result, args, kwargs: {
                        "requested_angle": float(args[1]) if len(args) > 1 else None,
                    },
                ),
                originals,
            )
            _patch_attr(
                scan_module,
                "capture_robust_slice",
                _record_call(
                    "capture_robust_slice",
                    run_ctx.started_at_perf,
                    wrapped_events,
                    scan_module.capture_robust_slice,
                    meta_factory=lambda result, args, kwargs: {
                        "slice_name": str(args[1]) if len(args) > 1 else None,
                        "point_count": len(result[0]),
                        "had_io_error": bool(result[1]),
                    },
                ),
                originals,
            )
            _patch_attr(
                scan_module,
                "_voxel_downsample_points",
                _record_call(
                    "voxel_downsample",
                    run_ctx.started_at_perf,
                    wrapped_events,
                    scan_module._voxel_downsample_points,
                    meta_factory=lambda result, args, kwargs: {
                        "before_points": len(args[0]) if args else None,
                        "after_points": len(result),
                    },
                ),
                originals,
            )
            _patch_attr(
                scan_module,
                "_sor_filter_points",
                _record_call(
                    "sor_filter",
                    run_ctx.started_at_perf,
                    wrapped_events,
                    scan_module._sor_filter_points,
                    meta_factory=lambda result, args, kwargs: {
                        "before_points": len(args[0]) if args else None,
                        "after_points": len(result[0]),
                        "note": str(result[1]),
                    },
                ),
                originals,
            )

            def progress_callback(payload: Dict[str, Any]) -> None:
                event = dict(payload)
                event["offset_ms"] = round(
                    (time.perf_counter() - run_ctx.started_at_perf) * 1000.0,
                    3,
                )
                progress_events.append(event)

            result = scan_module.run_scan(
                port=port,
                output_dir=str(data_dir),
                servo_config=config.get("servo", {}),
                scan_config=config.get("scan3d", {}),
                progress_callback=progress_callback,
            )

        else:
            raise ValueError(f"Unsupported local scan type: {scan_type}")

        run = _finish_run_context(run_ctx)
        run["mode"] = "local"
        run["scan_type"] = scan_type
        run["port"] = port
        run["config_path"] = str(config_path)
        run["output_dir"] = str(data_dir)
        run["result"] = result
        run["wrapped_events"] = wrapped_events
        run["progress_events"] = progress_events
        run["stage_summary"] = _summarize_named_events(wrapped_events)
        run["files"] = [_file_info(path) for path in result.get("files", [])]
        return run

    finally:
        _restore_patches(originals)


def _wait_for_connection(client: Any, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if client.is_connected():
            return
        time.sleep(0.05)
    raise TimeoutError(f"MQTT client did not connect within {timeout_s:.1f}s")


def _run_mqtt_once(
    scan_type: str,
    port: str,
    config_path: Path,
    timeout_s: float,
) -> Dict[str, Any]:
    from laptop_viewer_client import LaptopViewerClient

    run_ctx = _start_run_context()
    state: Dict[str, Any] = {
        "scan_id": None,
        "first_status_offset_ms": None,
        "terminal_status": None,
        "terminal_status_offset_ms": None,
        "data_received_offset_ms": None,
        "status_events": [],
        "received_files": [],
    }

    client = LaptopViewerClient(str(config_path))

    def on_status(scan_id: str, status: Any) -> None:
        if state["scan_id"] is None or scan_id != state["scan_id"]:
            return

        offset_ms = round((time.perf_counter() - run_ctx.started_at_perf) * 1000.0, 3)
        state["status_events"].append(
            {
                "offset_ms": offset_ms,
                "status": status.status,
                "message": status.message,
                "point_count": status.point_count,
            }
        )
        if state["first_status_offset_ms"] is None:
            state["first_status_offset_ms"] = offset_ms
        if status.status in {"completed", "error", "stopped"}:
            state["terminal_status"] = status.status
            state["terminal_status_offset_ms"] = offset_ms

    def on_data(scan_id: str, files: List[str]) -> None:
        if state["scan_id"] is None or scan_id != state["scan_id"]:
            return
        state["data_received_offset_ms"] = round(
            (time.perf_counter() - run_ctx.started_at_perf) * 1000.0,
            3,
        )
        state["received_files"] = list(files)

    client.set_status_callback(on_status)
    client.set_data_callback(on_data)

    if not client.connect():
        raise RuntimeError("Failed to connect to MQTT broker")

    try:
        _wait_for_connection(client, timeout_s=min(10.0, timeout_s))
        state["scan_id"] = client.request_scan(scan_type=scan_type, port=port)
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            terminal_status = state["terminal_status"]
            if terminal_status in {"error", "stopped"}:
                break
            if terminal_status == "completed" and state["data_received_offset_ms"] is not None:
                break
            time.sleep(0.05)
        else:
            raise TimeoutError(
                f"MQTT profiling timed out after {timeout_s:.1f}s for scan {state['scan_id']}"
            )

        run = _finish_run_context(run_ctx)
        run["mode"] = "mqtt"
        run["scan_type"] = scan_type
        run["port"] = port
        run["config_path"] = str(config_path)
        run["scan_id"] = state["scan_id"]
        run["first_status_offset_ms"] = state["first_status_offset_ms"]
        run["terminal_status"] = state["terminal_status"]
        run["terminal_status_offset_ms"] = state["terminal_status_offset_ms"]
        run["data_received_offset_ms"] = state["data_received_offset_ms"]
        run["post_complete_transfer_ms"] = None
        if (
            state["terminal_status_offset_ms"] is not None
            and state["data_received_offset_ms"] is not None
        ):
            run["post_complete_transfer_ms"] = round(
                state["data_received_offset_ms"] - state["terminal_status_offset_ms"],
                3,
            )
        run["status_events"] = state["status_events"]
        run["files"] = [_file_info(path) for path in state["received_files"]]
        return run
    finally:
        client.disconnect()


def _build_report(
    args: argparse.Namespace,
    runs: List[Dict[str, Any]],
    report_path: Optional[Path] = None,
) -> Dict[str, Any]:
    elapsed_values = [float(run["elapsed_ms"]) for run in runs if "elapsed_ms" in run]
    cpu_values = [float(run["cpu_time_ms"]) for run in runs if "cpu_time_ms" in run]

    report = {
        "created_at": _utc_now_iso(),
        "argv": sys.argv,
        "mode": args.mode,
        "scan_type": args.scan_type,
        "repeat": args.repeat,
        "summary": {
            "elapsed_ms": _stat_summary(elapsed_values),
            "cpu_time_ms": _stat_summary(cpu_values),
        },
        "runs": runs,
    }
    if report_path is not None:
        report["report_path"] = str(report_path)
    return report


def _print_summary(report: Dict[str, Any]) -> None:
    summary = report.get("summary", {})
    elapsed = summary.get("elapsed_ms", {})
    cpu = summary.get("cpu_time_ms", {})

    print("=" * 72)
    print(f"Mode: {report.get('mode')} | Scan: {report.get('scan_type')} | Runs: {report.get('repeat')}")
    print("=" * 72)
    print(f"Elapsed ms: {elapsed}")
    print(f"CPU time ms: {cpu}")

    for idx, run in enumerate(report.get("runs", []), start=1):
        print(f"\nRun {idx}: status={run.get('terminal_status', run.get('result', {}).get('success'))}")
        print(f"  elapsed_ms={run.get('elapsed_ms')}")
        if run.get("mode") == "local":
            print(f"  stage_summary={run.get('stage_summary')}")
        else:
            print(f"  first_status_offset_ms={run.get('first_status_offset_ms')}")
            print(f"  terminal_status_offset_ms={run.get('terminal_status_offset_ms')}")
            print(f"  data_received_offset_ms={run.get('data_received_offset_ms')}")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Profile local or MQTT scan execution.")
    parser.add_argument("mode", choices=["local", "mqtt"], help="Profiling mode.")
    parser.add_argument(
        "--scan-type",
        default="robust_3d",
        choices=["2d", "robust_3d"],
        help="Scan type to profile.",
    )
    parser.add_argument(
        "--port",
        default="auto",
        help='Serial port to use, or "auto".',
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Number of repeated runs.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=600.0,
        help="MQTT mode timeout in seconds.",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Override config path. Defaults to config_rpi.yaml for local and config_laptop.yaml for mqtt.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Override scan output directory for local mode.",
    )
    parser.add_argument(
        "--profile-dir",
        default="profiles",
        help="Directory to write JSON profiling reports.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    root = Path(__file__).resolve().parent
    config_path = Path(args.config) if args.config else root / (
        "config_rpi.yaml" if args.mode == "local" else "config_laptop.yaml"
    )
    output_dir = Path(args.output_dir) if args.output_dir else None
    profile_dir = root / args.profile_dir

    runs: List[Dict[str, Any]] = []

    try:
        for _ in range(args.repeat):
            if args.mode == "local":
                run = _run_local_once(
                    scan_type=args.scan_type,
                    port=args.port,
                    config_path=config_path,
                    output_dir_override=output_dir,
                )
            else:
                run = _run_mqtt_once(
                    scan_type=args.scan_type,
                    port=args.port,
                    config_path=config_path,
                    timeout_s=float(args.timeout),
                )
            runs.append(run)

        report = _build_report(args, runs)
        report_path = _write_report(
            profile_dir=profile_dir,
            prefix=f"profile_{args.mode}_{args.scan_type}",
            payload=report,
        )
        report["report_path"] = str(report_path)
        _print_summary(report)
        print(f"\nSaved profiling report to {report_path}")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        failure_report = {
            "created_at": _utc_now_iso(),
            "mode": args.mode,
            "scan_type": args.scan_type,
            "error": str(exc),
            "traceback": traceback.format_exc(),
            "runs": runs,
        }
        report_path = _write_report(
            profile_dir=profile_dir,
            prefix=f"profile_{args.mode}_{args.scan_type}_failed",
            payload=failure_report,
        )
        print(f"Profiling failed: {exc}", file=sys.stderr)
        print(f"Failure report saved to {report_path}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

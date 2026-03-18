from __future__ import annotations

import argparse
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np


S3DIS_LABEL_TO_NAMES = {
    0: "ceiling",
    1: "floor",
    2: "wall",
    3: "beam",
    4: "column",
    5: "window",
    6: "door",
    7: "table",
    8: "chair",
    9: "sofa",
    10: "bookcase",
    11: "board",
    12: "clutter",
}

# Collapsed view for Pi-side structural output.
COLLAPSED_LABEL_TO_NAMES = {
    0: "wall",
    1: "ceiling",
    2: "clutter",
}

S3DIS_TO_COLLAPSED = {
    0: 1,
    1: 2,
    2: 0,
    3: 0,
    4: 0,
    5: 0,
    6: 0,
    7: 2,
    8: 2,
    9: 2,
    10: 2,
    11: 0,
    12: 2,
}

CLASS_COLORS = {
    0: (65, 105, 225),   # wall
    1: (220, 60, 60),    # ceiling
    2: (128, 128, 128),  # clutter
}


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parent.parent
    default_scan = find_latest_scan(repo_root / "data")
    default_ckpt = repo_root / "segmentation" / "Randlanet S3DIS Jan 7 2022.pth"
    default_output = repo_root / "data" / "randla_inference_output"

    parser = argparse.ArgumentParser(
        description="Run CPU RandLA-Net inference on a stitched scan and write labeled outputs."
    )
    parser.add_argument(
        "scan_path",
        nargs="?",
        default=str(default_scan) if default_scan is not None else None,
        help="Path to an ASCII stitched .ply file. Defaults to the latest robust_scan_full*.ply under data/.",
    )
    parser.add_argument(
        "--checkpoint",
        default=str(default_ckpt),
        help="Path to the RandLA-Net checkpoint (.pth).",
    )
    parser.add_argument(
        "--output-dir",
        default=str(default_output),
        help="Directory for labeled outputs.",
    )
    parser.add_argument(
        "--grid-size",
        type=float,
        default=0.01,
        help="RandLA-Net preprocessing voxel size in meters.",
    )
    parser.add_argument(
        "--num-points",
        type=int,
        default=4096,
        help="Patch size for RandLA-Net inference.",
    )
    parser.add_argument(
        "--no-axis-remap",
        action="store_true",
        help="Disable the internal axis remap used for the stitched scan format.",
    )
    return parser.parse_args()


def find_latest_scan(data_dir: Path) -> Path | None:
    candidates = sorted(data_dir.glob("robust_scan_full*.ply"))
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def load_ascii_ply_xyz_intensity(path: Path) -> tuple[np.ndarray, np.ndarray | None, list[str]]:
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        lines = handle.readlines()

    if not lines or lines[0].strip() != "ply":
        raise ValueError(f"Not a PLY file: {path}")

    if "format ascii 1.0" not in "".join(lines[:10]):
        raise ValueError("This script only handles ASCII PLY input.")

    vertex_count = None
    properties: list[str] = []
    header_end = None
    in_vertex = False

    for index, raw_line in enumerate(lines[1:], start=1):
        line = raw_line.strip()
        if line.startswith("element "):
            parts = line.split()
            in_vertex = len(parts) == 3 and parts[1] == "vertex"
            if in_vertex:
                vertex_count = int(parts[2])
                properties = []
        elif in_vertex and line.startswith("property "):
            _, _, prop_name = line.split()
            properties.append(prop_name)
        elif line == "end_header":
            header_end = index
            break

    if vertex_count is None or header_end is None:
        raise ValueError("PLY header is incomplete.")

    rows = [line.strip().split() for line in lines[header_end + 1 : header_end + 1 + vertex_count]]
    array = np.asarray(rows, dtype=np.float32)
    name_to_col = {name: idx for idx, name in enumerate(properties)}

    xyz = array[:, [name_to_col["x"], name_to_col["y"], name_to_col["z"]]].astype(np.float32)
    intensity = array[:, name_to_col["intensity"]].astype(np.float32) if "intensity" in name_to_col else None
    return xyz, intensity, properties


def normalize_column(values: np.ndarray) -> np.ndarray:
    values = values.astype(np.float32)
    vmin = float(values.min())
    vmax = float(values.max())
    if abs(vmax - vmin) < 1e-8:
        return np.zeros_like(values, dtype=np.float32)
    return (values - vmin) / (vmax - vmin)


def collapse_predictions(
    pred_labels_s3dis: np.ndarray,
    pred_scores_s3dis: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    pred_labels = np.array([S3DIS_TO_COLLAPSED[int(label)] for label in pred_labels_s3dis], dtype=np.int32)

    pred_scores_collapsed = None
    max_prob = None
    if pred_scores_s3dis is not None:
        pred_scores_s3dis = np.asarray(pred_scores_s3dis, dtype=np.float32)
        if pred_scores_s3dis.ndim == 2 and pred_scores_s3dis.shape[0] == pred_labels_s3dis.shape[0]:
            pred_scores_collapsed = np.zeros(
                (pred_scores_s3dis.shape[0], len(COLLAPSED_LABEL_TO_NAMES)),
                dtype=np.float32,
            )
            for s3dis_label, collapsed_label in S3DIS_TO_COLLAPSED.items():
                pred_scores_collapsed[:, collapsed_label] += pred_scores_s3dis[:, s3dis_label]
            max_prob = np.max(pred_scores_collapsed, axis=1)

    return pred_labels, pred_scores_collapsed, max_prob


def write_ascii_labeled_ply(
    path: Path,
    xyz: np.ndarray,
    labels: np.ndarray,
    max_prob: np.ndarray | None,
) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {xyz.shape[0]}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property uchar red\n")
        handle.write("property uchar green\n")
        handle.write("property uchar blue\n")
        handle.write("property int label\n")
        if max_prob is not None:
            handle.write("property float max_prob\n")
        handle.write("end_header\n")

        for index in range(xyz.shape[0]):
            x, y, z = xyz[index]
            label = int(labels[index])
            red, green, blue = CLASS_COLORS[label]
            if max_prob is None:
                handle.write(f"{x:.4f} {y:.4f} {z:.4f} {red} {green} {blue} {label}\n")
            else:
                handle.write(
                    f"{x:.4f} {y:.4f} {z:.4f} {red} {green} {blue} {label} {float(max_prob[index]):.6f}\n"
                )


def run_inference(
    scan_path: str | Path,
    checkpoint: str | Path,
    output_dir: str | Path,
    *,
    grid_size: float = 0.01,
    num_points: int = 4096,
    axis_remap: bool = True,
    verbose: bool = True,
) -> dict:
    scan_path = Path(scan_path).expanduser().resolve()
    ckpt_path = Path(checkpoint).expanduser().resolve()
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not scan_path.exists():
        raise FileNotFoundError(f"Scan file not found: {scan_path}")
    if not ckpt_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    try:
        import torch
        import open3d as o3d
        import open3d.ml as _ml3d
        import open3d.ml.torch as ml3d
    except Exception as exc:
        raise RuntimeError(
            "Open3D/Open3D-ML import failed. This script needs torch, tensorboard, and open3d installed."
        ) from exc

    if verbose:
        print("python:", sys.version.split()[0])
        print("platform:", platform.platform())
        print("scan:", scan_path)
        print("checkpoint:", ckpt_path)
        print("output_dir:", output_dir)

    xyz_original, intensity, properties = load_ascii_ply_xyz_intensity(scan_path)
    xyz_model = xyz_original.copy()
    if axis_remap:
        xyz_model = xyz_model[:, [1, 2, 0]].copy()
        xyz_model[:, 2] *= -1

    intensity_norm = normalize_column(intensity) if intensity is not None else np.zeros((xyz_model.shape[0],), dtype=np.float32)
    pseudo_rgb = np.repeat(intensity_norm[:, None], 3, axis=1).astype(np.float32)
    labels_dummy = np.zeros((xyz_model.shape[0],), dtype=np.int32)
    data = {
        "point": xyz_model.astype(np.float32),
        "feat": pseudo_rgb,
        "label": labels_dummy,
    }

    checkpoint = torch.load(ckpt_path, map_location="cpu")
    state_dict = checkpoint["model_state_dict"]

    package_root = Path(o3d.__file__).resolve().parent
    cfg_candidates = sorted(package_root.rglob("randlanet_s3dis*.yml"))
    if not cfg_candidates:
        raise FileNotFoundError("Could not find randlanet_s3dis config inside the installed Open3D package.")

    cfg = _ml3d.utils.Config.load_from_file(str(cfg_candidates[0]))
    cfg.model.grid_size = float(grid_size)
    cfg.model.num_points = int(num_points)
    cfg.pipeline.batch_size = 1

    class DummyS3DISDataset:
        def __init__(self) -> None:
            self.name = "DummyS3DISProbe"
            self.label_to_names = S3DIS_LABEL_TO_NAMES
            self.num_classes = len(S3DIS_LABEL_TO_NAMES)
            self.ignored_label_inds = []

    model = ml3d.models.RandLANet(**cfg.model)
    load_result = model.load_state_dict(state_dict, strict=True)
    if verbose:
        print("state_dict loaded:", load_result)

    dataset = DummyS3DISDataset()
    pipeline = ml3d.pipelines.SemanticSegmentation(
        model,
        dataset=dataset,
        device="cpu",
        **cfg.pipeline,
    )

    started_at = time.perf_counter()
    result = pipeline.run_inference(data)
    elapsed_s = time.perf_counter() - started_at

    pred_labels_s3dis = np.asarray(result["predict_labels"], dtype=np.int32)
    pred_scores_s3dis = np.asarray(result["predict_scores"], dtype=np.float32)
    pred_labels, pred_scores_collapsed, max_prob = collapse_predictions(pred_labels_s3dis, pred_scores_s3dis)

    unique_labels, label_counts = np.unique(pred_labels, return_counts=True)
    summary = {
        "scan_path": str(scan_path),
        "checkpoint": str(ckpt_path),
        "grid_size": float(grid_size),
        "num_points": int(num_points),
        "axis_remap": bool(axis_remap),
        "input_properties": properties,
        "points": int(pred_labels.shape[0]),
        "time_seconds": float(elapsed_s),
        "counts": {
            COLLAPSED_LABEL_TO_NAMES[int(label_id)]: int(count)
            for label_id, count in zip(unique_labels.tolist(), label_counts.tolist())
        },
    }
    if max_prob is not None:
        summary["max_prob"] = {
            "min": float(max_prob.min()),
            "mean": float(max_prob.mean()),
            "median": float(np.median(max_prob)),
            "max": float(max_prob.max()),
        }

    stem = scan_path.stem
    ply_path = output_dir / f"{stem}_randlanet_labels.ply"
    npz_path = output_dir / f"{stem}_randlanet_outputs.npz"
    json_path = output_dir / f"{stem}_randlanet_summary.json"

    write_ascii_labeled_ply(ply_path, xyz_original, pred_labels, max_prob)
    save_payload = {
        "xyz_original": xyz_original.astype(np.float32),
        "xyz_model": xyz_model.astype(np.float32),
        "labels": pred_labels.astype(np.int32),
        "labels_s3dis": pred_labels_s3dis.astype(np.int32),
        "scores_s3dis": pred_scores_s3dis.astype(np.float32),
    }
    if pred_scores_collapsed is not None:
        save_payload["scores_collapsed"] = pred_scores_collapsed.astype(np.float32)
    if max_prob is not None:
        save_payload["max_prob"] = max_prob.astype(np.float32)
    np.savez_compressed(npz_path, **save_payload)
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    result = {
        "success": True,
        "files": [str(ply_path), str(npz_path), str(json_path)],
        "summary": summary,
        "labeled_ply": str(ply_path),
        "raw_outputs": str(npz_path),
        "summary_json": str(json_path),
    }
    if verbose:
        print("summary:", json.dumps(summary, indent=2))
        print("labeled_ply:", ply_path)
        print("raw_outputs:", npz_path)
        print("summary_json:", json_path)
    return result


def main() -> int:
    args = parse_args()

    if args.scan_path is None:
        raise FileNotFoundError("No stitched scan found under data/ and no scan_path was provided.")

    run_inference(
        scan_path=args.scan_path,
        checkpoint=args.checkpoint,
        output_dir=args.output_dir,
        grid_size=args.grid_size,
        num_points=args.num_points,
        axis_remap=not args.no_axis_remap,
        verbose=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

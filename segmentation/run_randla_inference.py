from __future__ import annotations

import argparse
import json
import platform
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

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


@dataclass(frozen=True)
class RandLAConfig:
    num_neighbors: int = 16
    num_layers: int = 5
    num_points: int = 4096
    num_classes: int = 13
    sub_sampling_ratio: tuple[int, ...] = (4, 4, 4, 4, 2)
    in_channels: int = 6
    dim_features: int = 8
    dim_output: tuple[int, ...] = (16, 64, 128, 256, 512)
    grid_size: float = 0.01
    recenter_dims: tuple[int, ...] = (0, 1)


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


def _recenter_points(points: np.ndarray, dims: tuple[int, ...]) -> np.ndarray:
    centered = points.copy()
    centered[:, list(dims)] = centered[:, list(dims)] - centered.mean(axis=0)[list(dims)]
    return centered.astype(np.float32)


def _majority_vote(labels: np.ndarray, inverse: np.ndarray, num_groups: int) -> np.ndarray:
    labels = labels.astype(np.int64, copy=False)
    if labels.size == 0:
        return np.zeros((num_groups,), dtype=np.int32)
    if np.all(labels == labels[0]):
        return np.full((num_groups,), labels[0], dtype=np.int32)

    out = np.zeros((num_groups,), dtype=np.int32)
    for group_idx in range(num_groups):
        group_labels = labels[inverse == group_idx]
        out[group_idx] = int(np.bincount(group_labels).argmax())
    return out


def _grid_subsample(
    points: np.ndarray,
    *,
    features: np.ndarray | None = None,
    labels: np.ndarray | None = None,
    grid_size: float,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    if grid_size <= 0:
        sub_points = points.astype(np.float32, copy=True)
        sub_features = None if features is None else features.astype(np.float32, copy=True)
        sub_labels = None if labels is None else labels.astype(np.int32, copy=True)
        return sub_points, sub_features, sub_labels

    voxel_keys = np.floor(points / grid_size).astype(np.int64)
    _, inverse, counts = np.unique(voxel_keys, axis=0, return_inverse=True, return_counts=True)
    num_voxels = counts.shape[0]
    counts_f = counts.astype(np.float32)

    sub_points = np.empty((num_voxels, points.shape[1]), dtype=np.float32)
    for dim in range(points.shape[1]):
        sub_points[:, dim] = np.bincount(
            inverse,
            weights=points[:, dim].astype(np.float64),
            minlength=num_voxels,
        ) / counts_f

    sub_features = None
    if features is not None:
        sub_features = np.empty((num_voxels, features.shape[1]), dtype=np.float32)
        for dim in range(features.shape[1]):
            sub_features[:, dim] = np.bincount(
                inverse,
                weights=features[:, dim].astype(np.float64),
                minlength=num_voxels,
            ) / counts_f

    sub_labels = None
    if labels is not None:
        sub_labels = _majority_vote(labels, inverse, num_voxels)

    return sub_points, sub_features, sub_labels


def _load_runtime_deps() -> tuple[Any, Any, Any]:
    try:
        import torch
        import torch.nn as nn
        from scipy.spatial import cKDTree
    except Exception as exc:
        raise RuntimeError(
            "Pure PyTorch RandLA inference needs torch and scipy installed."
        ) from exc
    return torch, nn, cKDTree


def _build_model_classes(nn: Any, torch: Any) -> tuple[type, type, type, type]:
    class SharedMLP(nn.Module):
        def __init__(
            self,
            in_channels: int,
            out_channels: int,
            *,
            kernel_size: int = 1,
            stride: int = 1,
            transpose: bool = False,
            bn: bool = True,
            activation_fn: Any = None,
        ):
            super().__init__()
            if transpose:
                self.conv = nn.ConvTranspose2d(
                    in_channels,
                    out_channels,
                    kernel_size=kernel_size,
                    stride=stride,
                    padding=(kernel_size - 1) // 2,
                )
            else:
                self.conv = nn.Conv2d(
                    in_channels,
                    out_channels,
                    kernel_size=kernel_size,
                    stride=stride,
                    padding=(kernel_size - 1) // 2,
                )
            self.batch_norm = nn.BatchNorm2d(out_channels, eps=1e-6, momentum=0.01) if bn else None
            self.activation_fn = activation_fn

        def forward(self, inputs: Any) -> Any:
            outputs = self.conv(inputs)
            if self.batch_norm is not None:
                outputs = self.batch_norm(outputs)
            if self.activation_fn is not None:
                outputs = self.activation_fn(outputs)
            return outputs

    class LocalSpatialEncoding(nn.Module):
        def __init__(self, dim_in: int, dim_out: int, num_neighbors: int, *, encode_pos: bool = False):
            super().__init__()
            self.num_neighbors = num_neighbors
            self.mlp = SharedMLP(dim_in, dim_out, activation_fn=nn.LeakyReLU(0.2))
            self.encode_pos = encode_pos

        @staticmethod
        def gather_neighbor(coords: Any, neighbor_indices: Any) -> Any:
            batch_size, num_points, num_neighbors = neighbor_indices.size()
            dim = coords.shape[2]
            expanded_indices = neighbor_indices.unsqueeze(1).expand(batch_size, dim, num_points, num_neighbors)
            expanded_coords = coords.transpose(-2, -1).unsqueeze(-1).expand(batch_size, dim, num_points, num_neighbors)
            return torch.gather(expanded_coords, 2, expanded_indices)

        def forward(
            self,
            coords: Any,
            features: Any,
            neighbor_indices: Any,
            relative_features: Any = None,
        ) -> tuple[Any, Any]:
            batch_size, num_points, num_neighbors = neighbor_indices.size()
            if self.encode_pos:
                neighbor_coords = self.gather_neighbor(coords, neighbor_indices)
                expanded_coords = coords.transpose(-2, -1).unsqueeze(-1).expand(batch_size, 3, num_points, num_neighbors)
                relative_pos = expanded_coords - neighbor_coords
                relative_dist = torch.sqrt(torch.sum(torch.square(relative_pos), dim=1, keepdim=True))
                relative_features = torch.cat(
                    [relative_dist, relative_pos, expanded_coords, neighbor_coords],
                    dim=1,
                )
            elif relative_features is None:
                raise ValueError("LocalSpatialEncoding requires relative_features on the second pass.")

            encoded_relative = self.mlp(relative_features)
            neighbor_features = self.gather_neighbor(features.transpose(1, 2).squeeze(3), neighbor_indices)
            return torch.cat([neighbor_features, encoded_relative], dim=1), encoded_relative

    class AttentivePooling(nn.Module):
        def __init__(self, in_channels: int, out_channels: int):
            super().__init__()
            self.score_fn = nn.Sequential(nn.Linear(in_channels, in_channels), nn.Softmax(dim=-2))
            self.mlp = SharedMLP(in_channels, out_channels, activation_fn=nn.LeakyReLU(0.2))

        def forward(self, x: Any) -> Any:
            scores = self.score_fn(x.permute(0, 2, 3, 1)).permute(0, 3, 1, 2)
            features = torch.sum(scores * x, dim=-1, keepdim=True)
            return self.mlp(features)

    class LocalFeatureAggregation(nn.Module):
        def __init__(self, dim_in: int, dim_out: int, num_neighbors: int):
            super().__init__()
            self.mlp1 = SharedMLP(dim_in, dim_out // 2, activation_fn=nn.LeakyReLU(0.2))
            self.lse1 = LocalSpatialEncoding(10, dim_out // 2, num_neighbors, encode_pos=True)
            self.pool1 = AttentivePooling(dim_out, dim_out // 2)
            self.lse2 = LocalSpatialEncoding(dim_out // 2, dim_out // 2, num_neighbors)
            self.pool2 = AttentivePooling(dim_out, dim_out)
            self.mlp2 = SharedMLP(dim_out, 2 * dim_out)
            self.shortcut = SharedMLP(dim_in, 2 * dim_out)
            self.lrelu = nn.LeakyReLU()

        def forward(self, coords: Any, feat: Any, neighbor_indices: Any) -> Any:
            x = self.mlp1(feat)
            x, neighbor_features = self.lse1(coords, x, neighbor_indices)
            x = self.pool1(x)
            x, _ = self.lse2(coords, x, neighbor_indices, relative_features=neighbor_features)
            x = self.pool2(x)
            return self.lrelu(self.mlp2(x) + self.shortcut(feat))

    class RandLANet(nn.Module):
        def __init__(self, cfg: RandLAConfig):
            super().__init__()
            self.cfg = cfg
            self.fc0 = nn.Linear(cfg.in_channels, cfg.dim_features)
            self.bn0 = nn.BatchNorm2d(cfg.dim_features, eps=1e-6, momentum=0.01)

            self.encoder = nn.ModuleList()
            encoder_dim_list: list[int] = []
            dim_feature = cfg.dim_features
            for layer_idx in range(cfg.num_layers):
                self.encoder.append(
                    LocalFeatureAggregation(dim_feature, cfg.dim_output[layer_idx], cfg.num_neighbors)
                )
                dim_feature = 2 * cfg.dim_output[layer_idx]
                if layer_idx == 0:
                    encoder_dim_list.append(dim_feature)
                encoder_dim_list.append(dim_feature)

            self.mlp = SharedMLP(dim_feature, dim_feature, activation_fn=nn.LeakyReLU(0.2))

            self.decoder = nn.ModuleList()
            for layer_idx in range(cfg.num_layers):
                self.decoder.append(
                    SharedMLP(
                        encoder_dim_list[-layer_idx - 2] + dim_feature,
                        encoder_dim_list[-layer_idx - 2],
                        transpose=True,
                        activation_fn=nn.LeakyReLU(0.2),
                    )
                )
                dim_feature = encoder_dim_list[-layer_idx - 2]

            self.fc1 = nn.Sequential(
                SharedMLP(dim_feature, 64, activation_fn=nn.LeakyReLU(0.2)),
                SharedMLP(64, 32, activation_fn=nn.LeakyReLU(0.2)),
                nn.Dropout(0.5),
                SharedMLP(32, cfg.num_classes, bn=False),
            )

        @staticmethod
        def random_sample(feature: Any, pool_idx: Any) -> Any:
            feature = feature.squeeze(3)
            num_neighbors = pool_idx.size()[2]
            batch_size = feature.size()[0]
            dim = feature.size()[1]

            pool_idx = torch.reshape(pool_idx, (batch_size, -1))
            pool_idx = pool_idx.unsqueeze(2).expand(batch_size, -1, dim)

            feature = feature.transpose(1, 2)
            pooled = torch.gather(feature, 1, pool_idx)
            pooled = torch.reshape(pooled, (batch_size, -1, num_neighbors, dim))
            pooled, _ = torch.max(pooled, 2, keepdim=True)
            return pooled.permute(0, 3, 1, 2)

        @staticmethod
        def nearest_interpolation(feature: Any, interp_idx: Any) -> Any:
            feature = feature.squeeze(3)
            dim = feature.size(1)
            batch_size = interp_idx.size()[0]
            up_num_points = interp_idx.size()[1]

            interp_idx = torch.reshape(interp_idx, (batch_size, up_num_points))
            interp_idx = interp_idx.unsqueeze(1).expand(batch_size, dim, -1)

            interpolated = torch.gather(feature, 2, interp_idx)
            return interpolated.unsqueeze(3)

        def forward(self, inputs: dict[str, Any]) -> Any:
            feat = inputs["features"]
            coords_list = inputs["coords"]
            neighbor_indices_list = inputs["neighbor_indices"]
            subsample_indices_list = inputs["sub_idx"]
            interpolation_indices_list = inputs["interp_idx"]

            feat = self.fc0(feat).transpose(-2, -1).unsqueeze(-1)
            feat = self.bn0(feat)
            feat = nn.LeakyReLU(0.2)(feat)

            encoder_feat_list = []
            for layer_idx in range(self.cfg.num_layers):
                feat_encoder = self.encoder[layer_idx](coords_list[layer_idx], feat, neighbor_indices_list[layer_idx])
                feat_sampled = self.random_sample(feat_encoder, subsample_indices_list[layer_idx])
                if layer_idx == 0:
                    encoder_feat_list.append(feat_encoder.clone())
                encoder_feat_list.append(feat_sampled.clone())
                feat = feat_sampled

            feat = self.mlp(feat)

            for layer_idx in range(self.cfg.num_layers):
                feat_interp = self.nearest_interpolation(feat, interpolation_indices_list[-layer_idx - 1])
                feat_decoder = torch.cat([encoder_feat_list[-layer_idx - 2], feat_interp], dim=1)
                feat = self.decoder[layer_idx](feat_decoder)

            scores = self.fc1(feat)
            return scores.squeeze(3).transpose(1, 2)

    return RandLANet, LocalFeatureAggregation, LocalSpatialEncoding, AttentivePooling


def _build_search_tree(points: np.ndarray, cKDTree: Any) -> Any:
    return cKDTree(points)


def _knn_search(support_pts: np.ndarray, query_pts: np.ndarray, k: int, cKDTree: Any) -> np.ndarray:
    support_count = support_pts.shape[0]
    if support_count == 0:
        raise ValueError("Cannot run knn_search with zero support points.")

    effective_k = min(k, support_count)
    tree = _build_search_tree(support_pts, cKDTree)
    _, indices = tree.query(query_pts, k=effective_k)
    indices = np.asarray(indices, dtype=np.int32)
    if effective_k == 1:
        indices = indices.reshape(-1, 1)
    if effective_k < k:
        pad = np.repeat(indices[:, -1:], k - effective_k, axis=1)
        indices = np.concatenate([indices, pad], axis=1)
    return indices


def _query_patch_indices(
    search_tree: Any,
    points: np.ndarray,
    center_id: int,
    num_points: int,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray]:
    center_point = points[center_id : center_id + 1]
    if points.shape[0] < num_points:
        base = np.arange(points.shape[0], dtype=np.int64)
        extra = rng.choice(base, size=num_points - points.shape[0], replace=True)
        indices = np.concatenate([base, extra], axis=0)
    else:
        _, indices = search_tree.query(center_point, k=num_points)
        indices = np.asarray(indices[0], dtype=np.int64)

    rng.shuffle(indices)
    return indices, center_point.astype(np.float32)


def _preprocess_for_inference(
    data: dict[str, np.ndarray],
    cfg: RandLAConfig,
    cKDTree: Any,
) -> dict[str, Any]:
    points = np.asarray(data["point"][:, 0:3], dtype=np.float32)
    labels = np.asarray(data.get("label", np.zeros((points.shape[0],), dtype=np.int32)), dtype=np.int32).reshape(-1)
    feat = data.get("feat")
    feat = None if feat is None else np.asarray(feat, dtype=np.float32)

    sub_points, sub_feat, sub_labels = _grid_subsample(points, features=feat, labels=labels, grid_size=cfg.grid_size)
    search_tree = _build_search_tree(sub_points, cKDTree)
    _, proj_inds = search_tree.query(points, k=1)
    proj_inds = np.asarray(proj_inds, dtype=np.int32).reshape(-1)

    return {
        "point": sub_points.astype(np.float32),
        "feat": None if sub_feat is None else sub_feat.astype(np.float32),
        "label": None if sub_labels is None else sub_labels.astype(np.int32),
        "search_tree": search_tree,
        "proj_inds": proj_inds,
    }


def _build_patch_inputs(
    processed: dict[str, Any],
    cfg: RandLAConfig,
    possibility: np.ndarray,
    rng: np.random.Generator,
    cKDTree: Any,
) -> dict[str, Any]:
    pc_all = processed["point"]
    feat_all = processed["feat"]
    label_all = processed["label"]
    search_tree = processed["search_tree"]

    center_id = int(np.argmin(possibility))
    selected_idxs, center_point = _query_patch_indices(search_tree, pc_all, center_id, cfg.num_points, rng)
    selected_pc_original = pc_all[selected_idxs].astype(np.float32, copy=True)

    dists = np.sum(np.square(selected_pc_original - center_point), axis=1)
    denom = float(np.max(dists))
    if denom < 1e-12:
        delta = np.ones_like(dists, dtype=np.float32)
    else:
        delta = np.square(1.0 - (dists / denom)).astype(np.float32)
    np.add.at(possibility, selected_idxs, delta)

    pc = _recenter_points(selected_pc_original, cfg.recenter_dims)
    labels = label_all[selected_idxs].astype(np.int64, copy=True)
    feat = None if feat_all is None else feat_all[selected_idxs].astype(np.float32, copy=True)

    features = pc.copy() if feat is None else np.concatenate([pc, feat], axis=1).astype(np.float32)
    if features.shape[1] != cfg.in_channels:
        raise RuntimeError(
            f"Wrong feature dimension: expected {cfg.in_channels}, got {features.shape[1]}"
        )

    coords: list[np.ndarray] = []
    neighbor_indices: list[np.ndarray] = []
    sub_idx: list[np.ndarray] = []
    interp_idx: list[np.ndarray] = []

    current_pc = pc
    for layer_idx in range(cfg.num_layers):
        neighbor_idx = _knn_search(current_pc, current_pc, cfg.num_neighbors, cKDTree)
        next_count = max(1, current_pc.shape[0] // cfg.sub_sampling_ratio[layer_idx])
        sub_points = current_pc[:next_count, :]
        pool_i = neighbor_idx[:next_count, :]
        up_i = _knn_search(sub_points, current_pc, 1, cKDTree)

        coords.append(current_pc.astype(np.float32))
        neighbor_indices.append(neighbor_idx.astype(np.int64))
        sub_idx.append(pool_i.astype(np.int64))
        interp_idx.append(up_i.astype(np.int64))
        current_pc = sub_points

    return {
        "coords": coords,
        "neighbor_indices": neighbor_indices,
        "sub_idx": sub_idx,
        "interp_idx": interp_idx,
        "features": features.astype(np.float32),
        "point_inds": selected_idxs.astype(np.int64),
        "labels": labels.astype(np.int64),
    }


def _to_torch_batch(inputs: dict[str, Any], torch: Any, device: str) -> dict[str, Any]:
    return {
        "coords": [torch.from_numpy(arr).unsqueeze(0).to(device=device, dtype=torch.float32) for arr in inputs["coords"]],
        "neighbor_indices": [
            torch.from_numpy(arr).unsqueeze(0).to(device=device, dtype=torch.int64)
            for arr in inputs["neighbor_indices"]
        ],
        "sub_idx": [torch.from_numpy(arr).unsqueeze(0).to(device=device, dtype=torch.int64) for arr in inputs["sub_idx"]],
        "interp_idx": [
            torch.from_numpy(arr).unsqueeze(0).to(device=device, dtype=torch.int64)
            for arr in inputs["interp_idx"]
        ],
        "features": torch.from_numpy(inputs["features"]).unsqueeze(0).to(device=device, dtype=torch.float32),
        "point_inds": torch.from_numpy(inputs["point_inds"]).unsqueeze(0),
        "labels": torch.from_numpy(inputs["labels"]).unsqueeze(0),
    }


def _load_model(checkpoint_path: Path, cfg: RandLAConfig, torch: Any, nn: Any) -> Any:
    RandLANet, _, _, _ = _build_model_classes(nn, torch)
    model = RandLANet(cfg)
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    state_dict = checkpoint["model_state_dict"]
    model.load_state_dict(state_dict, strict=True)
    model.eval()
    return model


def _run_model_inference(
    model: Any,
    data: dict[str, np.ndarray],
    cfg: RandLAConfig,
    *,
    torch: Any,
    cKDTree: Any,
    verbose: bool,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    processed = _preprocess_for_inference(data, cfg, cKDTree)
    rng = np.random.default_rng(0)
    possibility = rng.random(processed["point"].shape[0]).astype(np.float32) * 1e-3
    test_probs = np.zeros((processed["point"].shape[0], cfg.num_classes), dtype=np.float32)
    smooth = 0.95
    end_threshold = 0.5
    iteration = 0
    reported_complete = -1

    started_at = time.perf_counter()
    with torch.no_grad():
        while True:
            patch_inputs = _build_patch_inputs(processed, cfg, possibility, rng, cKDTree)
            batch = _to_torch_batch(patch_inputs, torch, device="cpu")
            results = model(batch)
            probs = torch.softmax(results.reshape(-1, cfg.num_classes), dim=-1).cpu().numpy().astype(np.float32)

            point_inds = batch["point_inds"][0].cpu().numpy().astype(np.int64)
            test_probs[point_inds] = smooth * test_probs[point_inds] + (1.0 - smooth) * probs

            iteration += 1
            completed = int(np.count_nonzero(possibility > end_threshold))
            if verbose and (iteration == 1 or iteration % 25 == 0 or completed != reported_complete):
                print(
                    f"inference steps={iteration} "
                    f"covered={completed}/{possibility.shape[0]} "
                    f"min_possibility={float(possibility.min()):.4f}"
                )
                reported_complete = completed

            if completed == possibility.shape[0]:
                break

    elapsed_s = time.perf_counter() - started_at
    proj_inds = processed["proj_inds"]
    pred_scores = test_probs[proj_inds]
    pred_labels = np.argmax(pred_scores, axis=1).astype(np.int32)
    diagnostics = {
        "subsampled_points": int(processed["point"].shape[0]),
        "inference_steps": int(iteration),
        "time_seconds": float(elapsed_s),
    }
    return pred_labels, pred_scores.astype(np.float32), diagnostics


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

    torch, nn, cKDTree = _load_runtime_deps()

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

    intensity_norm = (
        normalize_column(intensity)
        if intensity is not None
        else np.zeros((xyz_model.shape[0],), dtype=np.float32)
    )
    pseudo_rgb = np.repeat(intensity_norm[:, None], 3, axis=1).astype(np.float32)
    labels_dummy = np.zeros((xyz_model.shape[0],), dtype=np.int32)
    data = {
        "point": xyz_model.astype(np.float32),
        "feat": pseudo_rgb,
        "label": labels_dummy,
    }

    cfg = RandLAConfig(grid_size=float(grid_size), num_points=int(num_points))
    model = _load_model(ckpt_path, cfg, torch, nn)

    pred_labels_s3dis, pred_scores_s3dis, diagnostics = _run_model_inference(
        model,
        data,
        cfg,
        torch=torch,
        cKDTree=cKDTree,
        verbose=verbose,
    )

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
        "subsampled_points": diagnostics["subsampled_points"],
        "inference_steps": diagnostics["inference_steps"],
        "time_seconds": diagnostics["time_seconds"],
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

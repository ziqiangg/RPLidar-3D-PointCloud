"""
OpenCV panorama helpers for laptop-side image stitching and rendering.

This module centralizes the panorama capture assumptions used by the project:
- Logitech C270 captures at 1280x720
- 55 degree diagonal field of view
- one image every 30 degrees from 0 to 150 (6 captures expected)
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass
from datetime import datetime
from typing import List, Tuple, Optional

try:
    import cv2
except ImportError:
    cv2 = None


@dataclass(frozen=True)
class PanoramaCaptureSpec:
    diag_fov_deg: float = 55.0
    width_px: int = 1280
    height_px: int = 720
    start_deg: float = 0.0
    end_deg: float = 150.0
    step_deg: float = 30.0

    @property
    def expected_capture_count(self) -> int:
        return int(round((self.end_deg - self.start_deg) / self.step_deg)) + 1


CAPTURE_SPEC = PanoramaCaptureSpec()


def _capture_image_sort_key(path: str) -> Tuple[int, str]:
    """Sort panorama_XX images by numeric index where available."""
    base = os.path.basename(path)
    stem, _ = os.path.splitext(base)
    parts = stem.split("_")
    if len(parts) >= 2 and parts[-1].isdigit():
        return (int(parts[-1]), base.lower())
    return (10**9, base.lower())


def find_panorama_capture_images(images_dir: str) -> List[str]:
    """Return sorted source captures (excluding already stitched outputs)."""
    patterns = ["panorama_*.jpg", "panorama_*.jpeg", "panorama_*.png"]
    files: List[str] = []

    for pattern in patterns:
        files.extend(glob.glob(os.path.join(images_dir, pattern)))

    files = [
        p for p in files
        if "stitched" not in os.path.basename(p).lower() and os.path.isfile(p)
    ]
    files.sort(key=_capture_image_sort_key)
    return files


def stitch_panorama_from_paths(image_paths: List[str], output_path: str) -> Tuple[bool, str]:
    """
    Stitch capture images into a single panorama using OpenCV stitcher.

    Returns:
        (success, message)
    """
    if cv2 is None:
        return False, "opencv-python is not installed"

    if len(image_paths) < 2:
        return False, "Need at least 2 images to stitch"

    frames = []
    for path in image_paths:
        frame = cv2.imread(path)
        if frame is None:
            return False, f"Unable to read image: {path}"
        frames.append(frame)

    # PANORAMA mode is used to produce a single wide stitched output.
    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    status, pano = stitcher.stitch(frames)
    if status != cv2.Stitcher_OK or pano is None:
        return False, f"OpenCV stitch failed (status={status})"

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    ok = cv2.imwrite(output_path, pano, [int(getattr(cv2, "IMWRITE_JPEG_QUALITY", 1)), 95])
    if not ok:
        return False, f"Failed writing stitched output: {output_path}"

    return True, f"Stitched panorama saved: {output_path}"


def stitch_panorama_from_dir(images_dir: str, output_path: Optional[str] = None) -> Tuple[bool, str, str]:
    """
    Stitch all capture images from an images folder.

    Returns:
        (success, message, output_path_or_empty)
    """
    image_paths = find_panorama_capture_images(images_dir)
    if not image_paths:
        return False, f"No panorama capture images found in {images_dir}", ""

    if output_path is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = os.path.join(images_dir, f"panorama_stitched_{ts}.jpg")

    success, msg = stitch_panorama_from_paths(image_paths, output_path)
    expected = CAPTURE_SPEC.expected_capture_count
    count_msg = f"captures={len(image_paths)}/{expected} expected"

    if success:
        return True, f"{msg} ({count_msg})", output_path
    return False, f"{msg} ({count_msg})", ""


def render_panorama_window(image_path: str, window_name: str = "Panorama Preview") -> Tuple[bool, str]:
    """Render a stitched panorama in an OpenCV window."""
    if cv2 is None:
        return False, "opencv-python is not installed"

    if not os.path.exists(image_path):
        return False, f"Panorama image not found: {image_path}"

    frame = cv2.imread(image_path)
    if frame is None:
        return False, f"Unable to read panorama image: {image_path}"

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, frame)
    while True:
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord('q')):
            break
    cv2.destroyAllWindows()
    return True, "Panorama window closed"

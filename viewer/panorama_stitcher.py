"""Panorama stitching helpers for laptop-side image reconstruction and viewing."""

import glob
import os
import re
from typing import List, Tuple

import cv2


def find_panorama_images(images_dir: str) -> List[str]:
    """Return ordered panorama image paths from the images directory."""
    patterns = [
        "panorama_*.jpg",
        "panorama_*.jpeg",
        "panorama_*.png",
    ]
    image_paths: List[str] = []
    for pattern in patterns:
        image_paths.extend(glob.glob(os.path.join(images_dir, pattern)))
    capture_name = re.compile(r"^panorama_\d{2}\.(jpg|jpeg|png)$", re.IGNORECASE)
    image_paths = [
        p for p in sorted(set(image_paths))
        if capture_name.match(os.path.basename(p))
    ]
    return image_paths


def _extract_panorama_index(path: str):
    """Return numeric index for panorama_XX filenames, otherwise None."""
    base = os.path.basename(path)
    m = re.match(r"^panorama_(\d{2})\.(jpg|jpeg|png)$", base, re.IGNORECASE)
    if not m:
        return None
    return int(m.group(1))


def _effective_yaw_deg(
    index: int,
    images_per_camera: int,
    front_yaw_offset_deg: float,
    step_deg: float,
    rear_relative_yaw_deg: float,
) -> float:
    """Map dual-camera image index to effective yaw around 360 degrees.

    Index layout is front captures first, then rear captures:
    - 00..(N-1): front camera
    - N..(2N-1): rear camera
    """
    if images_per_camera <= 0:
        return float(index)

    if 0 <= index < images_per_camera:
        return float((front_yaw_offset_deg + (index * step_deg)) % 360.0)
    if images_per_camera <= index < (2 * images_per_camera):
        return float(
            (
                front_yaw_offset_deg
                + rear_relative_yaw_deg
                + ((index - images_per_camera) * step_deg)
            )
            % 360.0
        )
    # Fallback keeps ordering stable if indices extend beyond current dual-camera convention.
    return float(index)


def order_dual_camera_panorama_images(
    image_paths: List[str],
    front_yaw_offset_deg: float,
    step_deg: float,
    rear_relative_yaw_deg: float,
) -> List[str]:
    """Order panorama images by effective yaw for dual-camera 360 stitching."""
    indexed = []
    passthrough = []
    for p in image_paths:
        idx = _extract_panorama_index(p)
        if idx is None:
            passthrough.append(p)
        else:
            indexed.append((idx, p))

    if not indexed:
        return image_paths

    max_idx = max(idx for idx, _ in indexed)
    slot_count = max_idx + 1
    if slot_count % 2 == 0:
        images_per_camera = slot_count // 2
    else:
        images_per_camera = max(1, len(indexed) // 2)

    indexed.sort(
        key=lambda t: (
            _effective_yaw_deg(
                t[0],
                images_per_camera=images_per_camera,
                front_yaw_offset_deg=front_yaw_offset_deg,
                step_deg=step_deg,
                rear_relative_yaw_deg=rear_relative_yaw_deg,
            ),
            t[0],
        )
    )
    ordered = [p for _, p in indexed]
    if passthrough:
        ordered.extend(sorted(passthrough))
    return ordered


def stitch_equirectangular_panorama(
    image_paths: List[str],
    output_path: str,
    front_yaw_offset_deg: float = 0.0,
    step_deg: float = 30.0,
    rear_relative_yaw_deg: float = 180.0,
) -> Tuple[bool, str, str]:
    """Stitch source images into one panorama and save to output_path."""
    if not image_paths:
        return False, "No panorama images found", ""

    ordered_paths = order_dual_camera_panorama_images(
        image_paths,
        front_yaw_offset_deg=front_yaw_offset_deg,
        step_deg=step_deg,
        rear_relative_yaw_deg=rear_relative_yaw_deg,
    )

    frames = []
    for path in ordered_paths:
        frame = cv2.imread(path)
        if frame is None:
            return False, f"Failed to read image: {path}", ""
        frames.append(frame)

    if len(frames) < 2:
        return False, "Need at least 2 images to stitch", ""

    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    status, panorama = stitcher.stitch(frames)
    if status != cv2.Stitcher_OK or panorama is None:
        return False, f"OpenCV stitch failed (status={status})", ""

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    ok = cv2.imwrite(output_path, panorama)
    if not ok:
        return False, f"Failed writing stitched panorama: {output_path}", ""

    return True, f"Stitched panorama saved: {output_path}", output_path


def show_panorama_window(image_path: str, window_title: str = "Stitched Panorama") -> Tuple[bool, str]:
    """Display stitched panorama using OpenCV window."""
    if not os.path.exists(image_path):
        return False, f"Panorama file not found: {image_path}"

    image = cv2.imread(image_path)
    if image is None:
        return False, f"Unable to load panorama image: {image_path}"

    height, width = image.shape[:2]

    # Keep rendered aspect ratio so wide panoramas are not squished.
    window_flags = cv2.WINDOW_NORMAL
    if hasattr(cv2, "WINDOW_KEEPRATIO"):
        window_flags |= cv2.WINDOW_KEEPRATIO

    cv2.namedWindow(window_title, window_flags)
    try:
        cv2.resizeWindow(window_title, int(width), int(height))
    except cv2.error:
        pass

    cv2.imshow(window_title, image)

    # Keep window responsive until user closes it or presses a key.
    while True:
        key = cv2.waitKey(30) & 0xFF
        try:
            visible = cv2.getWindowProperty(window_title, cv2.WND_PROP_VISIBLE)
        except cv2.error:
            # Window may already be gone; treat as a normal close path.
            break
        if key != 255 or visible < 1:
            break

    try:
        visible = cv2.getWindowProperty(window_title, cv2.WND_PROP_VISIBLE)
        if visible >= 1:
            cv2.destroyWindow(window_title)
    except cv2.error:
        # If already closed by user/OS, do not fail rendering flow.
        pass

    try:
        cv2.waitKey(1)
    except cv2.error:
        pass

    return True, f"Opened panorama window: {window_title}"

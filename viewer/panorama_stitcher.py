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


def stitch_equirectangular_panorama(
    image_paths: List[str],
    output_path: str,
) -> Tuple[bool, str, str]:
    """Stitch source images into one panorama and save to output_path."""
    if not image_paths:
        return False, "No panorama images found", ""

    frames = []
    for path in image_paths:
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

    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
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

"""Standalone OpenCV panorama preview window."""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from viewer.panorama_tools import render_panorama_window


def main():
    parser = argparse.ArgumentParser(description="Render a stitched panorama in an OpenCV window")
    parser.add_argument("image", help="Path to stitched panorama image")
    args = parser.parse_args()

    ok, msg = render_panorama_window(args.image)
    if not ok:
        print(f"[ERROR] {msg}")
        sys.exit(1)

    print(f"[INFO] {msg}")


if __name__ == "__main__":
    main()

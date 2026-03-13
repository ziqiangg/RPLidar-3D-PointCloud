import csv, math, time, sys, os
from statistics import median
from rplidar import RPLidar
from utils.port_config import get_default_port

BAUD = 115200

# --- Tuning knobs for RPLidar A1 (indoor) ---
ANGLE_BIN_DEG = 1.0          # Angular binning resolution (deg). 1° looks nicer, but merge more scans.
MIN_COVERAGE = 0.85          # Target angular coverage fraction (0..1)
MAX_SCANS_TO_MERGE = 30      # Merge up to N revolutions to fill holes
TIMEOUT_SEC = 15             # Hard timeout for scan collection

# Distance filtering (mm) to drop obvious junk
MIN_DIST_MM = 200
MAX_DIST_MM = 8000

# "Dead spot" tolerance / checks
MAX_GAP_DEG = 6.0            # Reject (or stop trying) if we still have gaps bigger than this
PLATEAU_EPS = 0.002          # Coverage change threshold to call "plateau"
PLATEAU_ITERS = 4            # How many consecutive plateaus before early exit

# Optional: Fill small gaps for PLY visualization (CSV remains unfilled merged data)
FILL_GAPS_FOR_PLY = True
FILL_MAX_GAP_DEG = 6.0       # Only fill gaps up to this many degrees


def _bin_index(angle_deg: float, bin_deg: float) -> int:
    """Stable angle binning with wrap-around."""
    a = angle_deg % 360.0
    total_bins = int(round(360.0 / bin_deg))
    idx = int(round(a / bin_deg)) % total_bins
    return idx


def merge_scans_robust(scans, bin_deg=ANGLE_BIN_DEG):
    """
    Merge multiple scans using robust per-bin statistics.

    For each angle bin, keep all distances across merged scans and output:
      - distance: median(distances)  (robust to outliers)
      - quality:  max(quality)       (best observed quality)
      - angle:    bin center angle

    Returns:
      merged: list[(quality, angle_deg, distance_mm)]
      hit_counts: dict[bin_idx] -> int  (#samples contributing to that bin)
    """
    bin_dists = {}      # bin_idx -> [dist_mm, ...]
    bin_quals = {}      # bin_idx -> max_quality

    for scan in scans:
        for quality, angle, dist in scan:
            if dist is None:
                continue
            try:
                dist = float(dist)
            except Exception:
                continue
            if dist <= 0:
                continue
            if dist < MIN_DIST_MM or dist > MAX_DIST_MM:
                continue

            idx = _bin_index(angle, bin_deg)
            bin_dists.setdefault(idx, []).append(dist)
            if idx not in bin_quals or quality > bin_quals[idx]:
                bin_quals[idx] = quality

    merged = []
    hit_counts = {}

    for idx, dists in bin_dists.items():
        if not dists:
            continue
        dist_med = median(dists)
        q = bin_quals.get(idx, 0)
        ang = (idx * bin_deg) % 360.0
        merged.append((q, ang, dist_med))
        hit_counts[idx] = len(dists)

    merged.sort(key=lambda t: t[1])
    return merged, hit_counts


def validate_scan_quality_bins(merged_scan, bin_deg=ANGLE_BIN_DEG, min_coverage=MIN_COVERAGE):
    """
    Validate based on occupied bins (more stable than raw point angles).
    Returns: (is_valid, coverage_frac, max_gap_deg, point_count, message)
    """
    if not merged_scan or len(merged_scan) < 10:
        return False, 0.0, 360.0, 0, "Too few points"

    total_bins = int(round(360.0 / bin_deg))
    hit_bins = sorted({_bin_index(angle, bin_deg) for _, angle, dist in merged_scan if dist > 0})

    if len(hit_bins) < 10:
        return False, 0.0, 360.0, len(hit_bins), "Too few valid bins"

    coverage = len(hit_bins) / total_bins

    # Compute max angular gap between consecutive hit bins (bin centers).
    max_gap = 0.0
    for i in range(len(hit_bins)):
        j = (i + 1) % len(hit_bins)
        a = hit_bins[i]
        b = hit_bins[j]
        step = (b - a) % total_bins
        gap_deg = step * bin_deg
        max_gap = max(max_gap, gap_deg)

    is_valid = (coverage >= min_coverage) and (max_gap <= MAX_GAP_DEG)

    message = f"Coverage: {coverage*100:.1f}%, Max gap: {max_gap:.1f} deg, Points: {len(hit_bins)}"
    return is_valid, coverage, max_gap, len(hit_bins), message


def fill_small_gaps_for_visualization(merged_scan, bin_deg=ANGLE_BIN_DEG, max_fill_gap_deg=FILL_MAX_GAP_DEG):
    """
    Fill missing bins for nicer PLY output (linear interpolation in distance).
    Only fills gaps up to max_fill_gap_deg. Does NOT modify the original merged_scan.
    """
    if not merged_scan:
        return merged_scan

    total_bins = int(round(360.0 / bin_deg))
    by_bin = {}
    for q, ang, dist in merged_scan:
        idx = _bin_index(ang, bin_deg)
        by_bin[idx] = (q, (idx * bin_deg) % 360.0, float(dist))

    hit_bins = sorted(by_bin.keys())
    if len(hit_bins) < 2:
        return merged_scan

    filled = dict(by_bin)

    max_fill_steps = int(round(max_fill_gap_deg / bin_deg))

    for i in range(len(hit_bins)):
        a = hit_bins[i]
        b = hit_bins[(i + 1) % len(hit_bins)]
        step = (b - a) % total_bins
        if step <= 1:
            continue
        if step > max_fill_steps:
            continue

        qa, anga, da = by_bin[a]
        qb, angb, db = by_bin[b]

        for k in range(1, step):
            idx = (a + k) % total_bins
            t = k / step
            d = (1.0 - t) * da + t * db
            q = min(qa, qb)
            ang = (idx * bin_deg) % 360.0
            filled[idx] = (q, ang, d)

    out = list(filled.values())
    out.sort(key=lambda t: t[1])
    return out


def run_scan(port="auto", output_dir="data"):
    try:
        if port == "auto":
            port = get_default_port()

        lidar = RPLidar(port, baudrate=BAUD, timeout=3)

        try:
            _ = lidar.get_info()
            _ = lidar.get_health()

            lidar.start_motor()
            time.sleep(2)

            t0 = time.time()
            collected_scans = []

            last_coverage = 0.0
            plateau_count = 0

            merged_scan = None

            for s in lidar.iter_scans(max_buf_meas=2000, min_len=50):
                elapsed = time.time() - t0
                if not s or len(s) < 50:
                    if elapsed > TIMEOUT_SEC:
                        break
                    continue

                collected_scans.append(s)

                merged_scan, _ = merge_scans_robust(collected_scans, bin_deg=ANGLE_BIN_DEG)
                is_valid, coverage, max_gap, point_count, msg = validate_scan_quality_bins(
                    merged_scan, bin_deg=ANGLE_BIN_DEG, min_coverage=MIN_COVERAGE
                )

                if abs(coverage - last_coverage) < PLATEAU_EPS:
                    plateau_count += 1
                else:
                    plateau_count = 0
                last_coverage = coverage

                if is_valid:
                    break

                if plateau_count >= PLATEAU_ITERS and len(collected_scans) >= 5:
                    break

                if len(collected_scans) >= MAX_SCANS_TO_MERGE:
                    break

                if elapsed > TIMEOUT_SEC:
                    break

            if not merged_scan:
                return {
                    'success': False,
                    'point_count': 0,
                    'files': [],
                    'error': 'No scan data received',
                    'message': 'RPLidar did not return any scan data',
                    'scan_quality': {}
                }

            is_valid, coverage, max_gap, point_count, msg = validate_scan_quality_bins(
                merged_scan, bin_deg=ANGLE_BIN_DEG, min_coverage=MIN_COVERAGE
            )

            ply_scan = merged_scan
            if FILL_GAPS_FOR_PLY:
                ply_scan = fill_small_gaps_for_visualization(
                    merged_scan, bin_deg=ANGLE_BIN_DEG, max_fill_gap_deg=FILL_MAX_GAP_DEG
                )

            def to_points(scan_list):
                pts_local = []
                for q, ang, dist in scan_list:
                    if dist <= 0:
                        continue
                    r = float(dist) / 1000.0
                    th = math.radians(float(ang))
                    x = r * math.cos(th)
                    y = r * math.sin(th)
                    pts_local.append((int(q), float(ang), float(dist), x, y, 0.0))
                return pts_local

            pts_csv = to_points(merged_scan)
            pts_ply = to_points(ply_scan)

            os.makedirs(output_dir, exist_ok=True)

            ts = time.strftime("%Y%m%d_%H%M%S")
            csv_file = os.path.join(output_dir, f"scan_{ts}.csv")
            ply_file = os.path.join(output_dir, f"scan_{ts}.ply")

            with open(csv_file, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["quality", "angle_deg", "distance_mm", "x_m", "y_m", "z_m"])
                w.writerows(pts_csv)

            with open(ply_file, "w") as f:
                f.write("ply\nformat ascii 1.0\n")
                f.write(f"element vertex {len(pts_ply)}\n")
                f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
                for _, _, _, x, y, z in pts_ply:
                    f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

            status_msg = f"Scan completed: {len(pts_csv)} points"
            if len(collected_scans) > 1:
                status_msg += f" (merged {len(collected_scans)} scans)"
            if FILL_GAPS_FOR_PLY and len(pts_ply) != len(pts_csv):
                status_msg += f" (PLY filled to {len(pts_ply)} pts)"

            return {
                'success': True,
                'point_count': len(pts_csv),
                'files': [csv_file, ply_file],
                'error': None,
                'message': status_msg,
                'scan_quality': {
                    'is_valid': is_valid,
                    'coverage_percent': coverage * 100,
                    'max_gap_degrees': max_gap,
                    'scans_merged': len(collected_scans),
                    'angle_bin_deg': ANGLE_BIN_DEG,
                    'quality_message': msg
                }
            }

        finally:
            try:
                lidar.stop()
            except Exception:
                pass
            try:
                lidar.stop_motor()
            except Exception:
                pass
            lidar.disconnect()

    except Exception as e:
        return {
            'success': False,
            'point_count': 0,
            'files': [],
            'error': str(e),
            'message': f"Scan failed: {str(e)}",
            'scan_quality': {}
        }


def main():
    port = get_default_port()
    if len(sys.argv) > 1:
        port = sys.argv[1]

    print(f"Using port: {port}", flush=True)
    print("Starting scan...", flush=True)

    result = run_scan(port=port, output_dir="data")

    if result['success']:
        print(f"\n{'='*60}", flush=True)
        print("SCAN COMPLETED SUCCESSFULLY", flush=True)
        print(f"{'='*60}", flush=True)
        print(f"  Points captured (CSV): {result['point_count']}", flush=True)

        quality = result['scan_quality']
        if quality:
            print(f"  Scans merged: {quality.get('scans_merged', 1)}", flush=True)
            print(f"  Bin size: {quality.get('angle_bin_deg', ANGLE_BIN_DEG)}°", flush=True)
            print(f"  Coverage: {quality.get('coverage_percent', 0):.1f}%", flush=True)
            print(f"  Max gap: {quality.get('max_gap_degrees', 0):.1f}°", flush=True)
            print(f"  Quality: {quality.get('quality_message', 'N/A')}", flush=True)

        print("\n  Files saved:", flush=True)
        for file in result['files']:
            print(f"    - {file}", flush=True)
        print(f"{'='*60}\n", flush=True)
    else:
        print(f"\n{'='*60}", flush=True)
        print("SCAN FAILED", flush=True)
        print(f"{'='*60}", flush=True)
        print(f"  Error: {result['error']}", flush=True)
        print(f"  Message: {result['message']}", flush=True)
        print(f"{'='*60}\n", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
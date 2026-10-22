#!/usr/bin/env python3
"""Download a short single-exposure stereo segment of the FoMo dataset and
convert it to the BVSLAM sequence layout.

FoMo (https://fomo.norlab.ulaval.ca, CC BY 4.0) is a multi-season robot
navigation dataset recorded in the Montmorency Forest. This fetches ZED X
stereo images of the winter "red" trajectory (2025-01-30) from its public
bucket and writes:

    data/fomo_red_winter/images_{left,right}/<timestamp_ns>.png
    data/fomo_red_winter/calibration/stereo_calibration_{left,right}.yaml
    data/fomo_red_winter/ground_truth_ppk.txt   FoMo PPK-GNSS positions (TUM format, for reference)

Usage: python3 scripts/download_fomo.py [--start 150] [--duration 15] [--step 2] [--out data/fomo_red_winter]
"""

import argparse
import json
import re
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

BUCKET = "https://fomo-dataset.s3.amazonaws.com"
RECORDING = "data/2025-01-29/red_2025-01-30-09-58/"


def list_keys(prefix: str) -> list[str]:
    keys, token = [], None
    while True:
        url = f"{BUCKET}/?list-type=2&max-keys=1000&prefix={prefix}"
        if token:
            url += "&continuation-token=" + urllib.parse.quote(token)
        page = urllib.request.urlopen(url).read().decode()
        keys += re.findall(r"<Key>([^<]+)</Key>", page)
        token = (re.findall(r"<NextContinuationToken>([^<]+)", page) or [None])[0]
        if token is None:
            return keys


def fetch_json(path: str):
    return json.load(urllib.request.urlopen(f"{BUCKET}/{RECORDING}{path}"))


def write_calibration(path: Path, k: list[float], p: list[float]) -> None:
    """ROS camera_info-style YAML (the images are already rectified and undistorted)."""
    fmt = lambda values: "[" + ", ".join(repr(float(v)) for v in values) + "]"
    path.write_text(
        "image_width: 1920\nimage_height: 1200\n"
        f"camera_matrix:\n  rows: 3\n  cols: 3\n  data: {fmt(k)}\n"
        "distortion_model: plumb_bob\n"
        "distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: [0.0, 0.0, 0.0, 0.0, 0.0]\n"
        "rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n"
        f"projection_matrix:\n  rows: 3\n  cols: 4\n  data: {fmt(p)}\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--start", type=float, default=150.0, help="segment start, seconds after the first image")
    parser.add_argument("--duration", type=float, default=15.0, help="segment length in seconds")
    parser.add_argument("--step", type=int, default=2, help="keep one stereo pair every STEP (the ZED X runs at ~10 Hz)")
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "data" / "fomo_red_winter")
    args = parser.parse_args()

    # FoMo image names are timestamps in microseconds.
    stamps = sorted(int(Path(k).stem) for k in list_keys(RECORDING + "zedx_left/"))
    t0 = stamps[0] + args.start * 1e6
    selected = [t for t in stamps if t0 <= t < t0 + args.duration * 1e6][:: args.step]
    print(f"Downloading {len(selected)} stereo pairs (~{len(selected) * 7.4:.0f} MB) to {args.out}")

    jobs = []
    for side in ("left", "right"):
        (args.out / f"images_{side}").mkdir(parents=True, exist_ok=True)
        jobs += [(f"{RECORDING}zedx_{side}/{t}.png", args.out / f"images_{side}" / f"{t * 1000}.png") for t in selected]

    def download(job):
        key, dest = job
        if not dest.exists():
            urllib.request.urlretrieve(f"{BUCKET}/{key}", dest)

    with ThreadPoolExecutor(16) as pool:
        list(pool.map(download, jobs))

    # Both ZED X images share the left intrinsics; the right projection carries the baseline.
    left = fetch_json("calib/zedx_left.json")
    baseline = next(t for t in fetch_json("calib/transforms.json") if t["from"] == "zedx_left" and t["to"] == "zedx_right")["position"]["x"]
    calibration = args.out / "calibration"
    calibration.mkdir(exist_ok=True)
    p_right = list(left["p"])
    p_right[3] = -left["k"][0] * baseline
    write_calibration(calibration / "stereo_calibration_left.yaml", left["k"], left["p"])
    write_calibration(calibration / "stereo_calibration_right.yaml", left["k"], p_right)

    gt = urllib.request.urlopen(f"{BUCKET}/{RECORDING}gt.txt").read().decode().splitlines()
    t_min, t_max = selected[0] / 1e6, selected[-1] / 1e6
    (args.out / "ground_truth_ppk.txt").write_text("".join(l + "\n" for l in gt if l.strip() and t_min <= float(l.split()[0]) <= t_max))
    print(f"FoMo segment ready in {args.out} (stereo baseline {baseline:.4f} m)")


if __name__ == "__main__":
    main()

"""End-to-end smoke test on a small synthetic bracketed stereo sequence (CPU).

The scene is a textured corridor (floor, ceiling, two walls and an end wall)
rendered with a rectified pinhole stereo rig moving forward along the optical
axis. Images are written as raw RGGB Bayer PNGs with the same layout and
metadata as the real data; the exposure zone cycles 1, 0, 1, 2 (MAE, SAE,
MAE, LAE) and scales the image brightness accordingly.

Run `python tests/test_smoke.py <dir>` to only write the sequence to <dir>.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import cv2
import numpy as np

N_FRAMES = 12
WIDTH, HEIGHT = 512, 384
FX = FY = 400.0
CX, CY = WIDTH / 2.0, HEIGHT / 2.0
BASELINE_M = 0.12
STEP_M = 0.10  # forward motion per frame
DT_NS = 100_000_000  # 10 Hz
T0_NS = 1_000_000_000_000
ZONE_CYCLE = ["1", "0", "1", "2"]
ZONE_GAIN = {"0": 0.6, "1": 1.0, "2": 1.5}


def _texture(rng: np.random.Generator, size: int = 1024) -> np.ndarray:
    """Multi-octave smoothed noise in [0, 1], tileable via wrap-around sampling."""
    tex = np.zeros((size, size), np.float32)
    for cells, weight in ((8, 1.0), (32, 0.6), (128, 0.4), (512, 0.25)):
        coarse = rng.random((cells, cells)).astype(np.float32)
        tex += weight * cv2.resize(coarse, (size, size), interpolation=cv2.INTER_CUBIC)
    tex -= tex.min()
    return tex / tex.max()


def _render(tex: list[np.ndarray], cam_x: float, cam_z: float) -> np.ndarray:
    """Grayscale image (float, 0-255) of the corridor seen from (cam_x, 0, cam_z), looking along +z."""
    u, v = np.meshgrid(np.arange(WIDTH, dtype=np.float64), np.arange(HEIGHT, dtype=np.float64))
    dx, dy = (u - CX) / FX, (v - CY) / FY  # ray direction (dx, dy, 1)
    # planes: x = -2, x = +2, y = +1.2 (floor, y down), y = -1.5 (ceiling), z = 25 (end wall)
    planes = [(0, -2.0), (0, 2.0), (1, 1.2), (1, -1.5), (2, 25.0)]
    best_t = np.full(u.shape, np.inf)
    image = np.zeros(u.shape)
    for k, (axis, offset) in enumerate(planes):
        with np.errstate(divide="ignore", invalid="ignore"):
            if axis == 0:
                t = (offset - cam_x) / dx
            elif axis == 1:
                t = offset / dy
            else:
                t = np.full(u.shape, offset - cam_z)
            t = np.where(np.isfinite(t) & (t > 0), t, np.inf)
            hit = t < best_t
            px, py, pz = cam_x + t * dx, t * dy, cam_z + t
        # in-plane coordinates (m, offset so texture seams fall outside the view) -> texture pixels, wrapped
        a, b = {0: (pz, py), 1: (px, pz), 2: (px, py)}[axis]
        ta = np.mod((np.where(hit, a, 0) + 6.0) * 64.0, tex[k].shape[1]).astype(np.float32)
        tb = np.mod((np.where(hit, b, 0) + 6.0) * 64.0, tex[k].shape[0]).astype(np.float32)
        sample = cv2.remap(tex[k], ta, tb, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        image = np.where(hit, 40.0 + 150.0 * sample, image)
        best_t = np.where(hit, t, best_t)
    return image


def _calibration_yaml(path: Path, P: np.ndarray) -> None:
    K = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    fmt = lambda xs: "[" + ", ".join(f"{x:.6f}" for x in xs) + "]"  # noqa: E731
    path.write_text(
        f"image_width: {WIDTH}\nimage_height: {HEIGHT}\ncamera_name: synthetic\n"
        f"camera_matrix:\n  rows: 3\n  cols: 3\n  data: {fmt(K)}\n"
        f"distortion_model: plumb_bob\n"
        f"distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: {fmt([0.0] * 5)}\n"
        f"rectification_matrix:\n  rows: 3\n  cols: 3\n  data: {fmt(R)}\n"
        f"projection_matrix:\n  rows: 3\n  cols: 4\n  data: {fmt(P.ravel())}\n"
    )


def make_sequence(root: Path, n_frames: int = N_FRAMES, seed: int = 0) -> Path:
    """Write a synthetic sequence to `root` (data + calibration/). Returns `root`."""
    rng = np.random.default_rng(seed)
    textures = [_texture(rng) for _ in range(5)]
    for sub in ("images_left", "images_right", "images_meta_left", "calibration"):
        (root / sub).mkdir(parents=True, exist_ok=True)

    P1 = np.array([[FX, 0, CX, 0], [0, FY, CY, 0], [0, 0, 1, 0]], dtype=np.float64)
    P2 = P1.copy()
    P2[0, 3] = -FX * BASELINE_M
    _calibration_yaml(root / "calibration" / "stereo_calibration_left.yaml", P1)
    _calibration_yaml(root / "calibration" / "stereo_calibration_right.yaml", P2)

    rows = []
    for i in range(n_frames):
        ts = T0_NS + i * DT_NS
        zone = ZONE_CYCLE[i % len(ZONE_CYCLE)]
        for side, cam_x in (("left", 0.0), ("right", BASELINE_M)):
            gray = np.clip(_render(textures, cam_x, i * STEP_M) * ZONE_GAIN[zone], 0, 255).astype(np.uint8)
            # Uniform gray scene: every Bayer photosite gets the same value.
            cv2.imwrite(str(root / f"images_{side}" / f"{ts}.png"), gray)
        rows.append({"timestamp": ts, "sequence_index": i % 4, "ae_zone": zone, "exposure_us": 2000.0 * ZONE_GAIN[zone],
                     "gain_db": 0.0, "converged": "true"})
    with open(root / "images_meta_left" / "images_meta_left.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return root


def _read_tum(path: Path) -> np.ndarray:
    return np.loadtxt(path, ndmin=2)


def test_smoke(tmp_path: Path) -> None:
    from bvslam.cli import main

    data = make_sequence(tmp_path / "seq")
    config = tmp_path / "cpu.yaml"
    config.write_text("superpoint:\n  device: cpu\nlightglue:\n  device: cpu\n")
    out = tmp_path / "out" / "trajectory.tum"
    main(["--data-dir", str(data), "--config", str(config), "--out", str(out), "--viz", "none"])

    for path in (out, out.with_name("trajectory_global_ba.tum")):
        tum = _read_tum(path)
        assert tum.shape == (N_FRAMES, 8)
        assert np.all(np.diff(tum[:, 0]) > 0)
        travelled = tum[-1, 1:4] - tum[0, 1:4]
        expected = (N_FRAMES - 1) * STEP_M
        # forward (+z in the camera frame), roughly the true distance, little sideways drift
        assert abs(travelled[2] - expected) < 0.25 * expected, travelled
        assert np.linalg.norm(travelled[:2]) < 0.1 * expected, travelled


if __name__ == "__main__":
    make_sequence(Path(sys.argv[1]))

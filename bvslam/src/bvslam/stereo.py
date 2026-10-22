"""Stereo calibration loading and triangulation of rectified stereo observations.

Calibration files are ROS-style `stereo_calibration_{left,right}.yaml`
(camera_matrix, distortion_coefficients, rectification_matrix,
projection_matrix). Images are not warped: only matched keypoints are
undistorted and rectified (cv2.undistortPoints with R, P). The graph's pose
convention is the rectified left-camera frame, so all outputs stay in it.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import gtsam
import numpy as np
import yaml


@dataclass
class StereoRig:
    K_left: np.ndarray
    dist_left: np.ndarray
    K_right: np.ndarray
    dist_right: np.ndarray
    R1: np.ndarray  # rectification rotation, original left frame -> rectified left frame
    R2: np.ndarray  # rectification rotation, original right frame -> rectified right frame
    P1: np.ndarray  # rectified left projection matrix (3x4)
    P2: np.ndarray  # rectified right projection matrix (3x4)
    baseline_m: float


def _matrix(raw: dict, key: str, shape: tuple[int, ...]) -> np.ndarray:
    return np.array(raw[key]["data"], dtype=np.float64).reshape(shape)


def load_stereo_rig(calib_dir: str | Path) -> StereoRig:
    calib_dir = Path(calib_dir)
    raw = {}
    for side in ("left", "right"):
        with open(calib_dir / f"stereo_calibration_{side}.yaml", "r") as f:
            raw[side] = yaml.safe_load(f)
    raw_l, raw_r = raw["left"], raw["right"]

    P1 = _matrix(raw_l, "projection_matrix", (3, 4))
    P2 = _matrix(raw_r, "projection_matrix", (3, 4))
    return StereoRig(
        K_left=_matrix(raw_l, "camera_matrix", (3, 3)),
        dist_left=np.array(raw_l["distortion_coefficients"]["data"], dtype=np.float64),
        K_right=_matrix(raw_r, "camera_matrix", (3, 3)),
        dist_right=np.array(raw_r["distortion_coefficients"]["data"], dtype=np.float64),
        R1=_matrix(raw_l, "rectification_matrix", (3, 3)),
        R2=_matrix(raw_r, "rectification_matrix", (3, 3)),
        P1=P1,
        P2=P2,
        baseline_m=-P2[0, 3] / P1[0, 0],
    )


def rectify_points(pts: np.ndarray, K: np.ndarray, dist: np.ndarray, R: np.ndarray, P: np.ndarray) -> np.ndarray:
    """Undistort + rectify 2D pixel points into the rectified pixel frame defined by (R, P)."""
    if len(pts) == 0:
        return np.empty((0, 2))
    pts = pts.reshape(-1, 1, 2).astype(np.float64)
    return cv2.undistortPoints(pts, K, dist, R=R, P=P).reshape(-1, 2)


def stereo_calibration(rig: StereoRig) -> gtsam.Cal3_S2Stereo:
    """GTSAM rectified-stereo calibration from the rig's P1 intrinsics and baseline."""
    fx, fy = rig.P1[0, 0], rig.P1[1, 1]
    u0, v0 = rig.P1[0, 2], rig.P1[1, 2]
    return gtsam.Cal3_S2Stereo(fx, fy, 0.0, u0, v0, rig.baseline_m)


@dataclass
class StereoObservations:
    """Per-frame stereo observations in the rectified left-camera frame."""

    indices_left: np.ndarray  # (M,) indices into the left frame's keypoint array
    stereo_points: np.ndarray  # (M, 3): [uL, uR, v], rectified pixel coordinates
    points3d: np.ndarray  # (M, 3), rectified left-camera frame


def _empty_observations() -> StereoObservations:
    return StereoObservations(np.empty((0,), dtype=np.int64), np.empty((0, 3)), np.empty((0, 3)))


def compute_stereo_observations(
    kp_left: np.ndarray,
    kp_right: np.ndarray,
    match_indices_left: np.ndarray,
    match_indices_right: np.ndarray,
    rig: StereoRig,
    min_disparity_px: float = 1.0,
    max_depth_m: float = 60.0,
) -> StereoObservations:
    """Rectify and triangulate left/right matches; keep disparity > min and 0 < depth < max."""
    if len(match_indices_left) == 0:
        return _empty_observations()

    rect_l = rectify_points(kp_left[match_indices_left], rig.K_left, rig.dist_left, rig.R1, rig.P1)
    rect_r = rectify_points(kp_right[match_indices_right], rig.K_right, rig.dist_right, rig.R2, rig.P2)

    disparity = rect_l[:, 0] - rect_r[:, 0]
    valid = disparity > min_disparity_px
    if not np.any(valid):
        # cv2.triangulatePoints rejects an empty input.
        return _empty_observations()

    pts4d = cv2.triangulatePoints(rig.P1, rig.P2, rect_l[valid].T, rect_r[valid].T)
    pts3d_rect = (pts4d[:3] / pts4d[3]).T

    depth_ok = (pts3d_rect[:, 2] > 0) & (pts3d_rect[:, 2] < max_depth_m)

    kept = np.nonzero(valid)[0][depth_ok]
    stereo_points = np.stack([rect_l[kept, 0], rect_r[kept, 0], rect_l[kept, 1]], axis=1)
    return StereoObservations(match_indices_left[kept], stereo_points, pts3d_rect[depth_ok])

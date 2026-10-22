"""Robust stereo reprojection factor for landmark observations."""

from __future__ import annotations

import gtsam
import numpy as np


def make_stereo_observation_factor(
    pose_key: int,
    landmark_key: int,
    stereo_point: np.ndarray,
    K_stereo: gtsam.Cal3_S2Stereo,
    pixel_sigma: float,
    huber_k: float,
    depth_scaled_noise: bool = True,
    depth_scaled_noise_reference_m: float = 10.0,
) -> gtsam.GenericStereoFactor3D:
    """GenericStereoFactor3D for a rectified `[uL, uR, v]` observation, with a Huber kernel.

    With `depth_scaled_noise`, the pixel noise grows linearly with the
    observation's disparity-implied depth beyond `depth_scaled_noise_reference_m`:
    sigma = pixel_sigma * max(1, depth / reference). This down-weights far,
    poorly conditioned observations.
    """
    measured = gtsam.StereoPoint2(float(stereo_point[0]), float(stereo_point[1]), float(stereo_point[2]))
    effective_pixel_sigma = pixel_sigma
    if depth_scaled_noise:
        disparity = stereo_point[0] - stereo_point[1]
        if disparity > 1e-6:
            depth = (K_stereo.fx() * K_stereo.baseline()) / disparity
            effective_pixel_sigma = pixel_sigma * max(1.0, depth / depth_scaled_noise_reference_m)
    base_noise = gtsam.noiseModel.Isotropic.Sigma(3, effective_pixel_sigma)
    robust_noise = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(huber_k), base_noise)
    return gtsam.GenericStereoFactor3D(measured, robust_noise, pose_key, landmark_key, K_stereo)

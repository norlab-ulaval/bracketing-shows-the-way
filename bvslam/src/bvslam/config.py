"""YAML configuration loading into typed dataclasses.

Every default below equals the value in configs/paper.yaml, so a key missing
from a user config falls back to the paper setting. Unknown keys raise.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass
class DatasetConfig:
    bayer_pattern: str = "RGGB"  # RGGB | BGGR | GRBG | GBRG | none


@dataclass
class PreprocessingConfig:
    # Drop frames whose mean left-image intensity (0-255, after demosaicing)
    # lies outside [min, max]: fully black or saturated frames carry no features.
    drop_low_info_frames: bool = True
    drop_low_info_min_brightness: float = 1.0
    drop_low_info_max_brightness: float = 245.0


@dataclass
class SuperPointConfig:
    device: str = "cuda"  # falls back to CPU when CUDA is unavailable
    max_keypoints: int = 1000  # kept per image after bucketing (3x are detected first)


@dataclass
class TrackingConfig:
    # Keypoint bucketing grid; 1x1 keeps the max_keypoints highest-score keypoints.
    grid_rows: int = 1
    grid_cols: int = 1
    # Skip direct temporal matching between the shortest (SAE) and longest
    # (LAE) exposures; they stay connected through the medium (MAE) frames.
    block_lae_sae_matches: bool = True


@dataclass
class LightGlueConfig:
    device: str = "cuda"
    min_confidence: float = 0.9  # tau: matches with a lower LightGlue score are discarded


@dataclass
class StereoConfig:
    min_disparity_px: float = 1.0  # reject near-infinite depth
    max_depth_m: float = 60.0  # reject far, poorly triangulated points
    pixel_sigma: float = 1.0  # rectified-pixel reprojection noise
    huber_k: float = 1.345  # Huber kernel threshold
    landmark_prior_sigma: float = 3.0  # m, weak prior at each landmark's initial triangulation
    # Scale pixel_sigma by max(1, depth / depth_scaled_noise_reference_m).
    depth_scaled_noise: bool = True
    depth_scaled_noise_reference_m: float = 10.0


@dataclass
class MotionPriorConfig:
    # Identity BetweenFactorPose3 between consecutive frames: keeps every pose
    # well-posed (also on frames without landmark observations) while barely
    # constraining real motion.
    rotation_sigma: float = 3.14159  # rad
    translation_sigma: float = 10.0  # m
    initial_pose_prior_sigma: float = 1.0e-3  # gauge prior on the first pose
    # Each frame also carries a zero-initialized 6-D velocity variable with
    # this standalone prior. Velocities are not coupled to the poses.
    velocity_prior_sigma: float = 1.0


@dataclass
class GraphConfig:
    vo_lookback: int = 4  # previous frames matched against the current one (window W = 5)
    smoother_lag_s: float = 5.0  # IncrementalFixedLagSmoother window
    # Batch Levenberg-Marquardt over every factor after the incremental pass,
    # written to <out>_global_ba.tum.
    global_bundle_adjust: bool = True


@dataclass
class Config:
    dataset: DatasetConfig = field(default_factory=DatasetConfig)
    preprocessing: PreprocessingConfig = field(default_factory=PreprocessingConfig)
    superpoint: SuperPointConfig = field(default_factory=SuperPointConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    lightglue: LightGlueConfig = field(default_factory=LightGlueConfig)
    stereo: StereoConfig = field(default_factory=StereoConfig)
    motion_prior: MotionPriorConfig = field(default_factory=MotionPriorConfig)
    graph: GraphConfig = field(default_factory=GraphConfig)

    @staticmethod
    def load(path: str | Path) -> "Config":
        with open(path, "r") as f:
            raw = yaml.safe_load(f) or {}
        sections = {
            "dataset": DatasetConfig,
            "preprocessing": PreprocessingConfig,
            "superpoint": SuperPointConfig,
            "tracking": TrackingConfig,
            "lightglue": LightGlueConfig,
            "stereo": StereoConfig,
            "motion_prior": MotionPriorConfig,
            "graph": GraphConfig,
        }
        unknown = set(raw) - set(sections)
        if unknown:
            raise ValueError(f"unknown config section(s): {sorted(unknown)}")
        return Config(**{name: cls(**(raw.get(name) or {})) for name, cls in sections.items()})

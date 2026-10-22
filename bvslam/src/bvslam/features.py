"""SuperPoint keypoint extraction with grid-based score bucketing.

Uses the reference SuperPoint implementation from the `lightglue` package
(github.com/cvg/LightGlue), whose descriptors match kornia's "superpoint"
LightGlue weights used in matching.py.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch
from lightglue import SuperPoint

from bvslam.config import SuperPointConfig, TrackingConfig


@dataclass
class FrameFeatures:
    keypoints: np.ndarray  # (N, 2) float32, (x, y) pixel coords
    descriptors: np.ndarray  # (N, D) float32
    scores: np.ndarray  # (N,) float32


def resolve_device(requested: str) -> torch.device:
    if requested == "cuda" and not torch.cuda.is_available():
        return torch.device("cpu")
    return torch.device(requested)


def bucket_keypoints(
    keypoints: np.ndarray,
    scores: np.ndarray,
    image_shape: tuple[int, int],
    max_corners: int,
    grid_rows: int,
    grid_cols: int,
) -> np.ndarray:
    """Return indices of keypoints kept after per-cell score-based top-K selection.

    Distributes the `max_corners` budget evenly across a grid_rows x grid_cols
    grid so features aren't dominated by a single high-texture region.
    """
    h, w = image_shape[:2]
    n_cells = grid_rows * grid_cols
    per_cell_budget = max(1, max_corners // n_cells)

    cell_w = w / grid_cols
    cell_h = h / grid_rows

    col_idx = np.clip((keypoints[:, 0] // cell_w).astype(int), 0, grid_cols - 1)
    row_idx = np.clip((keypoints[:, 1] // cell_h).astype(int), 0, grid_rows - 1)
    cell_id = row_idx * grid_cols + col_idx

    keep: list[int] = []
    for c in range(n_cells):
        idx_in_cell = np.nonzero(cell_id == c)[0]
        if idx_in_cell.size == 0:
            continue
        order = np.argsort(-scores[idx_in_cell])[:per_cell_budget]
        keep.extend(idx_in_cell[order].tolist())

    return np.array(keep, dtype=np.int64)


class SuperPointExtractor:
    def __init__(self, cfg: SuperPointConfig, tracking_cfg: TrackingConfig):
        self.cfg = cfg
        self.tracking_cfg = tracking_cfg
        self.device = resolve_device(cfg.device)
        # Detect 3x the budget so bucket_keypoints has candidates to choose from.
        self.model = SuperPoint(max_num_keypoints=cfg.max_keypoints * 3).to(self.device).eval()

    @torch.inference_mode()
    def extract(self, image: np.ndarray) -> FrameFeatures:
        """`image` is a single-channel grayscale uint8 image."""
        h, w = image.shape[:2]
        tensor = torch.from_numpy(image).float().unsqueeze(0).unsqueeze(0)
        if image.dtype == np.uint8:
            tensor = tensor / 255.0
        tensor = tensor.to(self.device)

        out = self.model({"image": tensor})
        keypoints = out["keypoints"][0].detach().cpu().numpy().astype(np.float32)
        descriptors = out["descriptors"][0].detach().cpu().numpy().astype(np.float32)
        scores = out["keypoint_scores"][0].detach().cpu().numpy().astype(np.float32)

        if keypoints.shape[0] == 0:
            return FrameFeatures(keypoints, descriptors, scores)

        keep = bucket_keypoints(
            keypoints,
            scores,
            (h, w),
            self.cfg.max_keypoints,
            self.tracking_cfg.grid_rows,
            self.tracking_cfg.grid_cols,
        )
        return FrameFeatures(keypoints[keep], descriptors[keep], scores[keep])

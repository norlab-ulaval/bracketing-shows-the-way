"""LightGlue matching of SuperPoint features (kornia implementation, "superpoint" weights)."""

from __future__ import annotations

from dataclasses import dataclass

import kornia.feature as KF
import numpy as np
import torch

from bvslam.config import LightGlueConfig
from bvslam.features import FrameFeatures, resolve_device


@dataclass
class MatchResult:
    indices_a: np.ndarray  # (M,) indices into features_a
    indices_b: np.ndarray  # (M,) indices into features_b
    confidence: np.ndarray  # (M,) float32, higher = better


def _empty_match() -> MatchResult:
    return MatchResult(np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64), np.empty(0, dtype=np.float32))


class LightGlueMatcher:
    def __init__(self, cfg: LightGlueConfig):
        self.cfg = cfg
        self.device = resolve_device(cfg.device)
        self.matcher = KF.LightGlueMatcher("superpoint").to(self.device).eval()

    @torch.inference_mode()
    def match(
        self,
        feats_a: FrameFeatures,
        shape_a: tuple[int, int],
        feats_b: FrameFeatures,
        shape_b: tuple[int, int],
    ) -> MatchResult:
        if feats_a.keypoints.shape[0] == 0 or feats_b.keypoints.shape[0] == 0:
            return _empty_match()

        desc_a = torch.from_numpy(feats_a.descriptors).to(self.device)
        desc_b = torch.from_numpy(feats_b.descriptors).to(self.device)
        kp_a = torch.from_numpy(feats_a.keypoints).to(self.device)
        kp_b = torch.from_numpy(feats_b.keypoints).to(self.device)

        laf_a = KF.laf_from_center_scale_ori(kp_a[None])
        laf_b = KF.laf_from_center_scale_ori(kp_b[None])

        scores, idxs = self.matcher(desc_a, desc_b, laf_a, laf_b, hw1=shape_a, hw2=shape_b)

        if idxs.numel() == 0:
            return _empty_match()

        idxs_np = idxs.detach().cpu().numpy()
        # kornia returns the LightGlue matching score in [0, 1] (higher = better).
        confidence = scores.detach().cpu().numpy().reshape(-1)
        keep = confidence >= self.cfg.min_confidence

        return MatchResult(idxs_np[keep, 0], idxs_np[keep, 1], confidence[keep].astype(np.float32))

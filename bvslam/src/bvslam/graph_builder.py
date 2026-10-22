"""Incremental stereo bundle adjustment over a bracketed-exposure sequence.

Each frame's left image is matched (SuperPoint + LightGlue) against its right
image, giving rectified stereo observations, and against the previous
`vo_lookback` frames. Temporal matches link keypoints to persistent landmarks
(`LandmarkTracker`); each sighting adds a robust `GenericStereoFactor3D`.
Consecutive poses are tied by a loose identity `BetweenFactorPose3`.

The backend is `gtsam_unstable.IncrementalFixedLagSmoother` (stock PyPI
gtsam wheel): every variable is tagged with the timestamp it was last touched
at and marginalized once it falls `smoother_lag_s` behind the newest frame.
`global_bundle_adjust()` re-optimizes every factor ever added in one batch.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass

import gtsam
import gtsam_unstable
import numpy as np

from bvslam.config import Config
from bvslam.dataset import FrameInfo
from bvslam.factors import make_stereo_observation_factor
from bvslam.features import FrameFeatures, SuperPointExtractor
from bvslam.imaging import load_gray
from bvslam.landmarks import LandmarkTracker
from bvslam.matching import LightGlueMatcher
from bvslam.stereo import StereoObservations, StereoRig, compute_stereo_observations, stereo_calibration

log = logging.getLogger(__name__)


def _pose_key(idx: int) -> int:
    return gtsam.symbol("x", idx)


def _vel_key(idx: int) -> int:
    return gtsam.symbol("v", idx)


def _landmark_key(landmark_id: int) -> int:
    return gtsam.symbol("l", landmark_id)


def _triangulate_landmark(
    current_estimate: gtsam.Values, K_stereo: gtsam.Cal3_S2Stereo, pose_frame_idx: int, stereo_point: np.ndarray
) -> np.ndarray | None:
    """Backproject a stereo observation into a landmark initial value, or None if degenerate."""
    pose_key = _pose_key(pose_frame_idx)
    if not current_estimate.exists(pose_key):
        # The seeding frame's pose was already marginalized: the lag is
        # time-based, so after a long gap (e.g. many dropped dark frames)
        # a frame within vo_lookback can be older than smoother_lag_s.
        return None
    pose_est = current_estimate.atPose3(pose_key)
    camera = gtsam.StereoCamera(pose_est, K_stereo)
    point = camera.backproject(gtsam.StereoPoint2(float(stereo_point[0]), float(stereo_point[1]), float(stereo_point[2])))
    if not np.all(np.isfinite(point)):
        return None
    return point


@dataclass
class FrameResult:
    frame: FrameInfo
    pose: gtsam.Pose3
    velocity: np.ndarray
    n_landmark_observations: int


class PoseGraphBuilder:
    def __init__(self, cfg: Config, rig: StereoRig):
        self.cfg = cfg
        self.rig = rig
        self.K_stereo = stereo_calibration(rig)
        self.extractor = SuperPointExtractor(cfg.superpoint, cfg.tracking)
        self.matcher = LightGlueMatcher(cfg.lightglue)
        self.landmark_tracker = LandmarkTracker()

        self.smoother = gtsam_unstable.IncrementalFixedLagSmoother(cfg.graph.smoother_lag_s, gtsam.ISAM2Params())
        self.current_estimate = gtsam.Values()

        # Every factor/initial value ever added (no marginalization), for
        # global_bundle_adjust(). Backend-reset recovery priors are not added.
        self._full_graph = gtsam.NonlinearFactorGraph()
        self._full_initial = gtsam.Values()

        # Feature extraction and stereo matching for upcoming frames run on a
        # background thread while the main thread updates the smoother
        # (PyTorch and GTSAM both release the GIL).
        self._prefetch_pool = ThreadPoolExecutor(max_workers=1)

        self._feature_cache: dict[int, tuple[FrameFeatures, tuple[int, int]]] = {}
        self._right_feature_cache: dict[int, FrameFeatures] = {}
        self._stereo_obs_cache: dict[int, StereoObservations] = {}
        self._prefetch_futures: dict[int, Future[StereoObservations]] = {}
        self.results: list[FrameResult] = []
        self.n_backend_resets = 0
        # Frame in which each landmark was seeded (read by the visualization only).
        self.landmark_first_frame: dict[int, int] = {}
        self._earliest_valid_pose_idx = 0  # poses before a backend reset no longer exist in the smoother

    def _load_image(self, path) -> np.ndarray:
        return load_gray(path, self.cfg.dataset.bayer_pattern)

    def _get_left_features(self, idx: int, frame: FrameInfo) -> tuple[FrameFeatures, tuple[int, int]]:
        cached = self._feature_cache.get(idx)
        if cached is not None:
            return cached
        image = self._load_image(frame.image_path)
        feats = self.extractor.extract(image)
        entry = (feats, image.shape[:2])
        self._feature_cache[idx] = entry
        return entry

    def _get_right_features(self, idx: int, frame: FrameInfo) -> FrameFeatures:
        cached = self._right_feature_cache.get(idx)
        if cached is not None:
            return cached
        image = self._load_image(frame.right_image_path)
        feats = self.extractor.extract(image)
        self._right_feature_cache[idx] = feats
        return feats

    def _get_stereo_observations(self, idx: int, frame: FrameInfo) -> StereoObservations:
        cached = self._stereo_obs_cache.get(idx)
        if cached is not None:
            return cached
        future = self._prefetch_futures.pop(idx, None)
        if future is not None:
            # Wait for the in-flight prefetch instead of computing it twice.
            return future.result()
        return self._compute_stereo_observations(idx, frame)

    def _compute_stereo_observations(self, idx: int, frame: FrameInfo) -> StereoObservations:
        feats_left, shape_left = self._get_left_features(idx, frame)
        feats_right = self._get_right_features(idx, frame)
        match = self.matcher.match(feats_left, shape_left, feats_right, shape_left)
        obs = compute_stereo_observations(
            feats_left.keypoints,
            feats_right.keypoints,
            match.indices_a,
            match.indices_b,
            self.rig,
            self.cfg.stereo.min_disparity_px,
            self.cfg.stereo.max_depth_m,
        )
        self._stereo_obs_cache[idx] = obs
        return obs

    def _evict_old_features(self, current_idx: int) -> None:
        cutoff = current_idx - self.cfg.graph.vo_lookback
        for cache in (self._feature_cache, self._right_feature_cache, self._stereo_obs_cache):
            for k in [k for k in cache if k < cutoff]:
                del cache[k]
        self.landmark_tracker.evict_before(cutoff)

    def _observation_factor(self, frame_idx: int, landmark_id: int, stereo_point: np.ndarray) -> gtsam.GenericStereoFactor3D:
        stereo = self.cfg.stereo
        return make_stereo_observation_factor(
            _pose_key(frame_idx),
            _landmark_key(landmark_id),
            stereo_point,
            self.K_stereo,
            stereo.pixel_sigma,
            stereo.huber_k,
            depth_scaled_noise=stereo.depth_scaled_noise,
            depth_scaled_noise_reference_m=stereo.depth_scaled_noise_reference_m,
        )

    def _emit_landmark_observations(
        self,
        idx: int,
        frames: list[FrameInfo],
        graph: gtsam.NonlinearFactorGraph,
        initial: gtsam.Values,
        landmark_timestamps: dict[int, float],
    ) -> int:
        """Add landmark factors/values for frame `idx` against its lookback window.

        Records each touched landmark's timestamp (this frame's, or frame j's
        for a newly seeded landmark) in `landmark_timestamps` for the
        fixed-lag smoother. Returns the number of observation factors added.
        """
        frame = frames[idx]
        feats_i, shape_i = self._get_left_features(idx, frame)
        obs_i = self._get_stereo_observations(idx, frame)
        stereo_by_idx_i = {int(k): sp for k, sp in zip(obs_i.indices_left, obs_i.stereo_points)}

        n_obs = 0
        lookback_start = max(0, idx - self.cfg.graph.vo_lookback, self._earliest_valid_pose_idx)
        for j in range(lookback_start, idx):
            if self.cfg.tracking.block_lae_sae_matches and {frame.slot_label, frames[j].slot_label} == {"LAE", "SAE"}:
                continue
            feats_j, shape_j = self._get_left_features(j, frames[j])
            obs_j = self._get_stereo_observations(j, frames[j])
            stereo_by_idx_j = {int(k): sp for k, sp in zip(obs_j.indices_left, obs_j.stereo_points)}

            match = self.matcher.match(feats_j, shape_j, feats_i, shape_i)
            if match.indices_a.shape[0] == 0:
                continue

            for idx_a_raw, idx_b_raw in zip(match.indices_a, match.indices_b):
                idx_a, idx_b = int(idx_a_raw), int(idx_b_raw)

                if self.landmark_tracker.landmark_id_at(idx, idx_b) is not None:
                    continue  # already linked to a landmark via an earlier j this frame

                landmark_id = self.landmark_tracker.landmark_id_at(j, idx_a)
                stereo_point_i = stereo_by_idx_i.get(idx_b)

                if landmark_id is None:
                    # Seed a new landmark only if both endpoints have stereo
                    # depth, so it starts with two observations.
                    stereo_point_j = stereo_by_idx_j.get(idx_a)
                    if stereo_point_j is None or stereo_point_i is None:
                        continue
                    point_init = _triangulate_landmark(self.current_estimate, self.K_stereo, j, stereo_point_j)
                    if point_init is None:
                        continue
                    landmark_id, _ = self.landmark_tracker.get_or_create(j, idx_a)
                    self.landmark_first_frame[landmark_id] = j
                    landmark_timestamps[landmark_id] = frames[j].timestamp_s
                    initial.insert(_landmark_key(landmark_id), point_init)
                    graph.add(
                        gtsam.PriorFactorPoint3(
                            _landmark_key(landmark_id),
                            point_init,
                            gtsam.noiseModel.Isotropic.Sigma(3, self.cfg.stereo.landmark_prior_sigma),
                        )
                    )
                    graph.add(self._observation_factor(j, landmark_id, stereo_point_j))
                    n_obs += 1

                if stereo_point_i is None:
                    continue
                self.landmark_tracker.get_or_create(idx, idx_b, existing_landmark_id=landmark_id)
                landmark_timestamps[landmark_id] = frame.timestamp_s
                graph.add(self._observation_factor(idx, landmark_id, stereo_point_i))
                n_obs += 1

        return n_obs

    def process_frame(self, idx: int, frames: list[FrameInfo]) -> FrameResult:
        frame = frames[idx]
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()
        mp = self.cfg.motion_prior

        X_i = _pose_key(idx)
        V_i = _vel_key(idx)

        self._get_stereo_observations(idx, frame)

        if idx == 0:
            initial.insert(X_i, gtsam.Pose3())
            graph.add(gtsam.PriorFactorPose3(X_i, gtsam.Pose3(), gtsam.noiseModel.Isotropic.Sigma(6, mp.initial_pose_prior_sigma)))
        else:
            X_prev = _pose_key(idx - 1)
            initial.insert(X_i, self.current_estimate.atPose3(X_prev))
            identity_sigma = np.array([mp.rotation_sigma] * 3 + [mp.translation_sigma] * 3)
            graph.add(gtsam.BetweenFactorPose3(X_prev, X_i, gtsam.Pose3(), gtsam.noiseModel.Diagonal.Sigmas(identity_sigma)))
        # Inert per-frame velocity variable (zero-initialized, standalone prior).
        initial.insert(V_i, np.zeros(6))
        graph.add(gtsam.PriorFactorVector(V_i, np.zeros(6), gtsam.noiseModel.Isotropic.Sigma(6, mp.velocity_prior_sigma)))

        landmark_timestamps: dict[int, float] = {}
        n_obs = self._emit_landmark_observations(idx, frames, graph, initial, landmark_timestamps)

        self._full_graph.push_back(graph)
        self._full_initial.insert(initial)

        timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
        timestamps.insert((X_i, frame.timestamp_s))
        timestamps.insert((V_i, frame.timestamp_s))
        for landmark_id, ts in landmark_timestamps.items():
            timestamps.insert((_landmark_key(landmark_id), ts))

        try:
            self.smoother.update(graph, initial, timestamps)
            self.current_estimate = self.smoother.calculateEstimate()
        except (RuntimeError, IndexError) as e:
            # The incremental smoother occasionally ends up with a broken
            # linear system (IndeterminantLinearSystemException from update(),
            # or an IndexError from calculateEstimate() on an incomplete Bayes
            # tree). update() is not atomic, so build a fresh smoother anchored
            # at this frame's initial pose and continue without landmarks;
            # they are re-seeded on later frames.
            log.warning("Smoother update failed at frame %d (%s: %s) -- resetting backend and retrying without landmarks", idx, type(e).__name__, e)
            self.n_backend_resets += 1
            self._earliest_valid_pose_idx = idx
            # Keep allocating fresh landmark ids so they never collide with
            # landmarks already in the global-BA graph.
            self.landmark_tracker = LandmarkTracker(first_id=self.landmark_tracker.next_id)
            self.smoother = gtsam_unstable.IncrementalFixedLagSmoother(self.cfg.graph.smoother_lag_s, gtsam.ISAM2Params())

            reset_graph = gtsam.NonlinearFactorGraph()
            reset_initial = gtsam.Values()
            reset_initial.insert(X_i, initial.atPose3(X_i))
            reset_initial.insert(V_i, initial.atVector(V_i))
            reset_graph.add(gtsam.PriorFactorPose3(X_i, initial.atPose3(X_i), gtsam.noiseModel.Isotropic.Sigma(6, mp.initial_pose_prior_sigma)))
            reset_graph.add(gtsam.PriorFactorVector(V_i, initial.atVector(V_i), gtsam.noiseModel.Isotropic.Sigma(6, mp.velocity_prior_sigma)))
            reset_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
            reset_timestamps.insert((X_i, frame.timestamp_s))
            reset_timestamps.insert((V_i, frame.timestamp_s))
            self.smoother.update(reset_graph, reset_initial, reset_timestamps)
            self.current_estimate = self.smoother.calculateEstimate()
            n_obs = 0

        self._evict_old_features(idx)

        result = FrameResult(
            frame=frame,
            pose=self.current_estimate.atPose3(X_i),
            velocity=self.current_estimate.atVector(V_i),
            n_landmark_observations=n_obs,
        )
        self.results.append(result)
        return result

    def global_bundle_adjust(self) -> gtsam.Values:
        """Batch Levenberg-Marquardt over every factor added during `run()`.

        Poses and velocities are warm-started from each frame's incremental
        estimate at the time it was processed (earlier frames are no longer in
        the smoother's window); landmarks start from their initial
        triangulation. Does not modify the incremental results.
        """
        warm_start = gtsam.Values(self._full_initial)
        for idx, result in enumerate(self.results):
            warm_start.update(_pose_key(idx), result.pose)
            warm_start.update(_vel_key(idx), result.velocity)
        optimizer = gtsam.LevenbergMarquardtOptimizer(self._full_graph, warm_start, gtsam.LevenbergMarquardtParams())
        return optimizer.optimize()

    def left_keypoints(self, idx: int) -> np.ndarray:
        """(N, 2) left keypoints of a frame still in the lookback window."""
        return self._feature_cache[idx][0].keypoints

    def tracked_keypoints(self, idx: int) -> dict[int, int]:
        """{left keypoint index: landmark id} for a frame still in the lookback window."""
        return self.landmark_tracker.sightings(idx)

    def run(self, frames: list[FrameInfo], on_frame: Callable[[int, FrameResult], None] | None = None) -> list[FrameResult]:
        # Prefetch two frames ahead: one frame's extraction takes about as long
        # as one loop iteration, so a depth of 1 would rarely finish in time.
        prefetch_depth = 2
        if frames:
            self._get_stereo_observations(0, frames[0])
        for k in range(1, min(prefetch_depth, len(frames))):
            self._prefetch_futures[k] = self._prefetch_pool.submit(self._compute_stereo_observations, k, frames[k])

        for idx in range(len(frames)):
            next_idx = idx + prefetch_depth
            if next_idx < len(frames):
                self._prefetch_futures[next_idx] = self._prefetch_pool.submit(
                    self._compute_stereo_observations, next_idx, frames[next_idx]
                )
            result = self.process_frame(idx, frames)
            if on_frame is not None:
                on_frame(idx, result)
            if idx % 50 == 0 or idx == len(frames) - 1:
                log.info(
                    "frame %d/%d ts=%d slot=%s landmark_obs=%d",
                    idx,
                    len(frames) - 1,
                    result.frame.timestamp_ns,
                    result.frame.slot_label,
                    result.n_landmark_observations,
                )
        if self.n_backend_resets:
            log.info("Backend resets: %d", self.n_backend_resets)
        return self.results

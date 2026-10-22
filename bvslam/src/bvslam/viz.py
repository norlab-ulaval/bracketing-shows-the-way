"""Live Rerun visualization of a BVSLAM run.

Read-only: it inspects the builder after each processed frame and never feeds
anything back, so trajectories are identical with or without it.

Shows the camera trajectory, the landmark map colored by the exposure bracket
that first observed each landmark (Low / Mid / High, same palette as the
paper video), and the current left image with its tracked keypoints. After the
global bundle adjustment, the refined trajectory and map replace the live ones.
"""

from __future__ import annotations

import logging
import os
import shutil
import sys
from pathlib import Path

import gtsam
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from bvslam.dataset import FrameInfo
from bvslam.graph_builder import FrameResult, PoseGraphBuilder
from bvslam.imaging import load_gray

log = logging.getLogger(__name__)

# Bracket palette of the paper video (Okabe-Ito blue / green / orange).
SLOT_COLORS = {
    "SAE": (0x56, 0xB4, 0xE9),  # Low
    "MAE": (0x00, 0x9E, 0x73),  # Mid
    "LAE": (0xE6, 0x9F, 0x00),  # High
}
TRAJECTORY_COLOR = (0xFF, 0xFF, 0xFF)
IMAGE_SCALE = 0.5  # the logged camera image is downscaled to keep the recording small

MODES = ("auto", "window", "web", "none")


def _blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial2DView(origin="world/camera/image", name="Left camera"),
                rrb.Spatial2DView(origin="right_camera", name="Right camera"),
            ),
            rrb.Spatial3DView(origin="world", name="Map", background=[0, 0, 0]),
            row_shares=[1, 2],
        ),
        collapse_panels=True,
    )


class RerunViewer:
    def __init__(self, mode: str, builder: PoseGraphBuilder, frames: list[FrameInfo],
                 web_port: int = 9090, grpc_port: int = 9876) -> None:
        if mode in ("auto", "window"):
            # The viewer window is a separate `rerun` program, installed next to
            # this Python interpreter; make sure it can be found even when the
            # environment is not activated.
            bin_dir = str(Path(sys.executable).parent)
            if shutil.which("rerun") is None and shutil.which("rerun", path=bin_dir):
                os.environ["PATH"] = bin_dir + os.pathsep + os.environ.get("PATH", "")
            has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
            if mode == "auto" or shutil.which("rerun") is None:
                if mode == "window":
                    log.warning("The Rerun viewer program was not found; using the web viewer instead")
                mode = "window" if has_display and shutil.which("rerun") else "web"
        self.mode = mode
        self.builder = builder
        self.frames = frames
        self.web_url = None

        self.rec = rr.RecordingStream("bvslam")
        if mode == "window":
            self.rec.spawn(default_blueprint=_blueprint())
        else:
            grpc_url = self.rec.serve_grpc(grpc_port=grpc_port, default_blueprint=_blueprint())
            rr.serve_web_viewer(web_port=web_port, open_browser=False, connect_to=grpc_url)
            self.web_url = f"http://localhost:{web_port}/?url={grpc_url.replace('127.0.0.1', 'localhost')}"
            log.info("Rerun web viewer: %s", self.web_url)

        K = builder.K_stereo
        self.focal = K.fx() * IMAGE_SCALE
        self.principal = (K.px() * IMAGE_SCALE, K.py() * IMAGE_SCALE)
        self.landmarks: dict[int, np.ndarray] = {}
        self.positions: list[np.ndarray] = []

        self._log("world", rr.ViewCoordinates.RDF, static=True)  # camera convention: x right, y down, z forward

    def _log(self, path: str, *entities, static: bool = False) -> None:
        self.rec.log(path, *entities, static=static)

    def _set_time(self, idx: int, frame: FrameInfo) -> None:
        self.rec.set_time("frame", sequence=idx)
        self.rec.set_time("time", duration=frame.timestamp_s)

    def _landmark_color(self, landmark_id: int) -> tuple[int, int, int]:
        first = self.builder.landmark_first_frame.get(landmark_id)
        slot = self.frames[first].slot_label if first is not None else "MAE"
        return SLOT_COLORS.get(slot, SLOT_COLORS["MAE"])

    def _log_landmarks(self, path: str) -> None:
        ids = list(self.landmarks)
        if ids:
            self._log(path, rr.Points3D(np.array([self.landmarks[i] for i in ids]),
                                        colors=np.array([self._landmark_color(i) for i in ids], dtype=np.uint8), radii=rr.Radius.ui_points(2.0)))

    def on_frame(self, idx: int, result: FrameResult) -> None:
        frame = result.frame
        self._set_time(idx, frame)

        pose = result.pose
        self.positions.append(pose.translation())
        self._log("world/camera", rr.Transform3D(translation=pose.translation(), mat3x3=pose.rotation().matrix()))
        self._log("world/trajectory", rr.LineStrips3D([np.array(self.positions)], colors=[TRAJECTORY_COLOR]))

        # Landmarks currently in the smoother window get their latest estimate;
        # marginalized ones keep their last one.
        values = self.builder.current_estimate
        for key in values.keys():
            symbol = gtsam.Symbol(key)
            if symbol.chr() == ord("l"):
                self.landmarks[symbol.index()] = values.atPoint3(key)
        self._log_landmarks("world/landmarks")

        image = load_gray(frame.image_path, self.builder.cfg.dataset.bayer_pattern)
        h, w = image.shape[:2]
        small = image[:: int(1 / IMAGE_SCALE), :: int(1 / IMAGE_SCALE)]
        self._log("world/camera/image", rr.Pinhole(focal_length=self.focal, principal_point=self.principal,
                                                  width=small.shape[1], height=small.shape[0], image_plane_distance=1.0))
        self._log("world/camera/image", rr.Image(small).compress(jpeg_quality=85))
        if frame.right_image_path is not None:
            right = load_gray(frame.right_image_path, self.builder.cfg.dataset.bayer_pattern)
            self._log("right_camera", rr.Image(right[:: int(1 / IMAGE_SCALE), :: int(1 / IMAGE_SCALE)]).compress(jpeg_quality=85))
        kp_ids = self.builder.tracked_keypoints(idx)
        if kp_ids:
            keypoints = self.builder.left_keypoints(idx)
            kps = np.array([keypoints[k] for k in kp_ids]) * IMAGE_SCALE
            self._log("world/camera/image/keypoints",
                      rr.Points2D(kps, colors=np.array([self._landmark_color(lm) for lm in kp_ids.values()], dtype=np.uint8), radii=rr.Radius.ui_points(3.0)))
        else:
            self._log("world/camera/image/keypoints", rr.Clear(recursive=False))

    def on_global_ba(self, ba_values: gtsam.Values, n_poses: int) -> None:
        """Show the refined trajectory and map one step after the last frame."""
        self.rec.set_time("frame", sequence=n_poses)
        self.rec.set_time("time", duration=self.frames[n_poses - 1].timestamp_s + 1e-3)
        for key in ba_values.keys():
            symbol = gtsam.Symbol(key)
            if symbol.chr() == ord("l"):
                self.landmarks[symbol.index()] = ba_values.atPoint3(key)
        positions = np.array([ba_values.atPose3(gtsam.symbol("x", i)).translation() for i in range(n_poses)])
        self._log("world/trajectory", rr.LineStrips3D([positions], colors=[TRAJECTORY_COLOR]))
        self._log_landmarks("world/landmarks")

    def close(self) -> None:
        if self.mode == "web":
            log.info("Done. The viewer stays available at %s -- press Enter to exit.", self.web_url)
            try:
                input()
            except EOFError:
                pass

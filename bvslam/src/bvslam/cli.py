"""Run BVSLAM on a bracketed stereo sequence and write TUM trajectories.

Writes <out> (incremental fixed-lag smoother estimate) and, unless
graph.global_bundle_adjust is false, <out stem>_global_ba.tum (batch bundle
adjustment over every factor; the trajectory evaluated in the paper).
Poses are the rectified left-camera frame in the first frame's coordinates;
timestamps are in seconds.
"""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

import gtsam

from bvslam.config import Config
from bvslam.dataset import drop_low_information_frames, load_stereo_sequence
from bvslam.graph_builder import PoseGraphBuilder
from bvslam.imaging import load_raw
from bvslam.stereo import load_stereo_rig


def write_tum(path: str | Path, timestamps_s: list[float], poses: list[gtsam.Pose3]) -> None:
    with open(path, "w") as f:
        for ts, pose in zip(timestamps_s, poses):
            t = pose.translation()
            q = pose.rotation().toQuaternion()  # w, x, y, z
            f.write(
                f"{ts:.9f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} "
                f"{q.x():.9f} {q.y():.9f} {q.z():.9f} {q.w():.9f}\n"
            )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="bvslam-run", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--data-dir", required=True, help="Sequence directory (images_left/, images_right/, images_meta_left/)"
    )
    parser.add_argument(
        "--calibration-dir",
        default=None,
        help="Directory with stereo_calibration_{left,right}.yaml (default: <data-dir>/calibration)",
    )
    parser.add_argument(
        "--config", default=None, help="YAML config (default: built-in defaults, identical to configs/paper.yaml)"
    )
    parser.add_argument("--out", required=True, help="Output TUM trajectory path")
    parser.add_argument(
        "--global-ba",
        action="store_true",
        help="Force the global bundle adjustment even if graph.global_bundle_adjust is false (it is on by default)",
    )
    parser.add_argument(
        "--viz",
        choices=("auto", "window", "web", "none"),
        default="auto",
        help="Live Rerun visualization: 'window' opens the Rerun viewer, 'web' serves it at http://localhost:<web-port>, "
        "'none' disables it. 'auto' (default) = window if a display is available, else web. "
        "The trajectories do not depend on this option.",
    )
    parser.add_argument("--web-port", type=int, default=9090, help="Port of the Rerun web viewer (--viz web)")
    parser.add_argument(
        "--bayer-pattern",
        choices=("RGGB", "BGGR", "GRBG", "GBRG", "none"),
        default=None,
        help="Override dataset.bayer_pattern: the raw Bayer layout of the images, or 'none' for ordinary "
        "grayscale/color images. By default, color images are detected and read as ordinary images; "
        "single-channel images are demosaiced with the configured pattern (RGGB, as in the paper data)",
    )
    parser.add_argument("--max-frames", type=int, default=None, help="Only process the first N frames")
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    log = logging.getLogger("bvslam")

    cfg = Config.load(args.config) if args.config else Config()
    if args.bayer_pattern is not None:
        cfg.dataset.bayer_pattern = args.bayer_pattern
    log.info("Config: %s", args.config or "built-in defaults (paper)")
    data_dir = Path(args.data_dir)
    calibration_dir = Path(args.calibration_dir) if args.calibration_dir else data_dir / "calibration"

    frames = load_stereo_sequence(data_dir)
    if args.max_frames is not None:
        frames = frames[: args.max_frames]
    log.info("Loaded %d stereo frames from %s", len(frames), data_dir)

    if not frames:
        raise SystemExit("no frames to process")
    if args.bayer_pattern is None and cfg.dataset.bayer_pattern != "none" and load_raw(frames[0].image_path).ndim == 3:
        cfg.dataset.bayer_pattern = "none"
        log.info("Color images detected: reading them as ordinary images (no Bayer demosaicing)")

    pre = cfg.preprocessing
    if pre.drop_low_info_frames:
        n_before = len(frames)
        frames = drop_low_information_frames(
            frames, cfg.dataset.bayer_pattern, pre.drop_low_info_min_brightness, pre.drop_low_info_max_brightness
        )
        log.info(
            "Dropped %d/%d low-information frames (mean brightness outside [%.0f, %.0f])",
            n_before - len(frames), n_before, pre.drop_low_info_min_brightness, pre.drop_low_info_max_brightness,
        )
    if not frames:
        raise SystemExit("no frames to process")

    rig = load_stereo_rig(calibration_dir)
    log.info("Stereo rig loaded: baseline=%.4f m", rig.baseline_m)

    builder = PoseGraphBuilder(cfg, rig)
    viewer = None
    if args.viz != "none":
        # The visualization is optional: never let it stop a run.
        try:
            from bvslam.viz import RerunViewer

            viewer = RerunViewer(args.viz, builder, frames, web_port=args.web_port)
        except Exception as e:
            log.warning("Visualization disabled (%s: %s); continuing without it", type(e).__name__, e)
    t0 = time.time()
    results = builder.run(frames, on_frame=viewer.on_frame if viewer else None)
    elapsed = time.time() - t0
    log.info("Processed %d frames in %.1fs (%.2f fps)", len(results), elapsed, len(results) / max(elapsed, 1e-6))

    timestamps_s = [r.frame.timestamp_s for r in results]
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    write_tum(args.out, timestamps_s, [r.pose for r in results])
    log.info("Wrote trajectory to %s", args.out)

    if args.global_ba or cfg.graph.global_bundle_adjust:
        t0 = time.time()
        ba_values = builder.global_bundle_adjust()
        log.info("Global bundle adjustment: %.1fs", time.time() - t0)
        ba_out = str(Path(args.out).with_suffix("")) + "_global_ba.tum"
        write_tum(ba_out, timestamps_s, [ba_values.atPose3(gtsam.symbol("x", idx)) for idx in range(len(results))])
        log.info("Wrote global-BA trajectory to %s", ba_out)
        if viewer:
            viewer.on_global_ba(ba_values, len(results))

    if viewer:
        viewer.close()


if __name__ == "__main__":
    main()

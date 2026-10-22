# BVSLAM

Stereo visual SLAM on bracketed multi-exposure image sequences, as evaluated in
*Bracketing Shows the Way: Adaptive Multi-Exposure Policy as Input for Robust
Visual SLAM* (ICRA 2027).

## Pipeline

For every stereo frame (raw RGGB Bayer images demosaiced to grayscale, or ordinary images with `--bayer-pattern none`):

1. **Frame filtering**: frames whose mean left-image intensity is outside [1, 245] are dropped.
2. **Features**: SuperPoint keypoints (the 1000 highest scores per image) and
   LightGlue matches (kornia, `superpoint` weights), kept when the match score is ≥ τ = 0.9.
3. **Stereo observations**: left/right matches are undistorted and rectified;
   disparity must exceed 1 px and depth must be below 60 m.
4. **Temporal association**: the current frame is matched against the previous 4
   frames (window W = 5). Direct matching between the shortest (SAE) and
   longest (LAE) exposures is skipped; they are linked through the medium (MAE) frames.
   Matched keypoints extend existing landmarks; a new landmark needs stereo
   depth in both frames.
5. **Factor graph (GTSAM)**: a `GenericStereoFactor3D` per observation (Huber
   k = 1.345, pixel σ = 1 px scaled by max(1, depth / 10 m)), a σ = 3 m prior on each
   new landmark, and a loose identity `BetweenFactorPose3` between consecutive poses.
6. **Optimization**: `IncrementalFixedLagSmoother` (5 s lag) online; if the
   smoother fails it is rebuilt at the current pose. After the last frame, a
   batch Levenberg-Marquardt bundle adjustment over all factors gives the
   trajectory reported in the paper (`*_global_ba.tum`).

The exposure bracket of each frame comes from the `ae_zone` column of the
capture metadata (0 = SAE, the paper's Low bracket; 1 = MAE, Mid; 2 = LAE, High).
Without metadata, all frames are treated as one exposure.

## Install

Requires [uv](https://docs.astral.sh/uv/) and Python 3.11. A CUDA GPU is optional
(the PyPI torch wheels bundle CUDA; without a GPU everything runs on the CPU).

```bash
cd bvslam
uv sync --frozen
uv run bvslam-run --help
```

SuperPoint and LightGlue weights are downloaded by torch.hub on first use.

## Run

```bash
uv run bvslam-run \
  --data-dir <sequence> \
  --calibration-dir <calibration> \
  --out out/trajectory.tum
```

Input layout:

```
<sequence>/images_left/<timestamp_ns>.png         raw Bayer (RGGB) or ordinary images (png, jpg, ...)
<sequence>/images_right/<timestamp_ns>.png
<sequence>/images_meta_left/images_meta_left.csv   optional; columns timestamp, ae_zone
<calibration>/stereo_calibration_{left,right}.yaml ROS camera_info style (camera_matrix,
                                                   distortion_coefficients, rectification_matrix,
                                                   projection_matrix); default: <sequence>/calibration
```

Outputs (TUM format, timestamps in seconds, poses of the rectified left camera
relative to the first frame):

- `trajectory.tum`: incremental fixed-lag smoother estimate.
- `trajectory_global_ba.tum`: after global bundle adjustment (evaluated in the paper).

Options:
- `--config <yaml>`: default is the built-in values, identical to `configs/paper.yaml`.
- `--bayer-pattern {RGGB,BGGR,GRBG,GBRG,none}`: color images are detected automatically; use `none` for plain grayscale images.
- `--viz {auto,window,web,none}`: live Rerun view. The default `auto` opens a window, or the web viewer without a display.
- `--max-frames N`.
- `--global-ba`: forces the global bundle adjustment when a config disables it.
- `-v`.

## Configurations

- `configs/paper.yaml`: the configuration used for the paper. Every value
  equals the default in `src/bvslam/config.py`, so a missing key cannot change behavior.

## Docker

From the repository root:

```bash
docker build -f docker/bvslam.Dockerfile -t bvslam .
docker run --rm --gpus all -v /path/to/data:/data bvslam \
  bvslam-run --data-dir /data/<sequence> --calibration-dir /data/<calibration> --out /data/out/trajectory.tum
```

The image contains the model weights and runs offline; omit `--gpus all` to run on the CPU.

## Tests

```bash
uv run pytest tests
```

`tests/test_smoke.py` renders a small synthetic bracketed stereo sequence
(textured corridor, exposure zones cycling 1, 0, 1, 2), runs the full pipeline
on the CPU and checks the trajectory.

## License

MIT, see `../LICENSE`.

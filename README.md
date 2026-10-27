<div align="center">

# Bracketing Shows the Way: Adaptive Multi-Exposure Policy as Input for Robust Visual SLAM

[![arXiv](https://img.shields.io/badge/arXiv-2610.33174-b31b1b.svg)](https://arxiv.org/abs/2610.33174)
[![Video](https://img.shields.io/badge/Video-YouTube-red.svg)](https://youtu.be/LZGkNU1_s2g)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

<div align="center">
<img src="assets/bvslam_loop.gif" width="80%" alt="B-Adapt + BVSLAM mapping a loop through two tunnel exits; landmarks are colored by the bracket that first observed them"/>
<br><sub>B-Adapt + BVSLAM through two tunnel exits (2× speed).</sub>
</div>

</div>

Rapid illumination transitions, such as a tunnel exit, saturate images before auto-exposure (AE) can adapt, and the features a visual SLAM system needs disappear. We mitigate this at two levels:

- **B-Adapt** (acquisition side) is an online bracketing policy for a stereo camera pair. Two independent loops regulate the darkest and brightest exposures from one-sided intensity-tail objectives, and an intermediate exposure bridges them.
- **BVSLAM** (estimation side) is a factor-graph stereo VSLAM that uses the bracketed sequence **directly**, with no HDR merge. It associates SuperPoint/LightGlue features across neighbouring exposure levels.

<div align="center">
<img src="assets/pipeline.png" width="100%" alt="B-Adapt and BVSLAM pipeline"/>
</div>

---

## Contents

- [Repository layout](#repository-layout)
- [Installation](#installation)
- [Quick start: BVSLAM on the sample data](#quick-start-bvslam-on-the-sample-data)
- [Single-exposure example: FoMo](#single-exposure-example-fomo)
- [Running BVSLAM on your own images](#running-bvslam-on-your-own-images)
- [Acquisition with B-Adapt (ROS 2 driver)](#acquisition-with-b-adapt-ros-2-driver)
- [Parameters](#parameters)
- [License and acknowledgments](#license-and-acknowledgments)
- [Citation](#citation)

This repository contains our two methods, B-Adapt and BVSLAM. The baselines compared in the paper (auto-exposure, fixed bracketing, HDRFlow, ORB-SLAM3, cuVSLAM, AirSLAM) are not included.

## Repository layout

```
driver/        C++17 B-Adapt acquisition library for Basler cameras (stereo pair or single camera, Pylon 12)
driver_ros/    ROS 2 Jazzy node running B-Adapt
bvslam/        BVSLAM: Python factor-graph stereo VSLAM (GTSAM, SuperPoint + LightGlue)
docker/        Dockerfiles for BVSLAM (GPU or CPU) and for the ROS 2 driver
scripts/       Data download and demo
assets/        README figures
```

## Installation

```bash
git clone https://github.com/norlab-ulaval/bracketing-shows-the-way.git
cd bracketing-shows-the-way
```

BVSLAM runs on Linux, with or without a GPU. The driver is only needed to acquire new data ([see below](#acquisition-with-b-adapt-ros-2-driver)).

**Docker (recommended).** For GPU support, install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

```bash
docker compose build bvslam        # or bvslam-cpu without a GPU; the first build takes 5–20 min
```

**Native.** Requires Python 3.11 and [uv](https://docs.astral.sh/uv/).

```bash
cd bvslam && uv sync --frozen && cd ..
```

## Quick start: BVSLAM on the sample data

The sample (1.3 GB, [v1.0 release](https://github.com/norlab-ulaval/bracketing-shows-the-way/releases/tag/v1.0)) is a 15 s tunnel exit recorded with B-Adapt, with its calibration.

```bash
./scripts/download_sample.sh            # -> data/bstw_sample/
./scripts/run_demo.sh                   # Docker
./scripts/run_demo.sh --native          # native
```

A live [Rerun](https://rerun.io) view shows the images, the trajectory and the map. Natively it opens a window; with Docker, open http://localhost:9090/?url=rerun%2Bhttp://localhost:9876/proxy while the demo is running. Set `BVSLAM_VIZ=none` to disable it. The trajectories are written to `output/` in TUM format; `<sequence>_global_ba.tum` is the final one.

## Single-exposure example: FoMo

BVSLAM also runs on single-exposure stereo cameras. This downloads 15 s of a winter trajectory of the [FoMo](https://fomo.norlab.ulaval.ca) dataset (ZED X stereo camera, CC BY 4.0) and converts it:

```bash
python3 scripts/download_fomo.py               # -> data/fomo_red_winter/
./scripts/run_demo.sh data/fomo_red_winter
```

If you use these data, please cite [FoMo](https://arxiv.org/abs/2603.08433).

## Running BVSLAM on your own images

```
<sequence_dir>/
  images_left/<timestamp_ns>.png          any image format; raw Bayer or ordinary images
  images_right/<timestamp_ns>.png
  images_meta_left/images_meta_left.csv   optional: columns `timestamp` and `ae_zone` (0 = Low, 1 = Mid, 2 = High)
  calibration/stereo_calibration_{left,right}.yaml   ROS camera_info format
```

```bash
./scripts/run_demo.sh [--native] /path/to/sequence [/path/to/calibration]
```

Without the metadata file, all frames are treated as one exposure. Single-channel images are treated as raw Bayer RGGB; for plain grayscale images, call `bvslam-run` with `--bayer-pattern none`. See [`bvslam/README.md`](bvslam/README.md) for all options.

## Acquisition with B-Adapt (ROS 2 driver)

The driver runs B-Adapt (n = 3 brackets) on Basler GigE cameras and starts acquiring as soon as it finds them: **stereo with two cameras, mono with one**. A live Rerun view opens by default (`viewer:=none` to disable it). Both installations need the [Pylon 12 SDK](https://www.baslerweb.com/en/downloads/software/), which cannot be redistributed.

**Native (Ubuntu 24.04, ROS 2 Jazzy).**

```bash
pip install "rerun-sdk==0.23.1" "pyarrow<22"      # for the live view
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/norlab-ulaval/bracketing-shows-the-way.git
cd ~/ws && colcon build --packages-select basler_camera && source install/setup.bash
ros2 launch basler_camera basler_camera.launch.py
```

**Docker.** 

Put the Pylon installer in `docker/` (or pack an existing installation: `tar -C /opt/pylon -czf docker/pylon-local.tar.gz .`), then:

```bash
docker compose build driver
docker compose run --rm driver
```

While the driver is running, open the live view at http://localhost:9090/?url=rerun%2Bhttp://localhost:9876/proxy.

**Network.** 

The cameras must be on the same subnet as their Ethernet port. The default 8192-byte packets need jumbo frames (`sudo ip link set <iface> mtu 9000`); otherwise use `packet_size:=1500`.

**ROS launch parameters.**

Native (example):
```bash
ros2 launch basler_camera basler_camera.launch.py packet_size:=1500
```
Docker (example):
```bash
docker compose run --rm driver ros2 launch basler_camera basler_camera.launch.py packet_size:=1500
```

See [`driver_ros/README.md`](driver_ros/README.md) for the launch options, wiring, topics and parameters. To use your recordings with BVSLAM, convert the bag with [rosbag_extractor](https://github.com/norlab-ulaval/rosbag_extractor/tree/c40a158ae05129df398fc3d7ee5d1f21ac666123) (config [`yoda.yaml`](https://github.com/norlab-ulaval/rosbag_extractor/blob/c40a158ae05129df398fc3d7ee5d1f21ac666123/configs/yoda.yaml)).

## Parameters

Paper values (Table I), set in [`b_adapt.yaml`](driver_ros/config/b_adapt.yaml) and [`paper.yaml`](bvslam/configs/paper.yaml).

**B-Adapt** (ROS 2 parameters)

| Paper symbol | Meaning | Parameter | Value |
|---|---|---|---|
| $n$ | number of brackets | `ae_zone_count` | `3` |
| $\alpha$ | tail fraction treated as outliers | `ae_bridge_low_percentile` / `ae_bridge_high_percentile` | `99.0` / `1.0` (1 %) |
| $\beta$ | intensity margin from the sensor limits | `ae_bridge_low_target` / `ae_bridge_high_target` | `250` / `5` |
| $\lambda$ | EMA coefficient of the Low and High loops | `ae_zone_alpha` (entries 0 and 2) | `0.8` |
| $\Delta_{\min}$ | minimum Low/High separation | `ae_bridge_min_stops_gap` | `1.0` stop |
| $t$ | exposure-time range | `min_exposure_us` / `max_exposure_us` | 19 µs / 5 ms |
| $g$ | gain range | `min_gain_db` / `max_gain_db` | 0 / 48 dB |
| $f$ | frame rate | `acquisition_frame_rate` | 20 Hz nominal, 4 Hz tunnel |

**BVSLAM** ([`paper.yaml`](bvslam/configs/paper.yaml))

| Paper symbol | Meaning | Key | Value |
|---|---|---|---|
| $W$ | association window: current frame plus a lookback | `graph.vo_lookback` | `4` (so $W$ = 5) |
| — | LightGlue match-confidence threshold | `lightglue.min_confidence` | `0.9` |
| — | SuperPoint keypoints per image | `superpoint.max_keypoints` | `1000` |
| — | sliding-window (fixed-lag smoother) length | `graph.smoother_lag_s` | `5.0` s |

## License and acknowledgments

The code in this repository is released under the [MIT License](LICENSE). BVSLAM downloads the pretrained SuperPoint weights, which are distributed by Magic Leap under a **non-commercial** license, and LightGlue weights (Apache-2.0). Check those terms before any commercial use.

We build on [GTSAM](https://github.com/borglab/gtsam), [LightGlue](https://github.com/cvg/LightGlue), [kornia](https://github.com/kornia/kornia) and [Rerun](https://rerun.io).

## Citation

If you use this code or data, please cite our paper:

```bibtex
@misc{gamache2026bracketing,
  title         = {{Bracketing Shows the Way: Adaptive Multi-Exposure Policy as Input for Robust Visual {SLAM}}},
  author        = {Gamache, Olivier and Lavertu, Anthony and Duclos, David-Alexandre and Fortin, Jean-Michel and Pomerleau, Fran{\c{c}}ois and Gigu{\`e}re, Philippe},
  year          = {2026},
  eprint        = {2610.33174},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2610.33174}
}
```

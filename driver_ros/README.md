# basler_camera (ROS 2 Jazzy)

ROS 2 driver for a hardware-synchronized Basler GigE stereo pair (or a
single camera, see [mono mode](#mono-mode)) with a software multi-exposure
(bracketing) controller, used to record the data of
*Bracketing Shows the Way: Adaptive Multi-Exposure Policy as Input for Robust
Visual SLAM*. The camera-control library is in [`../driver`](../driver); its
control law is described in [`../driver/DESIGN.md`](../driver/DESIGN.md).

## Exposure policy: B-Adapt

The driver implements B-Adapt: three exposures per cycle. Low (zone 0) and
high (zone 2) are each controlled on a tail percentile of their own frame;
mid (zone 1) is derived every frame as the geometric mean of low's and
high's total light. `config/b_adapt.yaml` holds the values used for the
recordings.

Mapping to the paper's symbols (Table I):

| Symbol | Value | Parameter(s) |
|---|---|---|
| n | 3 | `ae_zone_count` |
| α | 1 % | `ae_bridge_low_percentile: 99.0`, `ae_bridge_high_percentile: 1.0` |
| β | 5 | `ae_bridge_low_target: 250.0` (= 255 − β), `ae_bridge_high_target: 5.0` |
| λ (EMA rate) | 0.8 | `ae_zone_alpha: [0.8, 0.4, 0.8]` (low, mid, high; mid is derived, its entry is unused) |
| Δmin | 1 stop | `ae_bridge_min_stops_gap: 1.0` |
| Exposure range | 19–5000 µs | `min_exposure_us`, `max_exposure_us` |
| Gain range | 0–48 dB | `min_gain_db`, `max_gain_db` |
| Frame rate | 20 Hz (4 Hz in the tunnel) | `acquisition_frame_rate` |

Other settings: `epsilon: 0.02`, `ae_saturation_boost_fraction: 0.5`,
`zone_sort_stride: 1`. The node's built-in parameter defaults are generic
and differ from the rig values, so always start from `config/b_adapt.yaml`
(the launch file's default). Set `verbose: true` for a per-frame AE log on
stdout.

## Published topics

For `X` in `camera_1` (left) and `camera_2` (right; not advertised in mono
mode), relative to the node namespace, reliable QoS, depth 4:

| Topic | Type | Content |
|---|---|---|
| `basler/X/image_raw` | `sensor_msgs/Image` | Every frame, `bayer_rggb8`, 1920x1200 |
| `basler/X/zone_<i>/image_raw` | `sensor_msgs/Image` | Only frames of zone `i` (0 = darkest bracket) |
| `basler/X/image_meta` | `basler_camera/BaslerFrameMeta` | Per-frame exposure, gain, zone, brightness, flags |

The capture order is ping-pong: zones `0,1,2,1,…` for three zones
(`sequence_index` 0..3), so zone 1 (mid) carries half of the frames. `frame_id` is `camera_1` / `camera_2`. Image and metadata share
the same stamp: **host time taken right after both cameras' grabs returned**
(after exposure, readout and GigE transfer), identical for the left and right
frame of a pair. See `msg/BaslerFrameMeta.msg` for the meaning of each field.

## Hardware

* 2x Basler a2A1920-51gcPRO (GigE, BayerRG8). camera_1 = left, camera_2 = right.
* Trigger cable: camera_1's `Line3` (output, source `ExposureActive`) to
  camera_2's `Line3` (input, `RisingEdge`). camera_1 is software-triggered
  once per frame and hardware-triggers camera_2, so both expose together.
* One GigE NIC per camera, on the same subnet as the camera. The config uses
  8192-byte packets, which need MTU 9000 (`sudo ip link set <iface> mtu 9000`).
  On a standard NIC (MTU 1500), launch with `packet_size:=1500`; otherwise every
  grab fails with "The buffer was incompletely grabbed".
* Basler pylon SDK 12.x.

## Build

Clone the whole repository into a colcon workspace (`driver/` is found as
`../driver` and is ignored by colcon via `COLCON_IGNORE`):

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone <repo-url> bracketing-shows-the-way
cd ~/ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select basler_camera
```

pylon is found through `pylon-config` in `$PYLON_ROOT/bin` or `/opt/pylon/bin`.
A different library location can be given with `--cmake-args -DBASLER_DRIVER_DIR=<path>`.

### Docker

`../docker/ros.Dockerfile` builds the package on `ros:jazzy-ros-base`. The
pylon SDK may not be redistributed: download pylon 12.x for Ubuntu 24.04
(x86_64 or arm64; `.deb`, Debian package archive or setup tarball) from
baslerweb.com and put it, unrenamed (the name must start with `pylon`), in
`docker/`. Then build from the repository root:

```bash
docker build -f docker/ros.Dockerfile -t bracketing-driver .
docker run --rm -it --network host bracketing-driver   # auto-detects the cameras
docker run --rm -it --network host bracketing-driver \
    ros2 launch basler_camera basler_camera.launch.py \
    camera_1_serial:=<left> camera_2_serial:=<right>
```

`--network host` is required so that the container sees the host NICs the
cameras are connected to (GigE Vision discovery uses broadcast). Configure
the NICs (IP, MTU 9000, or `packet_size:=1500`) on the host.

## Launch

```bash
ros2 launch basler_camera basler_camera.launch.py          # auto-detect (1 = mono, 2 = stereo)
ros2 launch basler_camera basler_camera.launch.py \
    camera_1_serial:=<left serial> camera_2_serial:=<right serial>
ros2 launch basler_camera basler_camera.launch.py camera_1_serial:=<serial>   # mono
ros2 launch basler_camera basler_camera.launch.py packet_size:=1500   # standard NIC, no jumbo frames
ros2 launch basler_camera basler_camera.launch.py viewer:=rerun       # live Rerun view
ros2 launch basler_camera basler_camera.launch.py config_file:=/path/to/my.yaml
```

**Camera auto-detection.** `camera_1_serial` and `camera_2_serial` are empty
by default (in `config/b_adapt.yaml` and the launch file). The node then
enumerates the Basler cameras it can see at startup:

* exactly 1 camera: mono mode with it as camera_1 (see below).
* exactly 2 cameras: they are sorted by serial number, the **lowest serial
  becomes camera_1 (left, trigger master)** and the other camera_2 (right).
  The assignment is logged. If left and right are swapped on your rig, set
  both serials explicitly (the log line prints the arguments to swap them).
* 0 or more than 2 cameras: the node exits with an error listing the
  cameras found (serial, model, IP). Check power, cabling and the NIC
  configuration, or set the serial(s) explicitly.

Explicit serials (launch arguments or YAML) bypass auto-detection: both set
= stereo, only `camera_1_serial` = mono, only `camera_2_serial` is an error. Other arguments: `config_file` (default
`config/b_adapt.yaml`), `node_name` (default `basler_camera`) and `respawn`
(default `true`).

### Mono mode

With a single camera (auto-detected, or only `camera_1_serial` set) the node
runs B-Adapt on camera_1 alone: same software trigger, exposure/gain
sequence, controller and `image_meta` as camera_1 in stereo, with no trigger
cable and no second camera. Only the `basler/camera_1/...` topics are
advertised. Unlike stereo, which publishes complete pairs only, every
camera_1 frame is published. Mono is for B-Adapt acquisition only: BVSLAM
needs the stereo pair.

On a fatal camera error (camera not found, link lost) the node logs the
error and exits with code 1. With `respawn:=true` the launch file restarts
it after 2 s; the controller then restarts from a new AE-Once seed.


## Live view

`viewer:=rerun` (or `ros2 run basler_camera rerun_view.py` next to a running
driver) shows the live feed in Rerun, in one row: left image, right image (empty
in mono mode), and the exposure value of every frame (log2 of exposure time
x gain), one line per bracket (Low blue, Mid green, High orange). It only subscribes to the driver's
topics. It opens a window when a display is available, otherwise a web viewer at
http://localhost:9090 (parameter `mode:=window|web`). Natively it needs
`pip install "rerun-sdk==0.23.1" "pyarrow<22"`; the Docker image includes it.

## Known limitations

* **Zone attribution.** Each frame is attributed to the zone whose *current*
  exposure estimate is closest to the frame's chunk-data exposure
  (`ZoneManager::closestZone()`, ties within `exposure_tie_tolerance_us`
  resolved to the requested zone). Because the controllers are updated on a
  background thread while the next frame is captured, a frame can be
  labeled, and fed back, as a different zone than the one requested during
  fast exposure transients. `ae_zone` in the metadata is that attributed zone.
* B-Adapt supports exactly three zones (`ae_zone_count: 3`).
* Low/high targets are rounded to integer percent of 255: the effective
  targets are 249.9 and 5.1 for the configured 250 and 5.
* `gain_db` is the programmed gain, not read back from the camera.
* `brightness` is always camera_1's measurement, also in camera_2's metadata.
* `bracket_index`, `requested_exposure_us` and `is_dark_frame` in
  `BaslerFrameMeta` are unused (always zero) and kept only so that recorded
  bags remain readable.
* Timestamps are host time after the grabs, not exposure start.

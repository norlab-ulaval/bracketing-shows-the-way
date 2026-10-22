#!/usr/bin/env python3
"""Live Rerun view of the B-Adapt driver: left image, right image, and the
exposure value of every frame, colored by bracket.

Exposure value (stops) = log2(exposure_us * 10^(gain_db / 20)): exposure time and
gain combined, as B-Adapt controls them.

Read-only: it only subscribes to the driver's topics.

    ros2 run basler_camera rerun_view.py                 # window if a display is available, else web viewer
    ros2 run basler_camera rerun_view.py --ros-args -p mode:=web

Requires the Rerun SDK: pip install "rerun-sdk==0.23.1" "pyarrow<22"
"""

import math
import os

import sys

import numpy as np
import rclpy

try:
    import rerun as rr
    import rerun.blueprint as rrb
except ImportError:
    print('[rerun_view] Rerun is not installed: pip install "rerun-sdk==0.23.1" "pyarrow<22". '
          "The driver keeps running; launch with viewer:=none to hide this message.", file=sys.stderr)
    sys.exit(0)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from basler_camera.msg import BaslerFrameMeta

# Bracket palette shared with BVSLAM and the paper video: Low, Mid, High.
BRACKETS = {0: ("Low", (0x56, 0xB4, 0xE9)), 1: ("Mid", (0x00, 0x9E, 0x73)), 2: ("High", (0xE6, 0x9F, 0x00))}


def debayer_rggb_half(raw: np.ndarray) -> np.ndarray:
    """Half-resolution RGB from an RGGB mosaic (one RGB pixel per 2x2 block)."""
    r = raw[0::2, 0::2]
    g = ((raw[0::2, 1::2].astype(np.uint16) + raw[1::2, 0::2]) // 2).astype(np.uint8)
    b = raw[1::2, 1::2]
    return np.dstack([r, g, b])


def blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="/camera_1", name="Left camera"),
            rrb.Spatial2DView(origin="/camera_2", name="Right camera"),
            rrb.TimeSeriesView(origin="/exposure_value", name="Exposure value (stops)"),
        ),
        collapse_panels=True,
    )


class RerunView(Node):
    def __init__(self) -> None:
        super().__init__("rerun_view")
        mode = self.declare_parameter("mode", "auto").value
        web_port = self.declare_parameter("web_port", 9090).value
        if mode == "auto":
            mode = "window" if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") else "web"

        rr.init("bracketing_driver")
        if mode == "window":
            rr.spawn(default_blueprint=blueprint())
        else:
            url = rr.serve_grpc(default_blueprint=blueprint())
            rr.serve_web_viewer(web_port=web_port, open_browser=False, connect_to=url)
            self.get_logger().info(f"Rerun web viewer: http://localhost:{web_port}/?url={url.replace('127.0.0.1', 'localhost').replace('+', '%2B')}")

        for name, color in BRACKETS.values():
            rr.log(f"exposure_value/{name}", rr.SeriesLines(colors=[color], names=[name], widths=[2.0]), static=True)

        for cam in ("camera_1", "camera_2"):
            self.create_subscription(Image, f"basler/{cam}/image_raw", lambda m, c=cam: self.on_image(c, m), qos_profile_sensor_data)
        self.create_subscription(BaslerFrameMeta, "basler/camera_1/image_meta", self.on_meta, 50)

    def set_time(self, stamp) -> None:
        rr.set_time("ros_time", timestamp=stamp.sec + stamp.nanosec * 1e-9)

    def on_image(self, cam: str, msg: Image) -> None:
        raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width]
        image = debayer_rggb_half(raw) if msg.encoding.startswith("bayer") else raw
        self.set_time(msg.header.stamp)
        rr.log(cam, rr.Image(image).compress(jpeg_quality=80))

    def on_meta(self, msg: BaslerFrameMeta) -> None:
        name = BRACKETS.get(msg.ae_zone, ("Mid", None))[0]
        self.set_time(msg.header.stamp)
        rr.log(f"exposure_value/{name}", rr.Scalars(math.log2(msg.exposure_us * 10 ** (msg.gain_db / 20))))


def main() -> None:
    rclpy.init()
    rclpy.spin(RerunView())


if __name__ == "__main__":
    main()

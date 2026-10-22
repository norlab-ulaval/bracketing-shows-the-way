"""Launch the Basler stereo bracketing node (B-Adapt).

    ros2 launch basler_camera basler_camera.launch.py
    ros2 launch basler_camera basler_camera.launch.py \
        camera_1_serial:=<left serial> camera_2_serial:=<right serial>
    ros2 launch basler_camera basler_camera.launch.py packet_size:=1500   # no jumbo frames
    ros2 launch basler_camera basler_camera.launch.py viewer:=none        # without the live view

Without serials the node auto-detects the cameras: one = mono, two = stereo
(lowest serial = camera_1, left). Only camera_1_serial set = mono.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value="",
            description="Parameter YAML (default: config/b_adapt.yaml)"),
        DeclareLaunchArgument(
            "camera_1_serial", default_value="",
            description="Left camera serial (overrides the YAML when non-empty; "
                        "empty in both = auto-detect; alone = mono)"),
        DeclareLaunchArgument(
            "camera_2_serial", default_value="",
            description="Right camera serial (overrides the YAML when non-empty; "
                        "empty in both = auto-detect)"),
        DeclareLaunchArgument(
            "packet_size", default_value="",
            description="GigE packet size in bytes for both cameras (overrides the YAML when non-empty). "
                        "The YAML uses 8192, which needs jumbo frames (MTU 9000) on the camera network "
                        "interface; use 1500 on a standard interface"),
        DeclareLaunchArgument(
            "viewer", default_value="rerun",
            description="'rerun' (default) also starts a live Rerun view (left image, right image, exposure "
                        "value per bracket): a window if a display is available, else a web viewer on port 9090; 'none' disables it"),
        DeclareLaunchArgument(
            "node_name", default_value="basler_camera",
            description="Node name"),
        DeclareLaunchArgument(
            "respawn", default_value="true",
            description="Restart the node after a fatal camera error (e.g. GigE link loss)"),
        OpaqueFunction(function=_launch_node),
    ])


def _launch_node(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    if not config_file:
        config_file = os.path.join(
            get_package_share_directory("basler_camera"), "config", "b_adapt.yaml")

    params = [config_file]
    overrides = {}
    for key in ("camera_1_serial", "camera_2_serial"):
        value = LaunchConfiguration(key).perform(context)
        if value:
            overrides[key] = value
    packet_size = LaunchConfiguration("packet_size").perform(context)
    if packet_size:
        overrides["camera_1_packet_size_bytes"] = int(packet_size)
        overrides["camera_2_packet_size_bytes"] = int(packet_size)
    if overrides:
        params.append(overrides)

    respawn = LaunchConfiguration("respawn").perform(context).lower() in ("true", "1", "yes")
    nodes = []
    if LaunchConfiguration("viewer").perform(context) == "rerun":
        nodes.append(Node(package="basler_camera", executable="rerun_view.py", name="rerun_view", output="screen"))
    return nodes + [Node(
        package="basler_camera",
        executable="basler_camera_node",
        name=LaunchConfiguration("node_name").perform(context),
        parameters=params,
        output="screen",
        emulate_tty=True,
        respawn=respawn,
        respawn_delay=2.0,
    )]

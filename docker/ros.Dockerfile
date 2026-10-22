# syntax=docker/dockerfile:1
# ROS 2 Jazzy image for the basler_camera driver (driver/ + driver_ros/).
#
# The Basler pylon SDK cannot be redistributed (Basler EULA). Download pylon
# 12.x for Linux (Ubuntu 24.04, x86_64 or aarch64: the Debian package, the
# Debian package archive, or the setup tarball) from
# https://www.baslerweb.com/en/downloads/software/ and put it in docker/
# without renaming it (the file name must start with "pylon"). Then build
# from the repository root:
#
#   docker build -f docker/ros.Dockerfile -t bracketing-driver .
#
# Run with host networking so the container sees the cameras' GigE NICs:
#
#   docker run --rm -it --network host bracketing-driver
#
# which auto-detects the cameras (two = stereo, lowest serial = left; one =
# mono). To choose them:
#
#   docker run --rm -it --network host bracketing-driver \
#       ros2 launch basler_camera basler_camera.launch.py \
#       camera_1_serial:=<left> camera_2_serial:=<right>

FROM ros:jazzy-ros-base AS deps
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws

# ── pylon SDK from the user-supplied package (docker/pylon*) ────────────
FROM deps AS pylon
# The package is bind-mounted for this step only, so it is not stored in the image.
RUN --mount=type=bind,source=docker,target=/tmp/docker_dir \
    set -eux; \
    mkdir /tmp/pylon_pkg; cp -a /tmp/docker_dir/pylon* /tmp/pylon_pkg/; \
    cd /tmp/pylon_pkg; \
    for f in *.tar.gz *.tgz; do if [ -f "$f" ]; then tar -xzf "$f"; rm "$f"; fi; done; \
    debs="$(find . -name 'pylon*.deb')"; \
    if [ -n "$debs" ]; then \
        apt-get update; \
        apt-get install -y --no-install-recommends $debs; \
        rm -rf /var/lib/apt/lists/*; \
    else \
        inner="$(find . -name 'pylon*.tar.gz' | head -n1)"; \
        mkdir -p /opt/pylon; \
        if [ -n "$inner" ]; then tar -C /opt/pylon -xzf "$inner"; else cp -a . /opt/pylon/; fi; \
    fi; \
    rm -rf /tmp/pylon_pkg; \
    test -x /opt/pylon/bin/pylon-config || { echo "no pylon-config found in docker/pylon*"; exit 1; }

# ── build ─────────────────────────────────────────────────────────────────
FROM pylon AS final
COPY driver     /ws/src/bracketing-shows-the-way/driver
COPY driver_ros /ws/src/bracketing-shows-the-way/driver_ros
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select basler_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN sed -i 's|^exec "\$@"|source /ws/install/setup.bash\nexec "$@"|' /ros_entrypoint.sh
CMD ["ros2", "launch", "basler_camera", "basler_camera.launch.py"]

# Basler bracketing driver (C++ library)

Camera-control library behind the `basler_camera` ROS 2 package: a
hardware-triggered Basler a2A1920-51gcPRO GigE stereo pair (or a single
camera, mono mode) with a software multi-exposure controller (the paper's
B-Adapt policy).

* Usage, parameters, build and launch: [`../driver_ros/README.md`](../driver_ros/README.md).
* Control law, threading and trigger design: [`DESIGN.md`](DESIGN.md).

The library is built by the ROS package through `add_subdirectory()`. It
needs the Basler pylon SDK (12.x), found through `pylon-config` in
`$PYLON_ROOT/bin` or `/opt/pylon/bin`. Standalone compile check:

```bash
cmake -S driver -B build/driver && cmake --build build/driver -j
```

`COLCON_IGNORE` keeps colcon from treating this directory as a package.

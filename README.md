# realsense_gazebo_plugin

> Custom Gazebo Harmonic sensor layer for Intel RealSense cameras, targeting **ROS 2 Jazzy** and the **gz-sim8 / gz-sensors8 / gz-rendering8** stack.

---

## Overview

`realsense_gazebo_plugin` provides three custom rendering sensors and the system plugin required to drive them inside Gazebo Sim:

| Sensor type | `gz:type` value | Purpose |
|---|---|---|
| `custom::Ros2Camera` | `Ros2Camera` | Baseline RGB camera (test / reference) |
| `custom::Ros2DepthCamera` | `Ros2DepthCamera` | Depth image + optional pointcloud |
| `custom::Ros2RgbdCamera` | `Ros2RgbdCamera` | RGB + depth + optional pointcloud |

> [!NOTE]
> These sensors are an extension of the official ones.

### Why custom sensors are necessary

The stock `gz-sensors8` depth and RGBD implementations measure range along the **x-axis** of the sensor frame. ROS 2 follows the optical-frame convention defined in [REP-103](https://www.ros.org/reps/rep-0103.html): **X right, Y down, Z forward** along the optical axis. When the canonical Gazebo sensor is published under the correct ROS optical frame, the pointcloud ends up projected along the wrong axis.

The custom sensors correct this without touching the render pose:

- ROS-friendly optical-frame handling, decoupled from the Gazebo render pose.
- RGBD pointcloud rotated into the ROS `_optical` frame convention before publication.
- Per-sensor `publish_pointcloud` flag to suppress the `/points` topic entirely when it is not needed.

Because Gazebo Sim does not manage custom rendering sensors end-to-end, this package also ships `Ros2CameraSystem` — a dedicated system plugin that discovers sensor entities in the Entity Component System (ECS), instantiates the custom sensor objects, and feeds the authoritative world pose into the rendering layer on every update.

---

## Supported stack

| Component | Version |
|---|---|
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| gz-sim | 8 |
| gz-sensors | 8 |
| gz-rendering | 8 |

> [!WARNING]
> This package targets the Harmonic stack exclusively and is not designed for Gazebo Classic or earlier Gazebo Garden/Fortress releases.

---

## Repository structure

```text
realsense_gazebo_plugin/
├── CMakeLists.txt
├── package.xml
├── env-hooks/
│   └── realsense_gazebo_plugin.dsv.in       # Env-hook: prepends PATHs of libraries
├── include/
│   ├── sensors/
│   │   ├── Ros2Camera.hh
│   │   ├── Ros2DepthCamera.hh
│   │   └── Ros2RgbdCamera.hh
│   └── utils/
│       ├── CameraSensorUtil.hh          # Vendored from gz-sensors8
│       └── PointCloudUtil.hh            # Vendored from gz-sensors8
├── src/
│   ├── plugin.cpp                       # Plugin registration and type aliases
│   ├── Ros2CameraSystem.cpp             # ECS bridge: entity discovery and pose sync
│   ├── sensors/
│   │   ├── Ros2Camera.cpp
│   │   ├── Ros2DepthCamera.cpp          # Pointcloud gating and frame correction
│   │   └── Ros2RgbdCamera.cpp           # RGBD publication and pointcloud rotation
│   └── utils/
│       ├── CameraSensorUtil.cc          # Vendored from gz-sensors8
│       └── PointCloudUtil.cc            # Vendored from gz-sensors8
└── README.md
```

**Key files at a glance:**

- `src/Ros2CameraSystem.cpp` — ECS entity discovery and pose synchronization.
- `src/sensors/Ros2DepthCamera.cpp` — Pointcloud gating and optical-frame correction.
- `src/sensors/Ros2RgbdCamera.cpp` — RGBD publication and pointcloud rotation.
- `src/plugin.cpp` — Plugin registration and `custom::`-qualified type aliases.
- `src/utils/` — Vendored copies of upstream `gz-sensors8` utilities. Preserve their license headers; document any local modifications.

---

## Design notes

### Why sensor sources are copied locally

`gz-sensors8` keeps critical sensor state and rendering callbacks inside private implementation classes (PIMPL). Inheritance is therefore an impractical integration path when the changes needed live inside the sensor internals. Specifically:

- Pointcloud publication must be gated by `publish_pointcloud`.
- The RGBD pointcloud must be rotated before publication to match the ROS optical-frame convention.
- The load path must accept the custom SDF sensor type and custom SDF parameters.

The relevant sensor sources were copied into `src/sensors/` and adapted locally. The goal is not to diverge from upstream behavior unnecessarily; local changes are kept to the minimum required for this integration.

### Why a system plugin is required

Gazebo Sim creates the Entity Component System (ECS) entity for every sensor declared in SDF, but it skips entities of `type="custom"` when creating the built-in sensor objects. A custom system plugin is therefore required to bridge the gap and is implemented under the name `Ros2CameraSystem`.

The upstream reference implementation for this pattern is [`gz::sensors::DopplerVelocityLog`](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1DopplerVelocityLog.html) together with its companion system plugin, which follows the same structure. See also the canonical tutorial at [gazebosim.org — Custom sensors](https://gazebosim.org/api/sensors/8/custom_sensors.html) and the example in `gz-sim/examples/plugins/custom_sensor_system`.

---

## Build

```bash
colcon build --packages-select realsense_gazebo_plugin
source install/setup.bash
```

> [!NOTE]
> The package installs environment hooks that prepend the shared-library path to both `GZ_SIM_SYSTEM_PLUGIN_PATH` and `GZ_PLUGIN_PATH`. Sourcing the workspace is sufficient for normal use.

---

## Usage

### 1. Load the system plugin

Add the following to your world or robot SDF:

```xml
<plugin filename="libRos2CameraSystem.so" name="custom::Ros2CameraSystem"/>
```

This plugin is responsible for discovering custom camera entities and driving them at runtime.

### 2. Declare a custom sensor

The `gz:type` attribute selects the sensor implementation. The `custom::` prefix is also accepted.

```xml
<sensor name="camera_rgbd" type="custom" gz:type="Ros2RgbdCamera">
  ...
</sensor>
```

Valid `gz:type` values: `Ros2Camera`, `Ros2DepthCamera`, `Ros2RgbdCamera`.

> [!TIP]
> All the parameters documented in the [SDF documentation](https://sdformat.org/spec/1.11/sensor/) are still used in the sensor. Also `gz_frame_id`.

### Custom depth camera

```xml
<sensor name="depth" type="custom" gz:type="Ros2DepthCamera">
  <topic>/camera/depth</topic>
  <publish_pointcloud>true</publish_pointcloud>
  <camera>
    <camera_info_topic>/camera/depth/camera_info</camera_info_topic>
    <optical_frame_id>camera_depth_optical_frame</optical_frame_id>
  </camera>
</sensor>
```

### Custom RGBD camera

```xml
<sensor name="rgbd" type="custom" gz:type="Ros2RgbdCamera">
  <topic>/camera/rgbd</topic>
  <publish_pointcloud>false</publish_pointcloud>
  <camera>
    <camera_info_topic>/camera/rgbd/camera_info</camera_info_topic>
    <optical_frame_id>camera_color_optical_frame</optical_frame_id>
  </camera>
</sensor>
```

When `publish_pointcloud` is `false`, the image and depth topics remain active; only the `/points` topic is suppressed.<br>

### `Ros2Camera`

`Ros2Camera` is a baseline sensor intended for testing and as a reference implementation. It adds no project-specific behavior beyond the custom loading path and is not part of the production pipeline.

---

## Optional pointcloud publication

Both `Ros2DepthCamera` and `Ros2RgbdCamera` adds a `publish_pointcloud` flag at the root of the sensor SDF element:

```xml
<publish_pointcloud>false</publish_pointcloud>
```

| Value | Behavior |
|---|---|
| `true` | `/points` topic is advertised and pointcloud messages are published. |
| `false` | `/points` topic is never created; pointcloud publication is completely disabled. |

---

## Runtime lifecycle

The following sequence describes the per-frame update path once the sensor is live:

1. The sensor entity is created in the ECS by Gazebo Sim.
2. `Ros2CameraSystem::PreUpdate` detects the new `CustomSensor` entity and instantiates the sensor object from its SDF description.
3. `Ros2CameraSystem::PostUpdate` reads the authoritative world pose from the ECS.
4. `Ros2CameraSystem::OnPreRender` pushes that pose into the rendering sensor.
5. The sensor updates its rendering buffers and publishes on the configured ROS topics.

This pose handoff ensures correct model placement in the simulation while allowing published ROS messages to use the optical-frame conventions required by downstream consumers such as `image_pipeline` and `depth_image_proc`.

---

## Vendored utilities

The following files are copied verbatim from the `gz-sensors8` upstream source:

| File | Origin |
|---|---|
| `src/utils/CameraSensorUtil.cc` | `gz-sensors` upstream |
| `src/utils/PointCloudUtil.cc` | `gz-sensors` upstream |

Treat these as vendored code. **Do not remove their license headers.** Document any local modifications inline with a `// LOCAL:` comment and a brief explanation.

---

## Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| Gazebo cannot find the plugin | `GZ_SIM_SYSTEM_PLUGIN_PATH` not set | Re-run `source install/setup.bash` in the current shell. |
| Sensor appears at the wrong position | ECS pose handoff failed | Inspect `Ros2CameraSystem::PostUpdate` and verify the entity pose is being read correctly. |
| `/points` topic published when it should not be | `publish_pointcloud` is missing or set to `true` | Confirm the `<publish_pointcloud>false</publish_pointcloud>` element is present under the sensor root. |
| RGBD pointcloud projection looks incorrect | Wrong optical frame or intrinsics | Verify camera intrinsics and the `optical_frame_id` value before modifying the sensor math. |

---

## Reference

Upstream API references relevant to this package:

- [`gz::sensors::RenderingSensor`](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1RenderingSensor.html)
- [`gz::sensors::RgbdCameraSensor`](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1RgbdCameraSensor.html)
- [`gz::sensors::DopplerVelocityLog`](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1DopplerVelocityLog.html) — reference implementation for the custom sensor + system pattern
- [`gz::sim::components::CustomSensor`](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1components_1_1CustomSensor.html)
- [`gz::sim::systems::Sensors`](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Sensors.html)
- [Gazebo Sensors — Custom sensors tutorial](https://gazebosim.org/api/sensors/8/custom_sensors.html)
- [REP-103 — Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html) — defines the ROS optical-frame convention (X right, Y down, Z forward)

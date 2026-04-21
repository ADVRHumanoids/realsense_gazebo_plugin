# realsense_gz_plugin — build & test guide

## ⚠️ Fix `package.xml` before building

The tool that generated these files strips `<name>` tags (it interprets them
as HTML).  Both `package.xml` files currently contain a broken `<n>` tag on
line 4.  Open each one and fix it:

```xml
<!-- wrong (what you received) -->
<n>realsense_gz_plugin</n>

<!-- correct -->
<name>realsense_gz_plugin</name>
```

Do the same for `realsense_gz_plugin_test/package.xml`  
(`<name>realsense_gz_plugin_test</name>`).

---

## Prerequisites

| Dependency | Notes |
|---|---|
| ROS 2 Jazzy | tested target |
| Gazebo Harmonic | gz-sim8, gz-sensors8, gz-rendering8 |
| `ros_gz_bridge` | for the test launch file |
| `tf2_ros` | for static TF publishers in the test |
| `rviz2` | optional, for visual verification |

---

## Repository layout

```
ws/src/
├── realsense_gz_plugin/          ← the plugin itself
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/realsense_gz_plugin/
│   │   ├── ros2_sensor_config.hh
│   │   ├── ros2_camera_sensor.hh
│   │   ├── ros2_depth_camera_sensor.hh
│   │   └── ros2_custom_sensor_system.hh
│   └── src/
│       ├── ros2_sensor_config.cpp
│       ├── ros2_camera_sensor.cpp
│       ├── ros2_depth_camera_sensor.cpp
│       ├── ros2_custom_sensor_system.cpp
│       └── realsense_gz_plugin.cpp
│
└── realsense_gz_plugin_test/     ← standalone test package
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── bridge_topics.yaml    ← ros_gz_bridge topic mapping
    ├── launch/
    │   └── test_sensors.launch.py
    ├── rviz/
    │   └── test_sensors.rviz
    └── worlds/
        └── test_sensors.sdf      ← two test cases (see below)
```

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select realsense_gz_plugin realsense_gz_plugin_test \
             --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

---

## Run the test world

```bash
# Let Gazebo find the plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$(ros2 pkg prefix realsense_gz_plugin)/lib

# Launch everything (Gazebo + bridge + TF + rviz2)
ros2 launch realsense_gz_plugin_test test_sensors.launch.py

# Headless / CI
ros2 launch realsense_gz_plugin_test test_sensors.launch.py \
  gz_gui:=false rviz:=false
```

---

## What the world contains

### Test case 1 — `test_cam_model`

| Sensor | SDF type | `<ros2>` block |
|---|---|---|
| `test_cam_color` | `camera` | `publish_frame_id: test_cam_color_optical_frame` |
| `test_cam_depth` | `depth_camera` | `publish_frame_id: test_cam_depth_optical_frame` + `enable_point_cloud: true` |

**What to check:**

```bash
# Image header frame must be the optical frame, NOT the render frame
ros2 topic echo /test_cam/color/image_raw \
  --field header.frame_id --once
# expected: test_cam_color_optical_frame

ros2 topic echo /test_cam/depth/image_rect_raw \
  --field header.frame_id --once
# expected: test_cam_depth_optical_frame

# Pointcloud must be present because enable_point_cloud=true
ros2 topic hz /test_cam/depth/color/points
```

### Test case 2 — `no_override_cam_model` (baseline / control)

| Sensor | SDF type | `<ros2>` block |
|---|---|---|
| `no_override_cam_color` | `camera` | **none** |

**What to check:**

```bash
# Without a <ros2> block the frame should be whatever Gazebo defaults to
# (typically the gz_frame_id value or the sensor entity name)
ros2 topic echo /no_override_cam/color/image_raw \
  --field header.frame_id --once
```

---

## Topic reference

| Topic | Type | Condition |
|---|---|---|
| `/test_cam/color/image_raw` | `sensor_msgs/Image` | always |
| `/test_cam/color/camera_info` | `sensor_msgs/CameraInfo` | always |
| `/test_cam/depth/image_rect_raw` | `sensor_msgs/Image` | always |
| `/test_cam/depth/camera_info` | `sensor_msgs/CameraInfo` | always |
| `/test_cam/depth/color/points` | `sensor_msgs/PointCloud2` | `enable_point_cloud: true` |
| `/no_override_cam/color/image_raw` | `sensor_msgs/Image` | always |
| `/no_override_cam/color/camera_info` | `sensor_msgs/CameraInfo` | always |
| `/clock` | `rosgraph_msgs/Clock` | always |

---

## SDF sensor format expected by the plugin

```xml
<sensor name="..." type="camera">           <!-- or depth_camera -->
  <gz_frame_id>render_frame</gz_frame_id>   <!-- Gazebo rendering frame -->
  <topic>camera_name/color/image_raw</topic>

  <camera name="...">
    <camera_info_topic>camera_name/color/camera_info</camera_info_topic>
    <optical_frame_id>color_optical_frame</optical_frame_id>
    <!-- intrinsics, FOV, image size, clip … -->
  </camera>

  <!-- optional: override the header frame written into published messages -->
  <ros2>
    <publish_frame_id>color_optical_frame</publish_frame_id>
    <!-- depth sensors only: -->
    <enable_point_cloud>true</enable_point_cloud>
  </ros2>
</sensor>
```

The `<ros2>` block is optional.  Without it the plugin behaves identically to
the standard Gazebo sensor.

---

## How the frame override works

`Ros2CustomSensorSystem::CreateSensors` intercepts **every** `camera` and
`depth_camera` sensor entity (not just `type="custom"` ones).  For each it
calls `Ros2CameraSensor::Load` / `Ros2DepthCameraSensor::Load` which:

1. Reads `<ros2><publish_frame_id>` via `LoadRos2SensorConfig`.
2. Clones the SDF element and patches the `type` attribute to `"camera"` /
   `"depth_camera"` so the Gazebo base class accepts it.
3. Calls the base class `Load`.
4. Calls `SetFrameId(publishFrameId)` — this overrides the frame that the
   base class writes into outgoing message headers.

The render frame (`gz_frame_id`) is untouched, so Gazebo places the camera
correctly in the scene while the published messages carry the ROS optical
frame.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `[Ros2CustomSensorSystem] failed creating sensor` | Base class `Load` rejected the SDF | Check that `<camera>` child and `<horizontal_fov>` are present |
| No topics visible | Plugin `.so` not found | Check `GZ_SIM_SYSTEM_PLUGIN_PATH` |
| Frame is still the render frame | `<publish_frame_id>` not parsed | Verify the `<ros2>` block is a direct child of `<sensor>` |
| No pointcloud | `enable_point_cloud` missing or false | Add `<enable_point_cloud>true</enable_point_cloud>` inside `<ros2>` |
| `gz-sensors8` depth_camera component not found | `gz-sensors8` built without depth support | Rebuild `gz-sensors` with `-DBUILD_TESTING=OFF` and all sensor components enabled |

# Intel RealSense Gazebo ROS plugin

This package is a Gazebo ROS plugin for the Intel D435 realsense camera.

A Gazebo Harmonic (gz-sim8) plugin that simulates Intel RealSense D435 cameras with full ROS2 Humble integration.

## Features

- **Multi-sensor simulation**: RGB, depth, and dual infrared cameras
- **ROS2 integration**: Native image_transport publishers with compression support
- **Real-time publishing**: Timer-based data streaming at 30Hz
- **Thread-safe execution**: Dedicated ROS2 spinner thread for reliable operation
- **Test pattern generation**: Built-in synthetic test images for validation
- **Automatic ROS2 initialization**: Plugin handles ROS2 setup if not already initialized

## Current Status

This package has been **extensively modified and enhanced** for Gazebo Harmonic compatibility:

- ✅ **Working ROS2 data publishing** - All camera topics actively publishing at 30Hz
- ✅ **Thread-safe operation** - Dedicated ROS2 spinner thread ensures reliable callbacks
- ✅ **Automatic initialization** - Handles ROS2 setup and context management
- ✅ **Comprehensive test patterns** - Generates RGB gradients, depth maps, and IR patterns
- ✅ **Full Gazebo Harmonic support** - Updated for gz-sim8 architecture

## Available ROS2 Topics

### Primary Image Topics

| Topic | Message Type | Description | Format |
|-------|-------------|-------------|---------|
| `/realsense/color/image_raw` | sensor_msgs/Image | RGB color image | 640x480, rgb8 |
| `/realsense/depth/image_raw` | sensor_msgs/Image | Depth image | 640x480, 32FC1 |
| `/realsense/infrared1/image_raw` | sensor_msgs/Image | First infrared image | 640x480, mono8 |
| `/realsense/infrared2/image_raw` | sensor_msgs/Image | Second infrared image | 640x480, mono8 |

### Compressed Topics (automatically available)

Each primary topic includes compressed variants:

- `*/compressed` - JPEG compression
- `*/compressedDepth` - PNG compression (for depth)
- `*/theora` - Theora video compression

## Acknowledgement

This is a continuation of work done by [SyrianSpock](https://github.com/SyrianSpock) for a Gazebo ROS plugin with RS200 camera.

This package also includes the work developed by Intel Corporation with the ROS model for the [D435](https://github.com/intel-ros/realsense) camera.

## Example usage with a custom robot

Note that this was tested for the ROS2 branch with ROS Foxy distro. A turtlebot3 like custom robot model was used.
In custom robot's `model.sdf`, we should attach the link, sensors, joint  and plugin block as following;

```xml
    <link name="realsense_link">
      <pose>0.4 0 0.25 0 0 0</pose>
      <visual name="realsense_link_visual">
        <pose>0 0 0 -1.57 0 -1.57</pose>
        <geometry>
          <mesh>
            <uri>model://chiconybot/meshes/d435.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="realsense_link_collision">
        <pose>0 0 0 -1.57 0 -1.57</pose>
        <geometry>
          <box>
            <size>0.02505 0.090 0.025</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.001</iyy>
          <iyz>0.000</iyz>
          <izz>0.001</izz>
        </inertia>
        <mass>0.564</mass>
      </inertial>

      <sensor name="cameradepth" type="depth">
        <camera name="camera">
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.100</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>0</visualize>
      </sensor>
      <sensor name="cameracolor" type="camera">
        <camera name="camera">
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>RGB_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>1</visualize>
      </sensor>
      <sensor name="cameraired1" type="camera">
        <camera name="camera">
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>L_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>1</update_rate>
        <visualize>0</visualize>
      </sensor>
      <sensor name="cameraired2" type="camera">
        <camera name="camera">
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>L_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>1</update_rate>
        <visualize>0</visualize>
      </sensor>
    </link>

    <joint name="realsense_joint" type="fixed">
      <parent>base_link</parent>
      <child>realsense_link</child>
      <pose>0.4 0 0.4 0 0 0</pose>
    </joint>

    <plugin name="camera" filename="librealsense_gazebo_plugin.so">
      <prefix>camera</prefix>
      <depthUpdateRate>30.0</depthUpdateRate>
      <colorUpdateRate>30.0</colorUpdateRate>
      <infraredUpdateRate>1.0</infraredUpdateRate>
      <depthTopicName>aligned_depth_to_color/image_raw</depthTopicName>
      <depthCameraInfoTopicName>depth/camera_info</depthCameraInfoTopicName>
      <colorTopicName>color/image_raw</colorTopicName>
      <colorCameraInfoTopicName>color/camera_info</colorCameraInfoTopicName>
      <infrared1TopicName>infra1/image_raw</infrared1TopicName>
      <infrared1CameraInfoTopicName>infra1/camera_info</infrared1CameraInfoTopicName>
      <infrared2TopicName>infra2/image_raw</infrared2TopicName>
      <infrared2CameraInfoTopicName>infra2/camera_info</infrared2CameraInfoTopicName>
      <colorOpticalframeName>camera_color_optical_frame</colorOpticalframeName>
      <depthOpticalframeName>camera_depth_optical_frame</depthOpticalframeName>
      <infrared1OpticalframeName>camera_left_ir_optical_frame</infrared1OpticalframeName>
      <infrared2OpticalframeName>camera_right_ir_optical_frame</infrared2OpticalframeName>
      <rangeMinDepth>0.3</rangeMinDepth>
      <rangeMaxDepth>3.0</rangeMaxDepth>
      <pointCloud>true</pointCloud>
      <pointCloudTopicName>depth/color/points</pointCloudTopicName>
      <pointCloudCutoff>0.3</pointCloudCutoff>
    </plugin>
```

Finally we should define the joint, links of each camera(color, depth, ir_right, ir_left) W.R.T robot body,
In URDF(usually in `xxx_description` package) of the robot add following;

```xml
  <link name="camera_bottom_screw_frame">
    <visual>
      <geometry>
        <mesh filename="package://chiconybot_description/meshes/sensors/d435.dae" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://chiconybot_description/meshes/sensors/d435.dae" />
      </geometry>
    </collision>
  </link>

  <link name="camera_link"></link>

  <link name="camera_depth_frame"></link>

  <link name="camera_depth_optical_frame"></link>

  <link name="camera_color_frame"></link>

  <link name="camera_color_optical_frame"></link>

  <link name="camera_left_ir_frame"></link>

  <link name="camera_left_ir_optical_frame"></link>

  <link name="camera_right_ir_frame"></link>

  <link name="camera_right_ir_optical_frame"></link>


   <joint name="camera_joint" type="fixed">
    <parent link="base_link" />
    <child link="camera_bottom_screw_frame" />
    <pose xyz="0.4 0 0.25" rpy="0 0 0" />
  </joint>

  <joint name="camera_link_joint" type="fixed">
    <parent link="camera_bottom_screw_frame" />
    <child link="camera_link" />
    <pose xyz="0 0.0175 0.0125 " rpy="0 0 0" />
  </joint>

  <joint name="camera_depth_joint" type="fixed">
    <parent link="camera_link" />
    <child link="camera_depth_frame" />
    <pose xyz="0 0 0" rpy="0 0 0" />
  </joint>

  <joint name="camera_depth_optical_joint" type="fixed">
    <parent link="camera_depth_frame" />
    <child link="camera_depth_optical_frame" />
    <pose xyz="0 0 0 " rpy="-1.57 0 -1.57" />
  </joint>

  <joint name="camera_color_joint" type="fixed">
    <parent link="camera_depth_frame" />
    <child link="camera_color_frame" />
    <pose xyz="0 0 0" rpy="0 0 0" />
  </joint>

  <joint name="camera_color_optical_joint" type="fixed">
    <parent link="camera_color_frame" />
    <child link="camera_color_optical_frame" />
    <pose xyz="0 0 0 " rpy="-1.57 0 -1.57" />
  </joint>

  <joint name="camera_left_ir_joint" type="fixed">
    <parent link="camera_depth_frame" />
    <child link="camera_left_ir_frame" />
    <pose xyz="0 0 0 " rpy="0 0 0 " />
  </joint>

  <joint name="camera_left_ir_optical_joint" type="fixed">
    <parent link="camera_left_ir_frame" />
    <child link="camera_left_ir_optical_frame" />
    <pose xyz="0 0 0 " rpy="-1.57 0 -1.57" />
  </joint>

  <joint name="camera_right_ir_joint" type="fixed">
    <parent link="camera_depth_frame" />
    <child link="camera_right_ir_frame" />
    <pose xyz="0 -0.050 0 " rpy="0 0 0" />
  </joint>

  <joint name="camera_right_ir_optical_joint" type="fixed">
    <parent link="camera_right_ir_frame" />
    <child link="camera_right_ir_optical_frame" />
    <pose xyz="0 0 0 " rpy="-1.57 0 -1.57" />
  </joint>

```

## Testing

### Build the Plugin

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build the plugin
colcon build --packages-select realsense_gazebo_plugin

# Source the workspace
source install/setup.bash

# Set plugin path
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/install/realsense_gazebo_plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

### Run Simulation

**Terminal 1: Start Simulation**

```bash
# Run simulation (server-only mode)
gz sim -s src/realsense_gazebo_plugin/test.sdf
```

**Terminal 2: Verify Data**

```bash
# Check available topics
ros2 topic list | grep realsense

# Test image data reception
ros2 topic echo /realsense/color/image_raw --once

# Monitor publishing rate
ros2 topic hz /realsense/color/image_raw
```

### Expected Output

**Successful startup:**

```
ROS2 not initialized, initializing now...
[INFO] [timestamp] [gazebo_realsense_realsense]: Realsense Gazebo ROS plugin loading.
[INFO] [timestamp] [gazebo_realsense_realsense]: Loaded Realsense Gazebo ROS plugin.
RealSensePlugin: The realsense_camera plugin is attached to model realsense_camera
```

**Topic data sample:**

```yaml
header:
  stamp:
    sec: 1755227640
    nanosec: 12165814
  frame_id: realsense_color_optical_frame
height: 480
width: 640
encoding: rgb8
is_bigendian: 0
step: 1920
data: [255, 128, 64, ...]
```

## Configuration

The plugin supports extensive configuration through SDF parameters:

```xml
<plugin filename="librealsense_gazebo_plugin.so"
        name="gz::realsense_gazebo_plugin::GazeboRosRealsense">
  <prefix>realsense</prefix>
  <depthUpdateRate>30.0</depthUpdateRate>
  <colorUpdateRate>30.0</colorUpdateRate>
  <infraredUpdateRate>30.0</infraredUpdateRate>
  <depthTopicName>depth/image_raw</depthTopicName>
  <colorTopicName>color/image_raw</colorTopicName>
  <infrared1TopicName>infrared1/image_raw</infrared1TopicName>
  <infrared2TopicName>infrared2/image_raw</infrared2TopicName>
  <colorOpticalframeName>realsense_color_optical_frame</colorOpticalframeName>
  <depthOpticalframeName>realsense_depth_optical_frame</depthOpticalframeName>
  <infrared1OpticalframeName>realsense_ired1_optical_frame</infrared1OpticalframeName>
  <infrared2OpticalframeName>realsense_ired2_optical_frame</infrared2OpticalframeName>
  <rangeMinDepth>0.2</rangeMinDepth>
  <rangeMaxDepth>10.0</rangeMaxDepth>
  <pointCloud>false</pointCloud>
</plugin>
```

## Test Data

The plugin generates synthetic test patterns:

- **Color Image**: RGB gradient pattern (red: horizontal gradient, green: vertical gradient, blue: constant 128)
- **Depth Image**: Linear depth gradient from 1.0m to 10.0m
- **Infrared Images**: Incrementing/decrementing test patterns

## Troubleshooting

### No Data Published

**Wait 8-10 seconds** after simulation start for full initialization.

```bash
# Restart ROS2 daemon
ros2 daemon stop && ros2 daemon start

# Check simulation is running
ps aux | grep gz

# Verify plugin path
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
```

### Plugin Not Found

```bash
# Rebuild and reset environment
colcon build --packages-select realsense_gazebo_plugin
source install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/install/realsense_gazebo_plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

### Multiple Simulations

```bash
# Kill all Gazebo processes
pkill -f "gz sim"
sleep 2
# Restart simulation
```

## Implementation Details

- **Architecture**: Timer-based publishing with dedicated ROS2 spinner thread
- **Publishing rate**: Consistent 30Hz for all camera topics
- **Thread safety**: Full thread-safe operation with proper cleanup
- **Memory management**: Smart pointers and automatic resource cleanup
- **ROS2 integration**: Native image_transport for efficient image streaming

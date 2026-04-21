#include <gz/plugin/Register.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sim/System.hh>

#include "realsense_gz_plugin/ros2_camera_sensor.hh"
#include "realsense_gz_plugin/ros2_custom_sensor_system.hh"
#include "realsense_gz_plugin/ros2_depth_camera_sensor.hh"

// ---------------------------------------------------------------------------
// Ros2CameraSensor — registered as a gz-sensors sensor plugin so that the
// sensor factory can create it by name when type="custom" gz:type="ros2_camera"
// is used.  In practice Ros2CustomSensorSystem creates instances directly, but
// the registration is required for the plugin loader.
// ---------------------------------------------------------------------------
GZ_ADD_PLUGIN(
    realsense_gz_plugin::Ros2CameraSensor,
    gz::sensors::Sensor)

GZ_ADD_PLUGIN_ALIAS(
    realsense_gz_plugin::Ros2CameraSensor,
    "realsense_gz_plugin::Ros2CameraSensor")

// ---------------------------------------------------------------------------
// Ros2DepthCameraSensor
// ---------------------------------------------------------------------------
GZ_ADD_PLUGIN(
    realsense_gz_plugin::Ros2DepthCameraSensor,
    gz::sensors::Sensor)

GZ_ADD_PLUGIN_ALIAS(
    realsense_gz_plugin::Ros2DepthCameraSensor,
    "realsense_gz_plugin::Ros2DepthCameraSensor")

// ---------------------------------------------------------------------------
// Ros2CustomSensorSystem — the gz-sim world system that intercepts camera /
// depth_camera sensor entities and creates the custom sensor objects.
// ---------------------------------------------------------------------------
GZ_ADD_PLUGIN(
    realsense_gz_plugin::Ros2CustomSensorSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    realsense_gz_plugin::Ros2CustomSensorSystem,
    "realsense_gz_plugin::Ros2CustomSensorSystem")

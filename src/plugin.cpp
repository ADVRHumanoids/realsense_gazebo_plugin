#include <gz/plugin/Register.hh>

#include "sensors/Ros2Camera.hh"
#include "sensors/Ros2DepthCamera.hh"
#include "sensors/Ros2RgbdCamera.hh"

GZ_ADD_PLUGIN(custom::Ros2Camera, gz::sensors::Sensor)
GZ_ADD_PLUGIN_ALIAS(custom::Ros2Camera, "Ros2Camera")
GZ_ADD_PLUGIN_ALIAS(custom::Ros2Camera, "custom::Ros2Camera")

GZ_ADD_PLUGIN(custom::Ros2DepthCamera, gz::sensors::Sensor)
GZ_ADD_PLUGIN_ALIAS(custom::Ros2DepthCamera, "Ros2DepthCamera")
GZ_ADD_PLUGIN_ALIAS(custom::Ros2DepthCamera, "custom::Ros2DepthCamera")

GZ_ADD_PLUGIN(custom::Ros2RgbdCamera, gz::sensors::Sensor)
GZ_ADD_PLUGIN_ALIAS(custom::Ros2RgbdCamera, "Ros2RgbdCamera")
GZ_ADD_PLUGIN_ALIAS(custom::Ros2RgbdCamera, "custom::Ros2RgbdCamera")

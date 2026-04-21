#pragma once

#include <string>
#include <sdf/Sensor.hh>
#include <sdf/Element.hh>

namespace realsense_gz_plugin
{

/// \brief Configuration parsed from the <ros2> SDF block of a sensor.
struct Ros2SensorConfig
{
  /// Frame id written into outgoing message headers.
  /// If empty the base class default is preserved.
  std::string publishFrameId;

  /// Enable pointcloud publication on depth sensors.
  /// Defaults to false so the pointcloud is only published when explicitly
  /// requested via <enable_point_cloud>true</enable_point_cloud>.
  bool enablePointCloud{false};
};

/// \brief Parse a Ros2SensorConfig from a sdf::Sensor wrapper.
/// Returns true even if no <ros2> block is present (config stays default).
bool LoadRos2SensorConfig(
    const sdf::Sensor &_sensor,
    Ros2SensorConfig &_config);

/// \brief Parse a Ros2SensorConfig from a raw sdf::ElementPtr.
/// Returns true even if no <ros2> block is present.
bool LoadRos2SensorConfig(
    sdf::ElementPtr _sensorElem,
    Ros2SensorConfig &_config);

}  // namespace realsense_gz_plugin

#include "realsense_gz_plugin/ros2_sensor_config.hh"

#include <string>

#include <sdf/Param.hh>

namespace realsense_gz_plugin
{

namespace
{

constexpr const char *kRos2Element = "ros2";
constexpr const char *kPublishFrameIdElement = "publish_frame_id";
constexpr const char *kEnablePointCloudElement = "enable_point_cloud";

sdf::ElementPtr Ros2Element(sdf::ElementPtr _sensorElem)
{
  if (!_sensorElem || !_sensorElem->HasElement(kRos2Element))
  {
    return nullptr;
  }

  return _sensorElem->GetElement(kRos2Element);
}

}  // namespace

bool LoadRos2SensorConfig(
    const sdf::Sensor &_sensor,
    Ros2SensorConfig &_config)
{
  return LoadRos2SensorConfig(_sensor.Element(), _config);
}

bool LoadRos2SensorConfig(
    sdf::ElementPtr _sensorElem,
    Ros2SensorConfig &_config)
{
  const auto ros2Elem = Ros2Element(_sensorElem);
  if (!ros2Elem)
  {
    // No <ros2> block is fine — just use defaults.
    return true;
  }

  if (ros2Elem->HasElement(kPublishFrameIdElement))
  {
    _config.publishFrameId =
        ros2Elem->Get<std::string>(kPublishFrameIdElement);
  }

  if (ros2Elem->HasElement(kEnablePointCloudElement))
  {
    _config.enablePointCloud =
        ros2Elem->Get<bool>(kEnablePointCloudElement);
  }

  return true;
}

}  // namespace realsense_gz_plugin

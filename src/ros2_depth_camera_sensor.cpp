#include "realsense_gz_plugin/ros2_depth_camera_sensor.hh"

namespace realsense_gz_plugin
{

namespace
{

bool ToDepthCameraSensor(sdf::ElementPtr _sensorElem, sdf::Sensor &_sensor)
{
  if (!_sensorElem)
  {
    return false;
  }

  auto clone = _sensorElem->Clone();
  if (!clone)
  {
    return false;
  }

  // Patch the type attribute so DepthCameraSensor::Load accepts the element.
  auto typeAttr = clone->GetAttribute("type");
  if (!typeAttr || !typeAttr->SetFromString("depth_camera"))
  {
    return false;
  }

  const auto errors = _sensor.Load(clone);
  return errors.empty();
}

}  // namespace

bool Ros2DepthCameraSensor::Load(const sdf::Sensor &_sdf)
{
  if (!LoadRos2SensorConfig(_sdf, this->config))
  {
    return false;
  }

  sdf::Sensor depthSensor;
  if (!ToDepthCameraSensor(_sdf.Element(), depthSensor))
  {
    return false;
  }

  if (!gz::sensors::DepthCameraSensor::Load(depthSensor))
  {
    return false;
  }

  this->ApplyPublishFrameConfig();
  return true;
}

bool Ros2DepthCameraSensor::Load(sdf::ElementPtr _sdf)
{
  if (!LoadRos2SensorConfig(_sdf, this->config))
  {
    return false;
  }

  if (!_sdf)
  {
    return false;
  }

  sdf::Sensor sensor;
  if (!ToDepthCameraSensor(_sdf, sensor))
  {
    return false;
  }

  if (!this->Load(sensor))
  {
    return false;
  }

  return true;
}

bool Ros2DepthCameraSensor::HasPointConnections() const
{
  if (!this->config.enablePointCloud)
  {
    return false;
  }

  return gz::sensors::DepthCameraSensor::HasPointConnections();
}

void Ros2DepthCameraSensor::ApplyPublishFrameConfig()
{
  if (!this->config.publishFrameId.empty())
  {
    this->SetFrameId(this->config.publishFrameId);
  }
}

}  // namespace realsense_gz_plugin

#pragma once

#include <sdf/Sensor.hh>
#include <gz/sensors/DepthCameraSensor.hh>

#include "realsense_gz_plugin/ros2_sensor_config.hh"

namespace realsense_gz_plugin
{

/// \brief A DepthCameraSensor derivative that overrides the outgoing message
/// header frame_id and gates pointcloud publication.
///
/// Rendering and depth image production are fully delegated to the Gazebo
/// base class.  The behavioural deltas are:
///
///   - The <ros2><publish_frame_id> value (if present) is written into the
///     sensor frame via SetFrameId() after the base class has loaded.
///   - HasPointConnections() returns false unless <enable_point_cloud>true
///     is set, preventing the pointcloud from being produced even when the
///     underlying rendering sensor would normally emit it.
class Ros2DepthCameraSensor : public gz::sensors::DepthCameraSensor
{
public:
  /// \brief Load from a fully-typed sdf::Sensor.
  /// The SDF element type is patched to "depth_camera" before forwarding
  /// to the base class.
  bool Load(const sdf::Sensor &_sdf) override;

  /// \brief Load from a raw SDF element (called by the sensor factory).
  bool Load(sdf::ElementPtr _sdf) override;

  /// \brief Gate pointcloud production.
  /// Returns false (suppressing pointcloud emission) unless
  /// config.enablePointCloud is true.
  bool HasPointConnections() const override;

private:
  /// \brief Apply publish_frame_id from config to the base class.
  void ApplyPublishFrameConfig();

  Ros2SensorConfig config;
};

}  // namespace realsense_gz_plugin

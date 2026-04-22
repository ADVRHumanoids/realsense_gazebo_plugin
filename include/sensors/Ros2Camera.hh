#ifndef ROS2_CAMERA_HH
#define ROS2_CAMERA_HH

#include <sdf/Sensor.hh>
#include <gz/sensors/CameraSensor.hh>

namespace custom
{

/// \brief A CameraSensor derivative that reloads a custom / copied sensor SDF
/// as a plain Gazebo camera sensor before forwarding to the base class.
class Ros2Camera : public gz::sensors::CameraSensor
{
public:
  /// \brief Load from a fully-typed sdf::Sensor.
  /// If the incoming sensor is declared as a custom sensor in SDF, its
  /// backing element is reloaded as a plain camera sensor before forwarding
  /// to the Gazebo base class.
  bool Load(const sdf::Sensor &_sdf) override;

  /// \brief Load from a raw SDF element (called by the sensor factory).
  bool Load(sdf::ElementPtr _sdf) override;
};

}  // namespace custom

#endif  // ROS2_CAMERA_HH

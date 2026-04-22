#include "sensors/Ros2Camera.hh"

#include <gz/common/Console.hh>
namespace custom
{

namespace
{

bool ToCameraSensor(sdf::ElementPtr _sensorElem, sdf::Sensor &_sensor)
{
  if (!_sensorElem)
  {
    gzerr << "ToCameraSensor(ElementPtr): null sensor element" << std::endl;
    return false;
  }

  auto clone = _sensorElem->Clone();
  if (!clone)
  {
    gzerr << "ToCameraSensor(ElementPtr): failed cloning sensor element"
          << std::endl;
    return false;
  }

  gzdbg << "ToCameraSensor(ElementPtr): original name=["
        << clone->Get<std::string>("name", "").first << "] type=["
        << clone->Get<std::string>("type", "").first << "]" << std::endl;

  // The SDF element arrives with type="custom" (or whatever the world SDF
  // declares).  CameraSensor::Load rejects any type other than "camera",
  // so we patch the attribute on the clone before forwarding.
  auto typeAttr = clone->GetAttribute("type");
  if (!typeAttr || !typeAttr->SetFromString("camera"))
  {
    gzerr << "ToCameraSensor(ElementPtr): failed setting type attribute to"
          << " camera" << std::endl;
    return false;
  }

  const auto errors = _sensor.Load(clone);
  if (!errors.empty())
  {
    gzerr << "ToCameraSensor(ElementPtr): sdf::Sensor::Load returned "
          << errors.size() << " errors" << std::endl;
  }
  return errors.empty();
}

bool ToCameraSensor(const sdf::Sensor &_input, sdf::Sensor &_cameraSensor)
{
  gzdbg << "ToCameraSensor(Sensor): name=[" << _input.Name() << "] type=["
        << _input.TypeStr() << "] hasCamera=["
        << (_input.CameraSensor() != nullptr) << "] hasElement=["
        << (_input.Element() != nullptr) << "]" << std::endl;

  if (_input.Type() == sdf::SensorType::CAMERA && _input.CameraSensor())
  {
    _cameraSensor = _input;
    return true;
  }

  return ToCameraSensor(_input.Element(), _cameraSensor);
}

}  // namespace

bool Ros2Camera::Load(const sdf::Sensor &_sdf)
{
  gzdbg << "Ros2Camera::Load(sdf::Sensor) name=[" << _sdf.Name()
        << "] type=[" << _sdf.TypeStr() << "]" << std::endl;

  sdf::Sensor cameraSensor;
  if (!ToCameraSensor(_sdf, cameraSensor))
  {
    gzerr << "Ros2Camera failed converting custom sensor [" << _sdf.Name()
          << "] into camera SDF" << std::endl;
    return false;
  }

  if (!gz::sensors::CameraSensor::Load(cameraSensor))
  {
    gzerr << "Ros2Camera base CameraSensor::Load failed for ["
          << cameraSensor.Name() << "]" << std::endl;
    return false;
  }

  gzdbg << "Ros2Camera successfully loaded [" << cameraSensor.Name()
        << "] topic=[" << this->Topic() << "] frame_id=["
        << this->FrameId() << "]" << std::endl;
  return true;
}

bool Ros2Camera::Load(sdf::ElementPtr _sdf)
{
  if (!_sdf)
  {
    gzerr << "Ros2Camera::Load(ElementPtr) received null SDF" << std::endl;
    return false;
  }

  gzdbg << "Ros2Camera::Load(ElementPtr) name=["
        << _sdf->Get<std::string>("name", "").first << "] type=["
        << _sdf->Get<std::string>("type", "").first << "]" << std::endl;

  sdf::Sensor sensor;
  if (!ToCameraSensor(_sdf, sensor))
  {
    gzerr << "Ros2Camera failed converting raw SDF element into camera sensor"
          << std::endl;
    return false;
  }

  gzdbg << "Ros2Camera::Load(ElementPtr) converted sensor name=["
        << sensor.Name() << "] type=[" << sensor.TypeStr() << "]"
        << std::endl;

  // Delegate to the sdf::Sensor overload which applies the frame config.
  if (!this->Load(sensor))
  {
    return false;
  }

  return true;
}

} // namespace custom

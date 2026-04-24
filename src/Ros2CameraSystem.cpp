/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/rendering/Events.hh>

#include "Ros2CameraSystem.hh"
#include "sensors/Ros2Camera.hh"
#include "sensors/Ros2DepthCamera.hh"

using namespace custom;

namespace
{

constexpr const char *kRos2CameraType = "Ros2Camera";
constexpr const char *kRos2CameraTypeQualified = "custom::Ros2Camera";
constexpr const char *kRos2DepthCameraType = "Ros2DepthCamera";
constexpr const char *kRos2DepthCameraTypeQualified = "custom::Ros2DepthCamera";

bool IsRos2CameraType(const sdf::Sensor &_sensor)
{
  const auto type = gz::sensors::customType(_sensor);
  gzdbg << "Ros2CameraSystem IsRos2CameraType: name=[" << _sensor.Name()
        << "] gz:type=[" << type << "]" << std::endl;
  return type == kRos2CameraType || type == kRos2CameraTypeQualified;
}

bool IsRos2DepthCameraType(const sdf::Sensor &_sensor)
{
  const auto type = gz::sensors::customType(_sensor);
  gzdbg << "Ros2CameraSystem IsRos2DepthCameraType: name=[" << _sensor.Name()
        << "] gz:type=[" << type << "]" << std::endl;
  return type == kRos2DepthCameraType || type == kRos2DepthCameraTypeQualified;
}

void EnsureScene(gz::sensors::RenderingSensor *_sensor)
{
  if (!_sensor || _sensor->Scene())
  {
    return;
  }

  if (gz::rendering::loadedEngines().empty())
  {
    gzdbg << "Ros2CameraSystem EnsureScene: no rendering engines loaded yet"
          << std::endl;
    return;
  }

  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!scene)
  {
    gzdbg << "Ros2CameraSystem EnsureScene: rendering engine exists but no"
          << " scene is available yet" << std::endl;
    return;
  }

  gzdbg << "Ros2CameraSystem attaching scene [" << scene->Name()
        << "] to custom camera [" << _sensor->Name() << "]" << std::endl;
  _sensor->SetScene(scene);
}

sdf::ElementPtr PrepareSensorElement(
    const sdf::Sensor &_sensor,
    const std::string &_scopedName,
    const std::string &_defaultTopic)
{
  auto elem = _sensor.Element();
  if (!elem)
  {
    gzdbg << "PrepareSensorElement: sensor [" << _sensor.Name()
          << "] has no backing SDF element" << std::endl;
    return nullptr;
  }

  auto clone = elem->Clone();
  if (!clone)
  {
    gzerr << "PrepareSensorElement: failed cloning sensor element for ["
          << _sensor.Name() << "]" << std::endl;
    return nullptr;
  }

  auto nameAttr = clone->GetAttribute("name");
  if (nameAttr)
  {
    nameAttr->SetFromString(_scopedName);
  }

  if (clone->HasElement("topic"))
  {
    clone->GetElement("topic")->Set(_sensor.Topic().empty() ? _defaultTopic
                                                            : _sensor.Topic());
  }

  return clone;
}

}  // namespace

//////////////////////////////////////////////////
Ros2CameraSystem::~Ros2CameraSystem()
{
  this->ClearSensors();
}

//////////////////////////////////////////////////
void Ros2CameraSystem::Configure(const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &_eventMgr)
{
  this->preRenderConnection =
      _eventMgr.Connect<gz::sim::events::PreRender>(
          std::bind(&Ros2CameraSystem::OnPreRender, this));
  this->renderTeardownConnection =
      _eventMgr.Connect<gz::sim::events::RenderTeardown>(
          std::bind(&Ros2CameraSystem::OnRenderTeardown, this));
}

//////////////////////////////////////////////////
void Ros2CameraSystem::PreUpdate(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachNew<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_custom,
        const gz::sim::components::ParentEntity *_parent)->bool
      {
        const sdf::Sensor &rawData = _custom->Data();
        const auto customType = gz::sensors::customType(rawData);
        gzdbg << "Ros2CameraSystem discovered CustomSensor entity [" << _entity
              << "] with gz:type [" << customType << "]" << std::endl;

        if (!IsRos2CameraType(rawData) && !IsRos2DepthCameraType(rawData))
        {
          gzdbg << "Ros2CameraSystem skipping CustomSensor entity [" << _entity
                << "] because it is not a supported custom camera sensor"
                << std::endl;
          return true;
        }

        auto sensorScopedName = gz::sim::removeParentScope(
            gz::sim::scopedName(_entity, _ecm, "::", false), "::");

        sdf::Sensor data = rawData;
        data.SetName(sensorScopedName);

        const bool isDepthCamera = IsRos2DepthCameraType(rawData);
        const std::string sensorKind = isDepthCamera ? "Ros2DepthCamera"
                                                     : "Ros2Camera";
        const std::string defaultTopic =
            gz::sim::scopedName(_entity, _ecm) + "/" + sensorKind;
        if (data.Topic().empty())
        {
          data.SetTopic(defaultTopic);
        }

        gzdbg << "Ros2CameraSystem creating " << sensorKind << " ["
              << sensorScopedName
              << "] on topic [" << data.Topic() << "]" << std::endl;

        gz::sensors::SensorFactory sensorFactory;
        std::shared_ptr<gz::sensors::Sensor> sensor;
        auto sensorElem = PrepareSensorElement(rawData, sensorScopedName,
            defaultTopic);
        if (isDepthCamera)
        {
          std::unique_ptr<custom::Ros2DepthCamera> depthSensor;
          if (sensorElem)
          {
            gzdbg << "Ros2CameraSystem creating depth sensor from raw SDF"
                  << " element for [" << sensorScopedName << "]" << std::endl;
            depthSensor =
                sensorFactory.CreateSensor<custom::Ros2DepthCamera>(sensorElem);
          }
          else
          {
            gzdbg << "Ros2CameraSystem falling back to sdf::Sensor creation"
                  << " for depth sensor [" << sensorScopedName << "] because"
                  << " the original SDF element is unavailable" << std::endl;
            depthSensor =
                sensorFactory.CreateSensor<custom::Ros2DepthCamera>(data);
          }
          sensor = std::shared_ptr<gz::sensors::Sensor>(std::move(depthSensor));
        }
        else
        {
          std::unique_ptr<custom::Ros2Camera> colorSensor;
          if (sensorElem)
          {
            gzdbg << "Ros2CameraSystem creating sensor from raw SDF element"
                  << " for [" << sensorScopedName << "]" << std::endl;
            colorSensor =
                sensorFactory.CreateSensor<custom::Ros2Camera>(sensorElem);
          }
          else
          {
            gzdbg << "Ros2CameraSystem falling back to sdf::Sensor creation"
                  << " for [" << sensorScopedName << "] because the original"
                  << " SDF element is unavailable" << std::endl;
            colorSensor = sensorFactory.CreateSensor<custom::Ros2Camera>(data);
          }
          sensor = std::shared_ptr<gz::sensors::Sensor>(std::move(colorSensor));
        }
        if (!sensor)
        {
          gzerr << "Failed to create " << sensorKind << " ["
                << sensorScopedName << "]"
                << " from custom sensor type [" << customType << "]"
                << std::endl;
          return true;
        }

        const auto *parentNameComp =
            _ecm.Component<gz::sim::components::Name>(_parent->Data());
        if (parentNameComp)
        {
          sensor->SetParent(parentNameComp->Data());
        }

        auto renderingSensor =
            std::dynamic_pointer_cast<gz::sensors::RenderingSensor>(sensor);
        if (renderingSensor)
        {
          EnsureScene(renderingSensor.get());
        }

        _ecm.CreateComponent(_entity,
            gz::sim::components::SensorTopic(sensor->Topic()));

        {
          std::lock_guard<std::mutex> lock(this->mutex);
          this->entitySensorMap[_entity] = sensor;
        }

        gzdbg << "Ros2CameraSystem created " << sensorKind << " for entity ["
              << _entity << "]" << std::endl;
        return true;
      });
}

//////////////////////////////////////////////////
void Ros2CameraSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->simTime = _info.simTime;
    this->paused = _info.paused;
    for (auto &[entity, sensor] : this->entitySensorMap)
    {
      (void)sensor;
      this->entityPoseMap[entity] = gz::sim::worldPose(entity, _ecm);
    }
  }

  this->RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void Ros2CameraSystem::RemoveSensorEntities(
    const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<gz::sim::components::CustomSensor>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *)->bool
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          this->entitySensorMap.erase(it);
        }
        this->entityPoseMap.erase(_entity);
        return true;
      });
}

//////////////////////////////////////////////////
void Ros2CameraSystem::ClearSensors()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  for (auto &[entity, sensor] : this->entitySensorMap)
  {
    (void)entity;
    (void)sensor;
  }

  this->entitySensorMap.clear();
  this->entityPoseMap.clear();
}

//////////////////////////////////////////////////
void Ros2CameraSystem::OnPreRender()
{
  struct SensorState
  {
    std::shared_ptr<gz::sensors::Sensor> sensor;
    gz::math::Pose3d pose;
  };

  std::vector<SensorState> sensors;
  std::chrono::steady_clock::duration now;
  bool isPaused = true;

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    now = this->simTime;
    isPaused = this->paused;
    sensors.reserve(this->entitySensorMap.size());
    for (const auto &[entity, sensor] : this->entitySensorMap)
    {
      auto poseIt = this->entityPoseMap.find(entity);
      if (poseIt == this->entityPoseMap.end())
      {
        // Pose not yet initialized by PostUpdate - skip this frame
        gzdbg << "Ros2CameraSystem OnPreRender: sensor ["
              << sensor->Name()
              << "] pose not yet available, skipping" << std::endl;
        continue;
      }
      sensors.push_back({sensor, poseIt->second});
    }
  }

  if (isPaused)
  {
    return;
  }

  for (const auto &sensorState : sensors)
  {
    auto renderingSensor =
        std::dynamic_pointer_cast<gz::sensors::RenderingSensor>(
            sensorState.sensor);
    if (!renderingSensor)
    {
      gzerr << "Ros2CameraSystem OnPreRender: failed to cast sensor ["
            << sensorState.sensor->Name()
            << "] to gz::sensors::RenderingSensor"
            << std::endl;
      continue;
    }

    EnsureScene(renderingSensor.get());
    if (!renderingSensor->Scene())
    {
      gzdbg << "Ros2CameraSystem OnPreRender: sensor ["
            << sensorState.sensor->Name()
            << "] still has no scene, skipping update" << std::endl;
      continue;
    }

    renderingSensor->SetPose(sensorState.pose);

    gzdbg << "Ros2CameraSystem OnPreRender: updating sensor ["
          << sensorState.sensor->Name() << "] at sim time ["
          << std::chrono::duration_cast<std::chrono::milliseconds>(now).count()
          << "ms]" << std::endl;
    sensorState.sensor->Update(now, false);
  }
}

//////////////////////////////////////////////////
void Ros2CameraSystem::OnRenderTeardown()
{
  this->ClearSensors();
}

GZ_ADD_PLUGIN(Ros2CameraSystem, gz::sim::System,
  Ros2CameraSystem::ISystemConfigure,
  Ros2CameraSystem::ISystemPreUpdate,
  Ros2CameraSystem::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Ros2CameraSystem, "custom::Ros2CameraSystem")

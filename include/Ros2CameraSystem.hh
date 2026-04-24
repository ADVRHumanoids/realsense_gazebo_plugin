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
#ifndef ROS2CAMERASYSTEM_HH_
#define ROS2CAMERASYSTEM_HH_

#include <chrono>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <gz/common/Event.hh>
#include <gz/math/Pose3.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sim/System.hh>

/// \brief Gazebo system that intercepts plain camera and depth_camera
/// sensor entities and creates Ros2CameraSensor / Ros2DepthCameraSensor
/// instances instead of the default Gazebo sensor types.
///
/// Registration
/// ------------
/// Add to your world SDF:
///
///   <plugin filename="Ros2CameraSystem"
///           name="custom::Ros2CustomSensorSystem"/>
///
/// The system loads both Ros2Camera and Ros2DepthCamera and Ros2RgbdCamera
/// typed sensor entities.


namespace custom
{
  class Ros2Camera;
  class Ros2DepthCamera;

  /// \brief Example showing how to tie a custom sensor, in this case an
  /// odometer, into simulation
  class Ros2CameraSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: ~Ros2CameraSystem() override;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) final;

    // Documentation inherited.
    // During PreUpdate, check for new sensors that were inserted
    // into simulation and create more components as needed.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

    // Documentation inherited.
    // During PostUpdate, update the known sensors and publish their data.
    // Also remove sensors that have been deleted.
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) final;

    /// \brief Remove custom sensors if their entities have been removed from
    /// simulation.
    /// \param[in] _ecm Immutable reference to ECM.
    private: void RemoveSensorEntities(
        const gz::sim::EntityComponentManager &_ecm);

    /// \brief Detach rendering state and release all tracked sensors.
    private: void ClearSensors();

    /// \brief Render-thread callback used to update rendering sensors.
    private: void OnPreRender();

    /// \brief Render-thread teardown callback.
    private: void OnRenderTeardown();

    /// \brief Render-thread snapshot for a tracked sensor entity.
    private: struct TrackedSensorState
    {
      std::shared_ptr<gz::sensors::Sensor> sensor;
      gz::math::Pose3d pose;
      bool poseReady{false};
    };

    /// \brief Active custom rendering sensors and their latest poses.
    private: std::unordered_map<gz::sim::Entity, TrackedSensorState>
        trackedSensorMap;

    /// \brief Synchronizes sim-thread bookkeeping with render-thread updates.
    private: std::mutex mutex;

    /// \brief Latest simulation time seen on the sim thread.
    private: std::chrono::steady_clock::duration simTime{
        std::chrono::steady_clock::duration::zero()};

    /// \brief Cached paused flag from the sim thread.
    private: bool paused{true};

    /// \brief Connection to the render-thread pre-render callback.
    private: gz::common::ConnectionPtr preRenderConnection;

    /// \brief Connection to the render-thread teardown callback.
    private: gz::common::ConnectionPtr renderTeardownConnection;

  };
}

#endif // ROS2CAMERASYSTEM_HH_

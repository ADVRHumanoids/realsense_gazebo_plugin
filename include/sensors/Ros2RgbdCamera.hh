/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef ROS2_RGBD_CAMERA_HH_
#define ROS2_RGBD_CAMERA_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <gz/sensors/CameraSensor.hh>

namespace custom
{
  class Ros2RgbdCameraPrivate;

  /// \brief Custom RGBD camera sensor.
  ///
  /// This mirrors Gazebo Sensors' RGBD camera implementation, but is loaded as
  /// a custom sensor so Ros2CameraSystem can drive it from simulation poses and
  /// the outgoing point cloud can be adapted to the ROS optical-frame
  /// convention.
  class Ros2RgbdCamera : public gz::sensors::CameraSensor
  {
    /// \brief Constructor.
    public: Ros2RgbdCamera();

    /// \brief Destructor.
    public: ~Ros2RgbdCamera() override;

    /// \brief Load the sensor from an sdf::Sensor object.
    public: bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Load the sensor from an SDF element.
    public: bool Load(sdf::ElementPtr _sdf) override;

    /// \brief Initialize the sensor.
    public: bool Init() override;

    /// \brief Force the sensor to generate data.
    public: bool Update(
        const std::chrono::steady_clock::duration &_now) override;

    /// \brief Set the rendering scene.
    public: void SetScene(gz::rendering::ScenePtr _scene) override;

    /// \brief Get image width.
    public: unsigned int ImageWidth() const override;

    /// \brief Get image height.
    public: unsigned int ImageHeight() const override;

    /// \brief Check whether any RGBD output has subscribers.
    public: bool HasConnections() const override;

    /// \brief Check whether the color image has subscribers.
    public: bool HasColorConnections() const;

    /// \brief Check whether the depth image has subscribers.
    public: bool HasDepthConnections() const;

    /// \brief Check whether the point cloud has subscribers.
    public: bool HasPointConnections() const;

    /// \brief Create the rendering camera in the current scene.
    private: bool CreateCameras();

    /// \brief Private data.
    private: std::unique_ptr<Ros2RgbdCameraPrivate> dataPtr;
  };
}  // namespace custom

#endif  // ROS2_RGBD_CAMERA_HH_

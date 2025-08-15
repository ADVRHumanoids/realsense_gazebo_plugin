// Copyright (c) 2024 Pal Robotics S.L. All rights reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/DepthCamera.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/DepthCameraSensor.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <sdf/sdf.hh>

namespace gz
{
namespace realsense_gazebo_plugin
{
#define DEPTH_CAMERA_NAME "depth"
#define COLOR_CAMERA_NAME "color"
#define IRED1_CAMERA_NAME "ired1"
#define IRED2_CAMERA_NAME "ired2"

struct CameraParams
{
  CameraParams()
  {
  }

  std::string topic_name;
  std::string camera_info_topic_name;
  std::string optical_frame;
};

/// \brief A plugin that simulates Real Sense camera streams.
class RealSensePlugin : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPostUpdate
{
  /// \brief Constructor.

public:
  RealSensePlugin();

  /// \brief Destructor.
  ~RealSensePlugin();

  // Documentation Inherited.
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  // Documentation Inherited.
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Initialize rendering sensors from sensor entities
  bool InitializeRenderingSensors(const gz::sim::EntityComponentManager &_ecm);

  /// \brief Callback that publishes a received Depth Camera Frame as an
  /// ImageStamped
  /// message.
  virtual void OnNewDepthFrame();

  /// \brief Callback that publishes a received Camera Frame as an
  /// ImageStamped message.
  virtual void OnNewFrame(
    const gz::rendering::CameraPtr cam,
    gz::transport::Node::Publisher pub);

protected:
  /// \brief Entity of the model containing the plugin.
  gz::sim::Entity modelEntity;

  /// \brief Pointer to the ECM.
  gz::sim::EntityComponentManager *ecm;

  /// \brief Pointer to the Depth Camera Renderer.
  gz::rendering::DepthCameraPtr depthCam;

  /// \brief Pointer to the Color Camera Renderer.
  gz::rendering::CameraPtr colorCam;

  /// \brief Pointer to the Infrared Camera Renderer.
  gz::rendering::CameraPtr ired1Cam;

  /// \brief Pointer to the Infrared2 Camera Renderer.
  gz::rendering::CameraPtr ired2Cam;

  /// \brief String to hold the camera prefix
  std::string prefix;

  /// \brief Pointer to the transport Node.
  gz::transport::Node node;

  // \brief Store Real Sense depth map data.
  std::vector<uint16_t> depthMap;

  /// \brief Pointer to the Depth Publisher.
  gz::transport::Node::Publisher depthPub;

  /// \brief Pointer to the Color Publisher.
  gz::transport::Node::Publisher colorPub;

  /// \brief Pointer to the Infrared Publisher.
  gz::transport::Node::Publisher ired1Pub;

  /// \brief Pointer to the Infrared2 Publisher.
  gz::transport::Node::Publisher ired2Pub;

  /// \brief Sensor entities.
  gz::sim::Entity depthEntity, colorEntity, ired1Entity, ired2Entity;

  std::map<std::string, CameraParams> cameraParamsMap_;

  bool pointCloud_ = false;
  std::string pointCloudTopic_;
  double pointCloudCutOff_, pointCloudCutOffMax_;

  double colorUpdateRate_;
  double infraredUpdateRate_;
  double depthUpdateRate_;

  float rangeMinDepth_;
  float rangeMaxDepth_;
};
}  // namespace realsense_gazebo_plugin
}  // namespace gz

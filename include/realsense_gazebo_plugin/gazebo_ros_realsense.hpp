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
#include <thread>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "realsense_gazebo_plugin/RealSensePlugin.hpp"
#include <gz/plugin/Register.hh>

namespace gz {
namespace realsense_gazebo_plugin {
/// \brief A plugin that simulates Real Sense camera streams.
class GazeboRosRealsense : public RealSensePlugin {
  /// \brief Constructor.

public:
  GazeboRosRealsense();

  /// \brief Destructor.

public:
  ~GazeboRosRealsense();

  // Documentation Inherited.

public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  // Documentation Inherited.
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Callback that publishes a received Depth Camera Frame as an
  /// ImageStamped message.

public:
  virtual void OnNewDepthFrame();

  /// \brief Helper function to fill the pointcloud information
  bool FillPointCloudHelper(sensor_msgs::msg::PointCloud2 &point_cloud_msg,
                            uint32_t rows_arg, uint32_t cols_arg,
                            uint32_t step_arg, const void *data_arg);

  /// \brief Callback that publishes a received Camera Frame as an
  /// ImageStamped message.

public:
  virtual void OnNewFrame(const gz::rendering::CameraPtr cam,
                          gz::transport::Node::Publisher pub) override;

  /// \brief Process and publish sensor data to ROS topics
  void PublishSensorData(const gz::sim::EntityComponentManager &_ecm);

  /// \brief Timer callback for publishing sensor data
  void TimerCallback();

  /// \brief A pointer to the ROS node.
  ///  A node will be instantiated if it does not exist.

protected:
  rclcpp::Node::SharedPtr node_;

protected:
  image_transport::Publisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::thread ros_spinner_thread_;

  /// \brief ROS image messages

protected:
  sensor_msgs::msg::Image image_msg_, depth_msg_;
  sensor_msgs::msg::PointCloud2 pointcloud_msg_;
};
} // namespace realsense_gazebo_plugin
} // namespace gz

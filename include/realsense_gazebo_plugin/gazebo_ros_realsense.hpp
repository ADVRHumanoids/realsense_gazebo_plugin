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

// #include <camera_info_manager/camera_info_manager.hpp>  // TODO: Re-enable when camera_info_manager is available
// #include <image_transport/image_transport.hpp>  // TODO: Re-enable when image_transport is available
// #include <point_cloud_transport/point_cloud_transport.hpp>  // TODO: Re-enable when point_cloud_transport is available
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "realsense_gazebo_plugin/RealSensePlugin.hpp"

namespace gz
{
namespace realsense_gazebo_plugin
{
/// \brief A plugin that simulates Real Sense camera streams.
class GazeboRosRealsense : public RealSensePlugin
{
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

  /// \brief Callback that publishes a received Depth Camera Frame as an
  /// ImageStamped message.

public:
  virtual void OnNewDepthFrame();

  /// \brief Helper function to fill the pointcloud information
  bool FillPointCloudHelper(
    sensor_msgs::msg::PointCloud2 & point_cloud_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, const void * data_arg);

  /// \brief Callback that publishes a received Camera Frame as an
  /// ImageStamped message.

public:
  virtual void OnNewFrame(
    const gz::rendering::CameraPtr cam,
    gz::transport::Node::Publisher pub) override;

// protected:
  // boost::shared_ptr<camera_info_manager::CameraInfoManager>
  // camera_info_manager_;  // TODO: Re-enable when camera_info_manager is available

  /// \brief A pointer to the ROS node.
  ///  A node will be instantiated if it does not exist.

protected:
  rclcpp::Node::SharedPtr node_;

// private:
  // std::unique_ptr<point_cloud_transport::PointCloudTransport> pctnode_;  // TODO: Re-enable when point_cloud_transport is available

// protected:
  // image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;  // TODO: Re-enable when image_transport is available
  // point_cloud_transport::Publisher pointcloud_pub_;  // TODO: Re-enable when point_cloud_transport is available

  /// \brief ROS image messages

protected:
  sensor_msgs::msg::Image image_msg_, depth_msg_;
  sensor_msgs::msg::PointCloud2 pointcloud_msg_;
};
}  // namespace realsense_gazebo_plugin
}  // namespace gz

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

#include "realsense_gazebo_plugin/gazebo_ros_realsense.hpp"
// #include <sensor_msgs/fill_image.hpp>  // TODO: Re-enable when available
// #include <sensor_msgs/image_encodings.hpp>  // TODO: Re-enable when available  
// #include <sensor_msgs/point_cloud2_iterator.hpp>  // TODO: Re-enable when available

// TODO: Re-enable when dependencies are available
// namespace
// {
// std::string extractCameraName(const std::string & name);
// sensor_msgs::msg::CameraInfo cameraInfo(
//   const sensor_msgs::msg::Image & image,
//   float horizontal_fov);
// }  // namespace

namespace gz
{
namespace realsense_gazebo_plugin  
{

GazeboRosRealsense::GazeboRosRealsense() {}

GazeboRosRealsense::~GazeboRosRealsense()
{
  if (this->node_) {
    RCLCPP_DEBUG_STREAM(this->node_->get_logger(), "realsense_camera Unloaded");
  }
}

void GazeboRosRealsense::Configure(const gz::sim::Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   gz::sim::EntityComponentManager &_ecm,
                                   gz::sim::EventManager &_eventMgr)
{
  RealSensePlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  std::string node_name = "gazebo_realsense";
  node_name += this->prefix.empty() ? "" : "_" + this->prefix;
  this->node_ = rclcpp::Node::make_shared(node_name);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "A ROS node for Gazebo has not been initialized, unable "
      "to load plugin. "
      "Load the Gazebo system plugin "
      "'libgazebo_ros_api_plugin.so' in the gazebo_ros "
      "package");
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Realsense Gazebo ROS plugin loading.");

  // TODO: Re-enable when dependencies are available
  // initialize camera_info_manager
  // this->camera_info_manager_.reset(
  //   new camera_info_manager::CameraInfoManager(
  //     this->node_.get(), this->GetHandle()));

  // this->color_pub_ = image_transport::create_camera_publisher(
  //   this->node_.get(), prefix + std::string("/") +
  //   cameraParamsMap_[COLOR_CAMERA_NAME].topic_name, rmw_qos_profile_sensor_data);
  // this->ir1_pub_ = image_transport::create_camera_publisher(
  //   this->node_.get(), prefix + std::string("/") +
  //   cameraParamsMap_[IRED1_CAMERA_NAME].topic_name, rmw_qos_profile_sensor_data);
  // this->ir2_pub_ = image_transport::create_camera_publisher(
  //   this->node_.get(), prefix + std::string("/") +
  //   cameraParamsMap_[IRED2_CAMERA_NAME].topic_name, rmw_qos_profile_sensor_data);
  // this->depth_pub_ = image_transport::create_camera_publisher(
  //   this->node_.get(), prefix + std::string("/") +
  //   cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name, rmw_qos_profile_sensor_data);

  // TODO: Re-enable when point_cloud_transport is available
  // if (pointCloud_) {
  //   this->pctnode_ = std::make_unique<point_cloud_transport::PointCloudTransport>(this->node_);
  //   this->pointcloud_pub_ = this->pctnode_->advertise(
  //     prefix + std::string(
  //       "/") + pointCloudTopic_, rmw_qos_profile_sensor_data);
  // }

  RCLCPP_INFO(node_->get_logger(), "Loaded Realsense Gazebo ROS plugin.");
}

void GazeboRosRealsense::OnNewFrame(
  const gz::rendering::CameraPtr cam,
  gz::transport::Node::Publisher pub)
{
  // TODO: This method needs significant rework for gz-sim
  // The ROS integration would need to be completely reimplemented
  // for the new gz-sim architecture
}

// Referenced from gazebo_plugins
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp#L302
// Fill depth information
bool GazeboRosRealsense::FillPointCloudHelper(
  sensor_msgs::msg::PointCloud2 & point_cloud_msg, uint32_t rows_arg,
  uint32_t cols_arg, uint32_t step_arg, const void * data_arg)
{
  // TODO: Re-enable when point_cloud_transport and related dependencies are available
  (void)point_cloud_msg;
  (void)rows_arg;
  (void)cols_arg;
  (void)step_arg;
  (void)data_arg;
  return true;
}

void GazeboRosRealsense::OnNewDepthFrame()
{
  // TODO: This method needs significant rework for gz-sim
  // The ROS integration would need to be completely reimplemented
  // for the new gz-sim architecture
}
}  // namespace realsense_gazebo_plugin
}  // namespace gz

// TODO: Fix plugin registration for gz-sim8
// Register the plugin  
// GZ_REGISTER_SYSTEM_PLUGIN(gz::realsense_gazebo_plugin::GazeboRosRealsense)


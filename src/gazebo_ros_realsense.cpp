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

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok()) {
    // Initialize ROS2 if not already initialized
    std::cerr << "ROS2 not initialized, initializing now..." << std::endl;
    rclcpp::init(0, nullptr);
  }

  std::string node_name = "gazebo_realsense";
  node_name += this->prefix.empty() ? "" : "_" + this->prefix;
  this->node_ = rclcpp::Node::make_shared(node_name);
  RCLCPP_INFO(node_->get_logger(), "Realsense Gazebo ROS plugin loading.");

  // TODO: Re-enable when camera_info_manager is available
  // initialize camera_info_manager
  // this->camera_info_manager_.reset(
  //   new camera_info_manager::CameraInfoManager(
  //     this->node_.get(), this->GetHandle()));

  // Create image transport publishers
  image_transport::ImageTransport it(this->node_);
  this->color_pub_ = it.advertise(prefix + "/" + cameraParamsMap_[COLOR_CAMERA_NAME].topic_name, 1);
  this->ir1_pub_ = it.advertise(prefix + "/" + cameraParamsMap_[IRED1_CAMERA_NAME].topic_name, 1);
  this->ir2_pub_ = it.advertise(prefix + "/" + cameraParamsMap_[IRED2_CAMERA_NAME].topic_name, 1);
  this->depth_pub_ = it.advertise(prefix + "/" + cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name, 1);

  // TODO: Re-enable when point_cloud_transport is available
  // if (pointCloud_) {
  //   this->pctnode_ = std::make_unique<point_cloud_transport::PointCloudTransport>(this->node_);
  //   this->pointcloud_pub_ = this->pctnode_->advertise(
  //     prefix + std::string(
  //       "/") + pointCloudTopic_, rmw_qos_profile_sensor_data);
  // }

  RCLCPP_INFO(node_->get_logger(), "Loaded Realsense Gazebo ROS plugin.");
}

/////////////////////////////////////////////////
void GazeboRosRealsense::PostUpdate(const gz::sim::UpdateInfo &_info,
                                   const gz::sim::EntityComponentManager &_ecm)
{
  // Call parent PostUpdate first
  RealSensePlugin::PostUpdate(_info, _ecm);

  // For now, the image processing is triggered by sensor callbacks
  // In a full implementation, we would process sensor data here
  // and publish ROS messages at the appropriate rate
}

void GazeboRosRealsense::OnNewFrame(
  const gz::rendering::CameraPtr cam,
  gz::transport::Node::Publisher /*gzPub*/)
{
  if (!cam || !this->node_) {
    return;
  }

  // Create ROS image message
  auto imageMsg = std::make_unique<sensor_msgs::msg::Image>();

  // Set header info
  imageMsg->header.stamp = this->node_->now();

  // Set image dimensions
  imageMsg->width = cam->ImageWidth();
  imageMsg->height = cam->ImageHeight();

  // Get image data using new API
  gz::rendering::Image image = cam->CreateImage();
  cam->Copy(image);

  // Set image format and data based on camera type
  if (cam->ImageFormat() == gz::rendering::PF_R8G8B8) {
    imageMsg->encoding = "rgb8";
    imageMsg->step = cam->ImageWidth() * 3;
    imageMsg->data.resize(imageMsg->step * imageMsg->height);
    std::memcpy(imageMsg->data.data(), image.Data<unsigned char>(), image.MemorySize());

    // Publish color image
    if (cam == this->colorCam) {
      imageMsg->header.frame_id = cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame;
      this->color_pub_.publish(*imageMsg);
    }
  } else if (cam->ImageFormat() == gz::rendering::PF_L8) {
    imageMsg->encoding = "mono8";
    imageMsg->step = cam->ImageWidth() * 1;
    imageMsg->data.resize(imageMsg->step * imageMsg->height);
    std::memcpy(imageMsg->data.data(), image.Data<unsigned char>(), image.MemorySize());

    // Publish infrared images
    if (cam == this->ired1Cam) {
      imageMsg->header.frame_id = cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame;
      this->ir1_pub_.publish(*imageMsg);
    } else if (cam == this->ired2Cam) {
      imageMsg->header.frame_id = cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame;
      this->ir2_pub_.publish(*imageMsg);
    }
  }
}

// Referenced from gazebo_plugins
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp#L302
// Fill depth information
bool GazeboRosRealsense::FillPointCloudHelper(
  sensor_msgs::msg::PointCloud2 & /*point_cloud_msg*/, uint32_t /*rows_arg*/,
  uint32_t /*cols_arg*/, uint32_t /*step_arg*/, const void * /*data_arg*/)
{
  // TODO: Re-enable when point_cloud_transport and related dependencies are available
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

// Register the plugin using GZ_ADD_PLUGIN
GZ_ADD_PLUGIN(gz::realsense_gazebo_plugin::GazeboRosRealsense,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)


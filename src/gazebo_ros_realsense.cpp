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
#include <chrono>
#include <thread>
// #include <sensor_msgs/fill_image.hpp>  // TODO: Re-enable when available
// #include <sensor_msgs/image_encodings.hpp>  // TODO: Re-enable when available
// #include <sensor_msgs/point_cloud2_iterator.hpp>  // TODO: Re-enable when
// available

// TODO: Re-enable when dependencies are available
// namespace
// {
// std::string extractCameraName(const std::string & name);
// sensor_msgs::msg::CameraInfo cameraInfo(
//   const sensor_msgs::msg::Image & image,
//   float horizontal_fov);
// }  // namespace

namespace gz {
namespace realsense_gazebo_plugin {

GazeboRosRealsense::GazeboRosRealsense() {}

GazeboRosRealsense::~GazeboRosRealsense() {
  if (this->node_) {
    RCLCPP_DEBUG_STREAM(this->node_->get_logger(), "realsense_camera Unloaded");

    // Stop ROS2 spinner thread
    rclcpp::shutdown();
    if (this->ros_spinner_thread_.joinable()) {
      this->ros_spinner_thread_.join();
    }
  }
}

void GazeboRosRealsense::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr) {
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
  this->color_pub_ = it.advertise(
      prefix + "/" + cameraParamsMap_[COLOR_CAMERA_NAME].topic_name, 1);
  this->ir1_pub_ = it.advertise(
      prefix + "/" + cameraParamsMap_[IRED1_CAMERA_NAME].topic_name, 1);
  this->ir2_pub_ = it.advertise(
      prefix + "/" + cameraParamsMap_[IRED2_CAMERA_NAME].topic_name, 1);
  this->depth_pub_ = it.advertise(
      prefix + "/" + cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name, 1);

  // TODO: Re-enable when point_cloud_transport is available
  // if (pointCloud_) {
  //   this->pctnode_ =
  //   std::make_unique<point_cloud_transport::PointCloudTransport>(this->node_);
  //   this->pointcloud_pub_ = this->pctnode_->advertise(
  //     prefix + std::string(
  //       "/") + pointCloudTopic_, rmw_qos_profile_sensor_data);
  // }

  // Start timer-based publishing since PostUpdate might not be called
  // consistently
  this->publish_timer_ =
      this->node_->create_wall_timer(std::chrono::milliseconds(33), // ~30 Hz
                                     [this]() { this->TimerCallback(); });

  // Start ROS2 spinner thread to handle timer callbacks
  this->ros_spinner_thread_ = std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->node_);
    executor.spin();
  });

  RCLCPP_INFO(node_->get_logger(), "Loaded Realsense Gazebo ROS plugin.");
  RCLCPP_DEBUG(node_->get_logger(), "Started timer-based publishing at 30Hz");
  RCLCPP_DEBUG(node_->get_logger(), "Started ROS2 spinner thread");
}

/////////////////////////////////////////////////
void GazeboRosRealsense::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) {
  // Call parent PostUpdate first
  RealSensePlugin::PostUpdate(_info, _ecm);

  // Check if we have valid node and are running
  if (!this->node_) {
    static bool logged_no_node = false;
    if (!logged_no_node) {
      std::cerr << "DEBUG: No ROS node available in PostUpdate" << std::endl;
      logged_no_node = true;
    }
    return;
  }

  if (_info.paused) {
    return;
  }

  static bool logged_first_update = false;
  if (!logged_first_update) {
    RCLCPP_INFO(this->node_->get_logger(),
                "DEBUG: PostUpdate called for first time");
    logged_first_update = true;
  }

  // Publish at 30Hz rate (approximately every 33ms)
  static std::chrono::steady_clock::time_point lastPublishTime;
  auto currentTime = std::chrono::steady_clock::now();
  auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(
      currentTime - lastPublishTime);

  if (timeDiff.count() >= 33) { // ~30Hz
    this->PublishSensorData(_ecm);
    lastPublishTime = currentTime;

    // Log every 100 publishes to show activity
    static int publishCount = 0;
    publishCount++;
    if (publishCount % 100 == 0) {
      RCLCPP_DEBUG(this->node_->get_logger(), "Published %d sensor data sets",
                   publishCount);
    }
  }
}

/////////////////////////////////////////////////
void GazeboRosRealsense::PublishSensorData(
    const gz::sim::EntityComponentManager &_ecm) {
  // Create and publish dummy image data for testing
  // This is a simplified implementation that publishes test images

  auto stamp = this->node_->now();

  // Publish color image
  auto colorMsg = std::make_unique<sensor_msgs::msg::Image>();
  colorMsg->header.stamp = stamp;
  colorMsg->header.frame_id = cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame;
  colorMsg->width = 640;
  colorMsg->height = 480;
  colorMsg->encoding = "rgb8";
  colorMsg->step = colorMsg->width * 3;
  colorMsg->data.resize(colorMsg->step * colorMsg->height);

  // Fill with test pattern (red gradient)
  for (uint32_t y = 0; y < colorMsg->height; ++y) {
    for (uint32_t x = 0; x < colorMsg->width; ++x) {
      uint32_t idx = (y * colorMsg->width + x) * 3;
      colorMsg->data[idx] = (x * 255) / colorMsg->width;      // Red
      colorMsg->data[idx + 1] = (y * 255) / colorMsg->height; // Green
      colorMsg->data[idx + 2] = 128;                          // Blue
    }
  }
  this->color_pub_.publish(*colorMsg);

  // Publish depth image
  auto depthMsg = std::make_unique<sensor_msgs::msg::Image>();
  depthMsg->header.stamp = stamp;
  depthMsg->header.frame_id = cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame;
  depthMsg->width = 640;
  depthMsg->height = 480;
  depthMsg->encoding = "32FC1";
  depthMsg->step = depthMsg->width * 4;
  depthMsg->data.resize(depthMsg->step * depthMsg->height);

  // Fill with test depth pattern
  float *depth_data = reinterpret_cast<float *>(depthMsg->data.data());
  for (uint32_t y = 0; y < depthMsg->height; ++y) {
    for (uint32_t x = 0; x < depthMsg->width; ++x) {
      uint32_t idx = y * depthMsg->width + x;
      depth_data[idx] =
          1.0f + (float)y / depthMsg->height * 9.0f; // 1-10m depth
    }
  }
  this->depth_pub_.publish(*depthMsg);

  // Publish infrared images
  auto ir1Msg = std::make_unique<sensor_msgs::msg::Image>();
  ir1Msg->header.stamp = stamp;
  ir1Msg->header.frame_id = cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame;
  ir1Msg->width = 640;
  ir1Msg->height = 480;
  ir1Msg->encoding = "mono8";
  ir1Msg->step = ir1Msg->width;
  ir1Msg->data.resize(ir1Msg->step * ir1Msg->height);

  // Fill with test IR pattern
  for (uint32_t i = 0; i < ir1Msg->data.size(); ++i) {
    ir1Msg->data[i] = (i % 255);
  }
  this->ir1_pub_.publish(*ir1Msg);

  auto ir2Msg = std::make_unique<sensor_msgs::msg::Image>();
  ir2Msg->header.stamp = stamp;
  ir2Msg->header.frame_id = cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame;
  ir2Msg->width = 640;
  ir2Msg->height = 480;
  ir2Msg->encoding = "mono8";
  ir2Msg->step = ir2Msg->width;
  ir2Msg->data.resize(ir2Msg->step * ir2Msg->height);

  // Fill with different test IR pattern
  for (uint32_t i = 0; i < ir2Msg->data.size(); ++i) {
    ir2Msg->data[i] = 255 - (i % 255);
  }
  this->ir2_pub_.publish(*ir2Msg);
}

/////////////////////////////////////////////////
void GazeboRosRealsense::TimerCallback() {
  if (!this->node_) {
    return;
  }

  // Use a dummy ECM for the timer-based publishing
  gz::sim::EntityComponentManager dummyEcm;
  this->PublishSensorData(dummyEcm);

  static int count = 0;
  count++;
  if (count % 30 == 0) { // Log every second at 30Hz
    RCLCPP_DEBUG(this->node_->get_logger(),
                 "Published %d sensor data frames via timer", count);
  }
}

void GazeboRosRealsense::OnNewFrame(const gz::rendering::CameraPtr cam,
                                    gz::transport::Node::Publisher /*gzPub*/) {
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
    std::memcpy(imageMsg->data.data(), image.Data<unsigned char>(),
                image.MemorySize());

    // Publish color image
    if (cam == this->colorCam) {
      imageMsg->header.frame_id =
          cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame;
      this->color_pub_.publish(*imageMsg);
    }
  } else if (cam->ImageFormat() == gz::rendering::PF_L8) {
    imageMsg->encoding = "mono8";
    imageMsg->step = cam->ImageWidth() * 1;
    imageMsg->data.resize(imageMsg->step * imageMsg->height);
    std::memcpy(imageMsg->data.data(), image.Data<unsigned char>(),
                image.MemorySize());

    // Publish infrared images
    if (cam == this->ired1Cam) {
      imageMsg->header.frame_id =
          cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame;
      this->ir1_pub_.publish(*imageMsg);
    } else if (cam == this->ired2Cam) {
      imageMsg->header.frame_id =
          cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame;
      this->ir2_pub_.publish(*imageMsg);
    }
  }
}

// Referenced from gazebo_plugins
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp#L302
// Fill depth information
bool GazeboRosRealsense::FillPointCloudHelper(
    sensor_msgs::msg::PointCloud2 & /*point_cloud_msg*/, uint32_t /*rows_arg*/,
    uint32_t /*cols_arg*/, uint32_t /*step_arg*/, const void * /*data_arg*/) {
  // TODO: Re-enable when point_cloud_transport and related dependencies are
  // available
  return true;
}

void GazeboRosRealsense::OnNewDepthFrame() {
  // TODO: This method needs significant rework for gz-sim
  // The ROS integration would need to be completely reimplemented
  // for the new gz-sim architecture
}
} // namespace realsense_gazebo_plugin
} // namespace gz

// Register the plugin using GZ_ADD_PLUGIN
GZ_ADD_PLUGIN(gz::realsense_gazebo_plugin::GazeboRosRealsense, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)

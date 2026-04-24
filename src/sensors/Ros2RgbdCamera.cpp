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

#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/DepthCamera.hh>

#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/rendering/Utils.hh>
#include <gz/sensors/ImageGaussianNoiseModel.hh>
#include <gz/sensors/ImageNoise.hh>
#include "sensors/Ros2RgbdCamera.hh"
#include <gz/sensors/RenderingEvents.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include "utils/CameraSensorUtil.hh"
#include "utils/PointCloudUtil.hh"

namespace custom
{
namespace
{

constexpr const char *kPublishPointCloudElement = "publish_pointcloud";

bool ToRgbdCameraSensor(sdf::ElementPtr _sensorElem, sdf::Sensor &_sensor)
{
  if (!_sensorElem)
  {
    gzerr << "ToRgbdCameraSensor(ElementPtr): null sensor element"
          << std::endl;
    return false;
  }

  auto clone = _sensorElem->Clone();
  if (!clone)
  {
    gzerr << "ToRgbdCameraSensor(ElementPtr): failed cloning sensor element"
          << std::endl;
    return false;
  }

  gzdbg << "ToRgbdCameraSensor(ElementPtr): original name=["
        << clone->Get<std::string>("name", "").first << "] type=["
        << clone->Get<std::string>("type", "").first << "]" << std::endl;

  auto typeAttr = clone->GetAttribute("type");
  if (!typeAttr || !typeAttr->SetFromString("rgbd_camera"))
  {
    gzerr << "ToRgbdCameraSensor(ElementPtr): failed setting type attribute to"
          << " rgbd_camera" << std::endl;
    return false;
  }

  const auto errors = _sensor.Load(clone);
  if (!errors.empty())
  {
    gzerr << "ToRgbdCameraSensor(ElementPtr): sdf::Sensor::Load returned "
          << errors.size() << " errors" << std::endl;
  }
  return errors.empty();
}

bool ToRgbdCameraSensor(const sdf::Sensor &_input, sdf::Sensor &_rgbdSensor)
{
  gzdbg << "ToRgbdCameraSensor(Sensor): name=[" << _input.Name()
        << "] type=[" << _input.TypeStr() << "] hasCamera=["
        << (_input.CameraSensor() != nullptr) << "] hasElement=["
        << (_input.Element() != nullptr) << "]" << std::endl;

  if (_input.Type() == sdf::SensorType::RGBD_CAMERA && _input.CameraSensor())
  {
    _rgbdSensor = _input;
    return true;
  }

  return ToRgbdCameraSensor(_input.Element(), _rgbdSensor);
}

void RotatePointCloudXyz(float *_pointCloud, unsigned int _width,
    unsigned int _height, unsigned int _channels)
{
  if (!_pointCloud || _channels < 3)
  {
    return;
  }

  const auto samples = _width * _height;
  for (unsigned int i = 0; i < samples; ++i)
  {
    const auto index = i * _channels;
    const float x = _pointCloud[index + 0];
    const float y = _pointCloud[index + 1];
    const float z = _pointCloud[index + 2];

    _pointCloud[index + 0] = -y;
    _pointCloud[index + 1] = -z;
    _pointCloud[index + 2] = x;
  }
}

}  // namespace

/// \brief Private data for Ros2RgbdCamera
class Ros2RgbdCameraPrivate
{
  /// \brief Depth data callback used to get the data from the sensor
  /// \param[in] _scan pointer to the data from the sensor
  /// \param[in] _width width of the depth image
  /// \param[in] _height height of the depth image
  /// \param[in] _channel bytes used for the depth data
  /// \param[in] _format string with the format
  public: void OnNewDepthFrame(const float *_scan,
              unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &_format);

  /// \brief Point cloud data callback used to get the data from the sensor
  /// \param[in] _scan pointer to the data from the sensor
  /// \param[in] _width width of the point cloud image
  /// \param[in] _height height of the point cloud image
  /// \param[in] _channel bytes used for the point cloud data
  /// \param[in] _format string with the format
  public: void OnNewRgbPointCloud(const float *_scan,
              unsigned int _width, unsigned int _height,
                    unsigned int _channels,
                    const std::string &_format);

  /// \brief node to create publisher
  public: gz::transport::Node node;

  /// \brief publisher to publish images
  public: gz::transport::Node::Publisher imagePub;

  /// \brief publisher to publish depth images
  public: gz::transport::Node::Publisher depthPub;

  /// \brief publisher to publish point cloud
  public: gz::transport::Node::Publisher pointPub;

  /// \brief True if pointcloud generation/publication is allowed by SDF.
  public: bool publishPointCloud = true;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering camera
  public: gz::rendering::DepthCameraPtr depthCamera;

  /// \brief Depth data buffer.
  public: float *depthBuffer = nullptr;

  /// \brief Point cloud data buffer.
  public: float *pointCloudBuffer = nullptr;

  /// \brief True if a depth far clipping value has been set.
  public: bool hasDepthFarClip = false;

  /// \brief True if a depth near clipping value has been set.
  public: bool hasDepthNearClip = false;

  /// \brief Depth camera far clipping distance in meters.
  public: double depthFarClip = 10.0;

  /// \brief Depth camera near clipping distance in meters.
  public: double depthNearClip = 0.1;

  /// \brief The number of channels (x, y, z, rgba, ...) in the
  /// point cloud.
  public: unsigned int channels = 4;

  /// \brief Pointer to an image to be published
  public: gz::rendering::Image image;

  /// \brief Noise added to sensor data
  public: std::map<gz::sensors::SensorNoiseType, gz::sensors::NoisePtr> noises;

  /// \brief Connection from depth camera with new depth data
  public: gz::common::ConnectionPtr depthConnection;

  /// \brief Connection from depth camera with new point cloud data
  public: gz::common::ConnectionPtr pointCloudConnection;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief The point cloud message.
  public: gz::msgs::PointCloudPacked pointMsg;

  /// \brief Helper class that can fill a msgs::PointCloudPacked
  /// image and depth data.
  public: gz::sensors::PointCloudUtil pointsUtil;
};

//////////////////////////////////////////////////
Ros2RgbdCamera::Ros2RgbdCamera()
    : dataPtr(new Ros2RgbdCameraPrivate())
{
}

//////////////////////////////////////////////////
Ros2RgbdCamera::~Ros2RgbdCamera()
{
  this->dataPtr->depthConnection.reset();
  this->dataPtr->pointCloudConnection.reset();
  if (this->dataPtr->depthBuffer)
  delete [] this->dataPtr->depthBuffer;
  if (this->dataPtr->pointCloudBuffer)
  delete [] this->dataPtr->pointCloudBuffer;
}

//////////////////////////////////////////////////
bool Ros2RgbdCamera::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool Ros2RgbdCamera::Load(sdf::ElementPtr _sdf)
{
  if (!_sdf)
  {
    gzerr << "Ros2RgbdCamera::Load(ElementPtr) received null SDF"
          << std::endl;
    return false;
  }

  if (_sdf->HasElement(kPublishPointCloudElement))
  {
    this->dataPtr->publishPointCloud =
        _sdf->Get<bool>(kPublishPointCloudElement);
  }

  sdf::Sensor sensor;
  if (!ToRgbdCameraSensor(_sdf, sensor))
  {
    gzerr << "Ros2RgbdCamera failed converting raw SDF element into RGBD"
          << " camera sensor" << std::endl;
    return false;
  }

  return this->Load(sensor);
}

//////////////////////////////////////////////////
bool Ros2RgbdCamera::Load(const sdf::Sensor &_sdf)
{
  sdf::Sensor rgbdSensor;
  if (!ToRgbdCameraSensor(_sdf, rgbdSensor))
  {
    gzerr << "Ros2RgbdCamera failed converting sensor [" << _sdf.Name()
          << "] into RGBD camera SDF" << std::endl;
    return false;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->Sensor::Load(rgbdSensor))
  {
    return false;
  }

  // Check if this is the right type
  if (rgbdSensor.Type() != sdf::SensorType::RGBD_CAMERA)
  {
    gzerr << "Attempting to load an RGBD camera sensor, but received a "
          << rgbdSensor.TypeStr() << std::endl;
    return false;
  }

  auto *cameraSdf = rgbdSensor.CameraSensor();
  if (!cameraSdf)
  {
    gzerr << "Attempting to a load an RGBD Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = rgbdSensor;

  // Create the 2d image publisher
  this->dataPtr->imagePub =
      this->dataPtr->node.Advertise<gz::msgs::Image>(
          this->Topic() + "/image");
  if (!this->dataPtr->imagePub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() + "/image" << "].\n";
    return false;
  }

  gzdbg << "RGB images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "/image]" << std::endl;

  // Create the depth image publisher
  this->dataPtr->depthPub =
      this->dataPtr->node.Advertise<gz::msgs::Image>(
          this->Topic() + "/depth_image");
  if (!this->dataPtr->depthPub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() + "/depth_image" << "].\n";
    return false;
  }

  gzdbg << "Depth images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "/depth_image]" << std::endl;

  if (this->dataPtr->publishPointCloud)
  {
    // Create the point cloud publisher only when pointclouds are enabled.
    this->dataPtr->pointPub =
        this->dataPtr->node.Advertise<gz::msgs::PointCloudPacked>(
            this->Topic() + "/points");
    if (!this->dataPtr->pointPub)
    {
      gzerr << "Unable to create publisher on topic["
        << this->Topic() + "/points" << "].\n";
      return false;
    }

    gzdbg << "Points for [" << this->Name() << "] advertised on ["
           << this->Topic() << "/points]" << std::endl;
  }
  else
  {
    gzdbg << "Pointcloud topic disabled for [" << this->Name()
          << "] by <publish_pointcloud>false</publish_pointcloud>"
          << std::endl;
  }

  if (_sdf.CameraSensor()->Triggered())
  {
    std::string triggerTopic = cameraSdf->TriggerTopic();
    if (triggerTopic.empty())
    {
      triggerTopic = gz::transport::TopicUtils::AsValidTopic(this->Topic() +
                                                             "/trigger");
    }
    this->SetTriggered(true, triggerTopic);
  }

  if (!this->AdvertiseInfo(this->Topic() + "/camera_info"))
    return false;

  if (this->Scene())
  {
    this->CreateCameras();
  }

  this->dataPtr->sceneChangeConnection =
      gz::sensors::RenderingEvents::ConnectSceneChangeCallback(
          std::bind(&Ros2RgbdCamera::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool Ros2RgbdCamera::CreateCameras()
{
  sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();

  if (!cameraSdf)
  {
    gzerr << "Unable to access camera SDF element\n";
    return false;
  }

  unsigned int width = cameraSdf->ImageWidth();
  unsigned int height = cameraSdf->ImageHeight();

  if (width == 0u || height == 0u)
  {
    gzerr << "Unable to create an RGBD camera sensor with 0 width or height."
          << std::endl;
    return false;
  }

  this->dataPtr->depthCamera =
      this->Scene()->CreateDepthCamera(this->Name());
  this->dataPtr->depthCamera->SetImageWidth(width);
  this->dataPtr->depthCamera->SetImageHeight(height);
  this->dataPtr->depthCamera->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->depthCamera->SetFarClipPlane(cameraSdf->FarClip());

  // Depth camera clip params are new and only override the camera clip
  // params if specified.
  if (cameraSdf->HasDepthCamera())
  {
    if (cameraSdf->HasDepthFarClip())
    {
      this->dataPtr->hasDepthFarClip = true;
      this->dataPtr->depthFarClip = cameraSdf->DepthFarClip();
    }
    if (cameraSdf->HasDepthNearClip())
    {
      this->dataPtr->hasDepthNearClip = true;
      this->dataPtr->depthNearClip = cameraSdf->DepthNearClip();
    }
  }

  this->dataPtr->depthCamera->SetVisibilityMask(cameraSdf->VisibilityMask());
  this->dataPtr->depthCamera->SetLocalPose(this->Pose());

  this->AddSensor(this->dataPtr->depthCamera);

  const std::map<gz::sensors::SensorNoiseType, sdf::Noise> noises = {
      {gz::sensors::CAMERA_NOISE, cameraSdf->ImageNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    // Add gaussian noise to camera sensor
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
      {
        this->dataPtr->noises[noiseType] =
            gz::sensors::ImageNoiseFactory::NewNoiseModel(noiseSdf, "rgbd_camera");

        std::dynamic_pointer_cast<gz::sensors::ImageGaussianNoiseModel>(
            this->dataPtr->noises[noiseType])->SetCamera(
                this->dataPtr->depthCamera);
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      gzwarn << "The depth camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->depthCamera->SetAntiAliasing(2);

  gz::math::Angle angle = cameraSdf->HorizontalFov();
  // todo(anyone) verify that rgb pixels align with d for angles >90 degrees.
  if (angle < 0.01 || angle > GZ_PI * 2)
  {
    gzerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }

  this->dataPtr->depthCamera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->depthCamera->SetHFOV(angle);

  // Create depth texture when the camera is reconfigured from default values
  this->dataPtr->depthCamera->CreateDepthTexture();

  // \todo(nkoenig) Port Distortion class
  // This->dataPtr->distortion.reset(new Distortion());
  // This->dataPtr->distortion->Load(this->dataPtr->sdf->GetElement("distortion"));

  this->Scene()->RootVisual()->AddChild(this->dataPtr->depthCamera);

  this->UpdateLensIntrinsicsAndProjection(this->dataPtr->depthCamera,
      *cameraSdf);

  this->PopulateInfo(cameraSdf);

  this->dataPtr->depthConnection =
      this->dataPtr->depthCamera->ConnectNewDepthFrame(
          std::bind(&Ros2RgbdCameraPrivate::OnNewDepthFrame,
              this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // Initialize the point message.
  gz::msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->OpticalFrameId(),
      false,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"rgb", gz::msgs::PointCloudPacked::Field::FLOAT32}});

  // Set the values of the point message based on the camera information.
  this->dataPtr->pointMsg.set_width(this->ImageWidth());
  this->dataPtr->pointMsg.set_height(this->ImageHeight());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() * this->ImageWidth());

  return true;
}

/////////////////////////////////////////////////
void Ros2RgbdCamera::SetScene(gz::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->dataPtr->pointCloudConnection.reset();
    this->dataPtr->depthConnection.reset();
    // TODO(anyone) Remove cameras from current scene
    this->dataPtr->depthCamera = nullptr;
    gz::sensors::RenderingSensor::SetScene(_scene);

    if (this->dataPtr->initialized)
      this->CreateCameras();
    }
}

//////////////////////////////////////////////////
void Ros2RgbdCameraPrivate::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &/*_format*/)
{
  GZ_PROFILE("Ros2RgbdCameraPrivate::OnNewDepthFrame");
  std::lock_guard<std::mutex> lock(this->mutex);

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * sizeof(float);

  if (!this->depthBuffer)
    this->depthBuffer = new float[depthSamples];

  std::memcpy(this->depthBuffer, _scan, depthBufferSize);
}

//////////////////////////////////////////////////
void Ros2RgbdCameraPrivate::OnNewRgbPointCloud(const float *_scan,
    unsigned int _width, unsigned int _height,
    unsigned int _channels,
    const std::string &/*_format*/)
{
  GZ_PROFILE("Ros2RgbdCameraPrivate::OnNewRgbPointCloud");
  std::lock_guard<std::mutex> lock(this->mutex);

  unsigned int pointCloudSamples = _width * _height;
  unsigned int pointCloudBufferSize = pointCloudSamples * _channels *
      sizeof(float);
  this->channels = _channels;

  if (!this->pointCloudBuffer)
    this->pointCloudBuffer = new float[pointCloudSamples * _channels];

  std::memcpy(this->pointCloudBuffer, _scan, pointCloudBufferSize);
}

//////////////////////////////////////////////////
bool Ros2RgbdCamera::Update(
    const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("Ros2RgbdCamera::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->depthCamera)
  {
    gzerr << "Depth or image cameras don't exist.\n";
    return false;
  }

  if (this->HasInfoConnections())
  {
    // publish the camera info message
    this->PublishInfo(_now);
  }

  // don't render if there are no subscribers
  if (!this->HasColorConnections() && !this->HasDepthConnections() &&
      !this->HasPointConnections())
  {
    return false;
  }

  // Ros2CameraSystem drives this custom sensor from world poses. Keep the
  // owned rendering camera in sync before Render().
  this->dataPtr->depthCamera->SetLocalPose(this->Pose());

  if ((this->HasPointConnections() || this->HasColorConnections()) &&
      !this->dataPtr->pointCloudConnection)
  {
    this->dataPtr->pointCloudConnection =
        this->dataPtr->depthCamera->ConnectNewRgbPointCloud(
            std::bind(&Ros2RgbdCameraPrivate::OnNewRgbPointCloud,
                this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));
  }
  else if (!this->HasPointConnections() && !this->HasColorConnections() &&
           this->dataPtr->pointCloudConnection)
  {
    this->dataPtr->pointCloudConnection.reset();
  }

  unsigned int width = this->dataPtr->depthCamera->ImageWidth();
  unsigned int height = this->dataPtr->depthCamera->ImageHeight();
  unsigned int depthSamples = height * width;

  // generate sensor data
  this->Render();

  // create and publish the depth message
  if (this->HasDepthConnections())
  {
    gz::msgs::Image msg;
    msg.set_width(width);
    msg.set_height(height);
    msg.set_step(width * gz::rendering::PixelUtil::BytesPerPixel(
        gz::rendering::PF_FLOAT32_R));
    msg.set_pixel_format_type(gz::msgs::PixelFormatType::R_FLOAT32);
    *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->OpticalFrameId());

    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    // The following code is a work around since gz-rendering's depth camera
    // does not support 2 different clipping distances. An assumption is made
    // that the depth clipping distances are within bounds of the rgb clipping
    // distances, if not, the rgb clipping values will take priority.
    if (this->dataPtr->hasDepthNearClip || this->dataPtr->hasDepthFarClip)
    {
      for (unsigned int i = 0; i < depthSamples; i++)
      {
        if (this->dataPtr->hasDepthFarClip &&
            (this->dataPtr->depthBuffer[i] > this->dataPtr->depthFarClip))
        {
          this->dataPtr->depthBuffer[i] = gz::math::INF_D;
        }
        if (this->dataPtr->hasDepthNearClip &&
            (this->dataPtr->depthBuffer[i] < this->dataPtr->depthNearClip))
        {
          this->dataPtr->depthBuffer[i] = -gz::math::INF_D;
        }
      }
    }
    msg.set_data(this->dataPtr->depthBuffer,
        gz::rendering::PixelUtil::MemorySize(gz::rendering::PF_FLOAT32_R,
        width, height));

    // publish
    {
    this->AddSequence(msg.mutable_header(), "depthImage");
      GZ_PROFILE("RgbdCameraSensor::Update Publish depth image");
    this->dataPtr->depthPub.Publish(msg);
    }
  }

  if (this->dataPtr->pointCloudBuffer)
  {
    bool filledImgData = false;
    if (this->dataPtr->image.Width() != width
        || this->dataPtr->image.Height() != height)
    {
      this->dataPtr->image =
          gz::rendering::Image(width, height, gz::rendering::PF_R8G8B8);
    }

    // publish point cloud msg
    if (this->HasPointConnections())
    {
      // Set the time stamp
      *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
          gz::msgs::Convert(_now);

      if ((this->dataPtr->hasDepthNearClip || this->dataPtr->hasDepthFarClip)
          && this->dataPtr->depthBuffer)
      {
        for (unsigned int i = 0; i < depthSamples; i++)
        {
          float depthValue = this->dataPtr->depthBuffer[i];
          if (std::isinf(depthValue))
          {
            unsigned int index = i * this->dataPtr->channels;

            this->dataPtr->pointCloudBuffer[index] = depthValue;
            this->dataPtr->pointCloudBuffer[index + 1] = depthValue;
            this->dataPtr->pointCloudBuffer[index + 2] = depthValue;
          }
        }
      }
      // Apply rotation to the point cloud so that match the z-forward convention of ROS.
      if (this->OpticalFrameId() != this->FrameId())
      {
        RotatePointCloudXyz(this->dataPtr->pointCloudBuffer,
            width, height, this->dataPtr->channels);
      }

      GZ_PROFILE("RgbdCameraSensor::Update Fill Point Cloud");
      // fill point cloud msg and image data
      this->dataPtr->pointsUtil.FillMsg(this->dataPtr->pointMsg,
          this->dataPtr->pointCloudBuffer, true,
          this->dataPtr->image.Data<unsigned char>());
      filledImgData = true;

      // publish
      {
        this->AddSequence(this->dataPtr->pointMsg.mutable_header(), "pointMsg");
        GZ_PROFILE("RgbdCameraSensor::Update Publish point cloud");
        this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
      }
    }

    // publish the 2d image message
    if (this->HasColorConnections())
    {
      if (!filledImgData)
      {
        GZ_PROFILE("RgbdCameraSensor::Update Fill RGB Image");
        // extract image data from point cloud data
        this->dataPtr->pointsUtil.RGBFromPointCloud(
            this->dataPtr->image.Data<unsigned char>(),
            this->dataPtr->pointCloudBuffer,
            width, height);
      }

      unsigned char *data = this->dataPtr->image.Data<unsigned char>();

      gz::msgs::Image msg;
      msg.set_width(width);
      msg.set_height(height);
      msg.set_step(width * gz::rendering::PixelUtil::BytesPerPixel(
          gz::rendering::PF_R8G8B8));
      msg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
      *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
      auto frame = msg.mutable_header()->add_data();
      frame->set_key("frame_id");
      frame->add_value(this->OpticalFrameId());
      msg.set_data(data, gz::rendering::PixelUtil::MemorySize(
          gz::rendering::PF_R8G8B8, width, height));

      // publish the image message
      {
      this->AddSequence(msg.mutable_header(), "rgbdImage");
        GZ_PROFILE("RgbdCameraSensor::Update Publish RGB image");
      this->dataPtr->imagePub.Publish(msg);
      }
    }
  }

  return true;
}

//////////////////////////////////////////////////
unsigned int Ros2RgbdCamera::ImageWidth() const
{
  return this->dataPtr->depthCamera->ImageWidth();
}

//////////////////////////////////////////////////
unsigned int Ros2RgbdCamera::ImageHeight() const
{
  return this->dataPtr->depthCamera->ImageHeight();
}

//////////////////////////////////////////////////
bool Ros2RgbdCamera::HasConnections() const
{
  return this->HasColorConnections() || this->HasDepthConnections() ||
         this->HasPointConnections() || this->HasInfoConnections();
}

//////////////////////////////////////////////////
bool Ros2RgbdCamera::HasColorConnections() const
{
  return this->dataPtr->imagePub && this->dataPtr->imagePub.HasConnections();
}

//////////////////////////////////////////////////
bool Ros2RgbdCamera::HasDepthConnections() const
{
  return this->dataPtr->depthPub && this->dataPtr->depthPub.HasConnections();
}

//////////////////////////////////////////////////
bool Ros2RgbdCamera::HasPointConnections() const
{
  return this->dataPtr->publishPointCloud && this->dataPtr->pointPub &&
         this->dataPtr->pointPub.HasConnections();
}

}  // namespace custom

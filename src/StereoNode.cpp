/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2016, Kevin Hallenbeck
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin Hallenbeck nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ueye/StereoNode.h>

namespace ueye
{

StereoNode::StereoNode(ros::NodeHandle node, ros::NodeHandle priv_nh) :
    srv_(priv_nh), it_(node)
{
  visualize_ = false;
  running_ = false;
  configured_ = false;
  force_streaming_ = false;
  auto_exposure_ = false;
  auto_gain_ = false;
  trigger_mode_ = zoom_ = -1;
  l_stamp_ = r_stamp_ = ros::Time(0);
  l_exposure_ = 0;
  r_exposure_ = 0;
  l_exposure_new_ = 0;
  r_exposure_new_ = 0;
  //l_firstPPScontrolValueNeeded_ = true;
  //r_firstPPScontrolValueNeeded_ = true;
  leftPpsCount = 0;
  rightPpsCount = 0;
  timeout = boost::posix_time::milliseconds(100);

  // Check for a valid uEye installation and supported version
  const char *version;
  int major, minor, build;
  if (l_cam_.checkVersion(major, minor, build, version)) {
    ROS_INFO("Loaded uEye SDK %s.", version);
  } else {
    ROS_WARN("Loaded uEye SDK %d.%d.%d. Expecting %s.", major, minor, build, version);
  }

  // Make sure there is at least one camera available
  int num_cameras = l_cam_.getNumberOfCameras();
  if (num_cameras > 0) {
    if (num_cameras == 1) {
      ROS_ERROR("Found 1 uEye camera.");
      ros::shutdown();
    } else {
      ROS_INFO("Found %d uEye cameras.", num_cameras);
    }
  } else {
    ROS_ERROR("Found 0 uEye cameras.");
    ros::shutdown();
    return;
  }

  // Service call for setting calibration.
  l_srv_cam_info_ = node.advertiseService("left/set_camera_info", &StereoNode::setCameraInfoL, this);
  r_srv_cam_info_ = node.advertiseService("right/set_camera_info", &StereoNode::setCameraInfoR, this);

  // Special publisher for images to support compression
  l_pub_stream_ = it_.advertiseCamera("left/image_raw", 0);
  r_pub_stream_ = it_.advertiseCamera("right/image_raw", 0);
  
  // New added publishers by Petri
  // Special publisher for camera exposure
  l_pub_extras_ = node.advertise<ueye::extras>("/camera/left/extras", 0);
  r_pub_extras_ = node.advertise<ueye::extras>("/camera/right/extras", 0);

  // Open cameras with either serialNo, deviceId, or cameraId
  int id = 0;
  std::string serial_str;
  priv_nh.getParam("lSerialNo", serial_str);
  unsigned int serial = std::stol(serial_str);
  if (serial) {
    if (!l_cam_.openCameraSerNo(serial)) {
      ROS_ERROR("Failed to open uEye camera with serialNo: %u.", serial);
      ros::shutdown();
      return;
    }
  } else if (priv_nh.getParam("lDeviceId", id)) {
    if (!l_cam_.openCameraDevId(id)) {
      ROS_ERROR("Failed to open uEye camera with deviceId: %u.", id);
      ros::shutdown();
      return;
    }
  } else {
    priv_nh.getParam("lCameraId", id);
    if (!l_cam_.openCameraCamId(id)) {
      ROS_ERROR("Failed to open uEye camera with cameraId: %u.", id);
      ros::shutdown();
      return;
    }
  }
  ROS_INFO("Left  camera: %s %u", l_cam_.getCameraName(), l_cam_.getCameraSerialNo());
  id = 0;
  serial_str = "0";
  priv_nh.getParam("rSerialNo", serial_str);
  serial = std::stol(serial_str);
  if (id) {
    if (!r_cam_.openCameraSerNo(id)) {
      ROS_ERROR("Failed to open uEye camera with serialNo: %u.", id);
      ros::shutdown();
      return;
    }
  } else if (priv_nh.getParam("rDeviceId", id)) {
    if (!r_cam_.openCameraDevId(id)) {
      ROS_ERROR("Failed to open uEye camera with deviceId: %u.", id);
      ros::shutdown();
      return;
    }
  } else {
    priv_nh.getParam("rCameraId", id);
    if (!r_cam_.openCameraCamId(id)) {
      ROS_ERROR("Failed to open uEye camera with cameraId: %u.", id);
      ros::shutdown();
      return;
    }
  }
  ROS_INFO("Right camera: %s %u", r_cam_.getCameraName(), r_cam_.getCameraSerialNo());

  // Disable trigger delays
  l_cam_.setTriggerDelay(0);
  r_cam_.setTriggerDelay(0);

  timer_force_trigger_ = node.createTimer(ros::Duration(1.0), &StereoNode::timerForceTrigger, this);
  timer_force_trigger_.stop();

  // Setup Dynamic Reconfigure
  dynamic_reconfigure::Server<stereoConfig>::CallbackType f = boost::bind(&StereoNode::reconfig, this, _1, _2);
  srv_.setCallback(f);	//start dynamic reconfigure

  // Set up Timer
  timer_ = node.createTimer(ros::Duration(0.5), &StereoNode::timerCallback, this);
}

StereoNode::~StereoNode()
{
  closeCamera();
}

void StereoNode::handlePath(std::string &path)
{
  // Set default path if not present
  if (path.length() == 0) {
    path = ros::package::getPath("ueye");
  }

  // Remove trailing "/" from folder path
  unsigned int length = path.length();
  if (length > 0) {
    if (path.c_str()[length - 1] == '/') {
      path = path.substr(0, length - 1);
    }
  }
  config_path_ = path;
}
void StereoNode::reconfigCam(stereoConfig &config, uint32_t level, Camera &cam)
{
  // Color Mode
  uEyeColor color;
  switch (config.color) {
    default:
    case mono_COLOR_MONO8:
      color = MONO8;
      break;
    case mono_COLOR_MONO16:
      color = MONO16;
      break;
    case mono_COLOR_YUV:
      color = YUV;
      break;
    case mono_COLOR_YCbCr:
      color = YCbCr;
      break;
    case mono_COLOR_BGR5:
      color = BGR5;
      break;
    case mono_COLOR_BGR565:
      color = BGR565;
      break;
    case mono_COLOR_BGR8:
      color = BGR8;
      break;
    case mono_COLOR_BGRA8:
      color = BGRA8;
      break;
    case mono_COLOR_BGRY8:
      color = BGRY8;
      break;
    case mono_COLOR_RGB8:
      color = RGB8;
      break;
    case mono_COLOR_RGBA8:
      color = RGBA8;
      break;
    case mono_COLOR_RGBY8:
      color = RGBY8;
      break;
  }
  if (cam.getColorMode() != color) {
    cam.setColorMode(color);
  }

  // Hardware Gamma Correction
  if (cam.getHardwareGamma() != config.hardware_gamma) {
    cam.setHardwareGamma(&config.hardware_gamma);
  }

  // Gain Boost
  if (cam.getGainBoost() != config.gain_boost) {
    cam.setGainBoost(&config.gain_boost);
  }

  // Hardware Gain
  if (cam.getAutoGain() != config.auto_gain) {
    cam.setAutoGain(&config.auto_gain);
  }
  if (!config.auto_gain) {
    cam.setHardwareGain(&config.gain);
  }

  // Zoom
  if (cam.getZoom() != config.zoom) {
    cam.setZoom(&config.zoom);
  }
  
  // Extended Pixel Clock
  if (cam.getExtendedPixelClock() != config.extended_pixel_clock) {
    cam.setExtendedPixelClock(&config.extended_pixel_clock);
  }
  
  // Pixel Clock
  if (cam.getPixelClock() != config.pixel_clock) {
    cam.setPixelClock(&config.pixel_clock);
  }
  
  // Frame Rate
  if (config.trigger == stereo_TGR_SOFTWARE) {
    //In software trigger mode we don't want to set a frame rate
    double d = 2.0;
    cam.setFrameRate(&d);
  } else {
    cam.setFrameRate(&config.frame_rate);
    ROS_INFO("config.trigger %d", config.trigger);
  }

  // Exposure
  if (cam.getAutoExposure() != config.auto_exposure) {
    cam.setAutoExposure(&config.auto_exposure);
  }
  if (cam.getAutoExposureReference() != config.auto_exposure_reference) {
    cam.setAutoExposureReference(&config.auto_exposure_reference);
  }
  if (cam.getAutoExposureMax() != config.auto_exposure_max) {
    bool enable = false;
    cam.setAutoExposure(&enable);
    cam.setAutoExposureMax(&config.auto_exposure_max);
    cam.setAutoExposure(&config.auto_exposure);
  }
  if (cam.getAutoExposureSpeed() != config.auto_exposure_speed) {
    cam.setAutoExposureSpeed(&config.auto_exposure_speed);
  }
  if (cam.getAutoExposureHysteresis() != config.auto_exposure_hysteresis) {
    cam.setAutoExposureHysteresis(&config.auto_exposure_hysteresis);
  }
  if (cam.getAutoExposureSkipFrames() != config.auto_exposure_skip_frames) {
    cam.setAutoExposureSkipFrames(&config.auto_exposure_skip_frames);
  }
  if (!config.auto_exposure) {
    cam.setExposure(&config.exposure_time);
    exposure_time_ = config.exposure_time;
  }
  
  // Mirror
  if (cam.getMirror() != config.mirror) {
    cam.setMirror(&config.mirror);
  }
  
  // Visualize brightness AOI
  if (visualize_ != config.visualize_brightness_AOI)
    visualize_ = config.visualize_brightness_AOI;
}
void StereoNode::reconfig(stereoConfig &config, uint32_t level)
{
  stopCamera();

  force_streaming_ = config.force_streaming;
  handlePath(config.config_path);

  const FlashMode flash_active_mode = config.flash_polarity ? FLASH_FREERUN_ACTIVE_HI : FLASH_FREERUN_ACTIVE_LO;

  if (trigger_mode_ != config.trigger) {
    stopCamera();
    TriggerMode l_trigger, r_trigger;
    FlashMode l_flash = FLASH_OFF;
    FlashMode r_flash = FLASH_OFF;
    switch (config.trigger) {
      case stereo_TGR_SOFTWARE:
      case stereo_TGR_HARDWARE_RISING:
        l_trigger = r_trigger = TRIGGER_LO_HI;
        break;
      case stereo_TGR_HARDWARE_FALLING:
        l_trigger = r_trigger = TRIGGER_HI_LO;
        break;
      case stereo_TGR_L_MASTER_R_RISING:
        l_trigger = TRIGGER_OFF;
        r_trigger = TRIGGER_LO_HI;
        l_flash = flash_active_mode;
        break;
      case stereo_TGR_L_MASTER_R_FALLING:
        l_trigger = TRIGGER_OFF;
        r_trigger = TRIGGER_HI_LO;
        l_flash = flash_active_mode;
        break;
      case stereo_TGR_R_MASTER_L_RISING:
        l_trigger = TRIGGER_LO_HI;
        r_trigger = TRIGGER_OFF;
        r_flash = flash_active_mode;
        break;
      case stereo_TGR_R_MASTER_L_FALLING:
        l_trigger = TRIGGER_HI_LO;
        r_trigger = TRIGGER_OFF;
        r_flash = flash_active_mode;
        break;
      case stereo_TGR_AUTO:
      default:
        config.trigger = stereo_TGR_AUTO;
        l_trigger = r_trigger = TRIGGER_OFF;
        break;
    }
    if (!(l_cam_.setTriggerMode(l_trigger) && r_cam_.setTriggerMode(r_trigger))) {
      ROS_ERROR("Failed to configure triggers");
      l_cam_.setTriggerMode(TRIGGER_OFF);
      r_cam_.setTriggerMode(TRIGGER_OFF);
      config.trigger = stereo_TGR_AUTO;
      l_cam_.setFlash(FLASH_OFF);
      r_cam_.setFlash(FLASH_OFF);
    } else {
      l_cam_.setFlash(l_flash, config.flash_delay, config.flash_duration);
      r_cam_.setFlash(r_flash, config.flash_delay, config.flash_duration);
    }
  }

  // Latch Auto Parameters
  if (auto_gain_ && !config.auto_gain) {
    config.gain = l_cam_.getHardwareGain();
  }
  auto_gain_ = config.auto_gain;
  if (auto_exposure_ && !config.auto_exposure) {
    config.exposure_time = l_cam_.getExposure();
  }
  auto_exposure_ = config.auto_exposure;

  // Left Camera AOI config consistency check
  {
  // Width needs to be dividable by 8 and height needs to be even integer
  if (config.l_AOI_Width % 8 != 0)
  	config.l_AOI_Width -= config.l_AOI_Width % 8;
  if (config.l_AOI_Height % 2 != 0)
  	config.l_AOI_Height += 1;
  // Image borders cannot be outside the physical sensor
  if (config.l_AOI_X > 1936 - config.l_AOI_Width)
  	config.l_AOI_X = 1936 - config.l_AOI_Width;
  if (config.l_AOI_Y > 1216 - config.l_AOI_Height)
  	config.l_AOI_Y = 1216 - config.l_AOI_Height;
  // X needs to be dividable by 8 and Y needs to be even integer
  if (config.l_AOI_X % 8 != 0)
  	config.l_AOI_X -= config.l_AOI_X % 8;
  if (config.l_AOI_Y % 2 != 0)
  	config.l_AOI_Y += 1;
  }
  
  // Right Camera AOI config consistency check
  {
  // Width needs to be dividable by 8 and height needs to be even integer
  if (config.r_AOI_Width % 8 != 0)
  	config.r_AOI_Width -= config.r_AOI_Width % 8;
  if (config.r_AOI_Height % 2 != 0)
  	config.r_AOI_Height += 1;
  // Image borders cannot be outside the physical sensor
  if (config.r_AOI_X > 1936 - config.r_AOI_Width)
  	config.r_AOI_X = 1936 - config.r_AOI_Width;
  if (config.r_AOI_Y > 1216 - config.r_AOI_Height)
  	config.r_AOI_Y = 1216 - config.r_AOI_Height;
  // X needs to be dividable by 8 and Y needs to be even integer
  if (config.r_AOI_X % 8 != 0)
  	config.r_AOI_X -= config.r_AOI_X % 8;
  if (config.r_AOI_Y % 2 != 0)
  	config.r_AOI_Y += 1;
  
  // Save the values for Left and Right AOI and submit to camera
  l_aoi_.s32Width = config.l_AOI_Width;
  l_aoi_.s32Height = config.l_AOI_Height;
  r_aoi_.s32Width = config.r_AOI_Width;
  r_aoi_.s32Height = config.r_AOI_Height;
  switch (config.mirror)
  {
    case NO_MIRROR:
      l_aoi_.s32X = config.l_AOI_X;
      l_aoi_.s32Y = config.l_AOI_Y;
      r_aoi_.s32X = config.r_AOI_X;
      r_aoi_.s32Y = config.r_AOI_Y;
      break;
    case MIRROR_BOTH:
      l_aoi_.s32X = 1936 - config.l_AOI_X - config.l_AOI_Width;
      l_aoi_.s32Y = 1216 - config.l_AOI_Y - config.l_AOI_Height;
      r_aoi_.s32X = 1936 - config.r_AOI_X - config.r_AOI_Width;
      r_aoi_.s32Y = 1216 - config.r_AOI_Y - config.r_AOI_Height;
      break;
    case MIRROR_UP_DOWN:
      l_aoi_.s32X = config.l_AOI_X;
      l_aoi_.s32Y = 1216 - config.l_AOI_Y - config.l_AOI_Height;
      r_aoi_.s32X = config.r_AOI_X;
      r_aoi_.s32Y = 1216 - config.r_AOI_Y - config.r_AOI_Height;
      break;
    case MIRROR_LEFT_RIGHT:
      l_aoi_.s32X = 1936 - config.l_AOI_X - config.l_AOI_Width;
      l_aoi_.s32Y = config.l_AOI_Y;
      r_aoi_.s32X = 1936 - config.r_AOI_X - config.r_AOI_Width;
      r_aoi_.s32Y = config.r_AOI_Y;
      break;
  }
  
  l_cam_.setAOI(l_aoi_);
  r_cam_.setAOI(r_aoi_);
  }
  
  // Left Camera Auto Brightness AOI
  // Specified from the mirrored images area of interest!!!!!
  {
  // Check the validity of the config values and correct accordingly:
  // Width needs to be dividable by 8 and height needs to be even integer
  if (config.l_brightness_AOI_Width % 8 != 0)
  	config.l_brightness_AOI_Width -= config.l_brightness_AOI_Width % 8;
  if (config.l_brightness_AOI_Height % 2 != 0)
  	config.l_brightness_AOI_Height += 1;
  // Brightness AOI borders cannot be outside the physical sensor
  if (config.l_brightness_AOI_X > 1936 - config.l_brightness_AOI_Width)
  	config.l_brightness_AOI_X = 1936 - config.l_brightness_AOI_Width;
  if (config.l_brightness_AOI_Y > 1216 - config.l_brightness_AOI_Height)
  	config.l_brightness_AOI_Y = 1216 - config.l_brightness_AOI_Height;
  // X needs to be dividable by 8 and Y needs to be even integer
  if (config.l_brightness_AOI_X % 8 != 0)
  	config.l_brightness_AOI_X -= config.l_brightness_AOI_X % 8;
  if (config.l_brightness_AOI_Y % 2 != 0)
  	config.l_brightness_AOI_Y += 1;
  // Brightness AOI cannot be larger than the AOI
  if (config.l_brightness_AOI_Width > config.l_AOI_Width)
    config.l_brightness_AOI_Width = config.l_AOI_Width;
  if (config.l_brightness_AOI_Height > config.l_AOI_Height)
    config.l_brightness_AOI_Height = config.l_AOI_Height;
  // Brightness AOI cannot be located outside the AOI
  if ((config.l_brightness_AOI_X + config.l_brightness_AOI_Width) > config.l_AOI_Width)
    config.l_brightness_AOI_X = config.l_AOI_Width - config.l_brightness_AOI_Width;
  if ((config.l_brightness_AOI_Y + config.l_brightness_AOI_Height) > config.l_AOI_Height)
    config.l_brightness_AOI_Y = config.l_AOI_Height - config.l_brightness_AOI_Height;
  
  l_brightness_aoi_.s32Width = config.l_brightness_AOI_Width;
  l_brightness_aoi_.s32Height = config.l_brightness_AOI_Height;
  l_brightness_aoi_.s32X = l_aoi_.s32X + config.l_brightness_AOI_X;
  l_brightness_aoi_.s32Y = l_aoi_.s32Y + config.l_brightness_AOI_Y;
  
  l_cam_.setAutoBrightnessAOI(l_brightness_aoi_);
  }
  
  l_visualize_brightness_aoi_.s32Width = config.l_brightness_AOI_Width;
  l_visualize_brightness_aoi_.s32Height = config.l_brightness_AOI_Height;
  l_visualize_brightness_aoi_.s32X = config.l_brightness_AOI_X;
  l_visualize_brightness_aoi_.s32Y = config.l_brightness_AOI_Y;
  
  // Right Camera Auto Brightness AOI
  // This is specified from the mirrored images area of interest!!!!!
  {
  // Check the validity of the config values and correct accordingly:
  // Width needs to be dividable by 8 and height needs to be even integer
  if (config.r_brightness_AOI_Width % 8 != 0)
  	config.r_brightness_AOI_Width -= config.r_brightness_AOI_Width % 8;
  if (config.r_brightness_AOI_Height % 2 != 0)
  	config.r_brightness_AOI_Height += 1;
  // Brightness AOI borders cannot be outside the physical sensor
  if (config.r_brightness_AOI_X > 1936 - config.r_brightness_AOI_Width)
  	config.r_brightness_AOI_X = 1936 - config.r_brightness_AOI_Width;
  if (config.r_brightness_AOI_Y > 1216 - config.r_brightness_AOI_Height)
  	config.r_brightness_AOI_Y = 1216 - config.r_brightness_AOI_Height;
  // X needs to be dividable by 8 and Y needs to be even integer
  if (config.r_brightness_AOI_X % 8 != 0)
  	config.r_brightness_AOI_X -= config.r_brightness_AOI_X % 8;
  if (config.r_brightness_AOI_Y % 2 != 0)
  	config.r_brightness_AOI_Y += 1;
  // Brightness AOI cannot be larger than the AOI
  if (config.r_brightness_AOI_Width > config.r_AOI_Width)
    config.r_brightness_AOI_Width = config.r_AOI_Width;
  if (config.r_brightness_AOI_Height > config.r_AOI_Height)
    config.r_brightness_AOI_Height = config.r_AOI_Height;
  // Brightness AOI cannot be located outside the AOI
  if ((config.r_brightness_AOI_X + config.r_brightness_AOI_Width) > config.r_AOI_Width)
    config.r_brightness_AOI_X = config.r_AOI_Width - config.r_brightness_AOI_Width;
  if ((config.r_brightness_AOI_Y + config.r_brightness_AOI_Height) > config.r_AOI_Height)
    config.r_brightness_AOI_Y = config.r_AOI_Height - config.r_brightness_AOI_Height;
  
  r_brightness_aoi_.s32Width = config.r_brightness_AOI_Width;
  r_brightness_aoi_.s32Height = config.r_brightness_AOI_Height;
  r_brightness_aoi_.s32X = r_aoi_.s32X + config.r_brightness_AOI_X;
  r_brightness_aoi_.s32Y = r_aoi_.s32Y + config.r_brightness_AOI_Y;
  
  r_cam_.setAutoBrightnessAOI(r_brightness_aoi_);
  }
  
  r_visualize_brightness_aoi_.s32Width = config.r_brightness_AOI_Width;
  r_visualize_brightness_aoi_.s32Height = config.r_brightness_AOI_Height;
  r_visualize_brightness_aoi_.s32X = config.r_brightness_AOI_X;
  r_visualize_brightness_aoi_.s32Y = config.r_brightness_AOI_Y;

  // Config the common parameters for both cameras
  reconfigCam(config, level, l_cam_);
  reconfigCam(config, level, r_cam_);

  trigger_mode_ = config.trigger;
  if (trigger_mode_ == stereo_TGR_SOFTWARE) {
    timer_force_trigger_.setPeriod(ros::Duration(1 / config.frame_rate));
  }

  if (zoom_ != config.zoom) {
    zoom_ = config.zoom;
    loadIntrinsics(l_cam_, l_msg_camera_info_);
    loadIntrinsics(r_cam_, r_msg_camera_info_);
  }
  
  // Set binning 2x
  if (binning_ != config.software_binning)
  {
    binning_ = config.software_binning;
    loadIntrinsics(l_cam_, l_msg_camera_info_);
    loadIntrinsics(r_cam_, r_msg_camera_info_);
    
    l_msg_camera_info_.height = l_msg_camera_info_.height/2;
    l_msg_camera_info_.width = l_msg_camera_info_.width/2;
    l_msg_camera_info_.roi.height = l_msg_camera_info_.roi.height/2;
    l_msg_camera_info_.roi.width = l_msg_camera_info_.roi.width/2;
    
    r_msg_camera_info_.height = r_msg_camera_info_.height/2;
    r_msg_camera_info_.width = r_msg_camera_info_.width/2;
    r_msg_camera_info_.roi.height = r_msg_camera_info_.roi.height/2;
    r_msg_camera_info_.roi.width = r_msg_camera_info_.roi.width/2;
  }

  l_msg_camera_info_.header.frame_id = config.l_frame_id;
  r_msg_camera_info_.header.frame_id = config.r_frame_id;
  
  configured_ = true;
  
  startCamera();
}

void StereoNode::timerCallback(const ros::TimerEvent& event)
{
  if (force_streaming_ || (l_pub_stream_.getNumSubscribers() > 0) || (r_pub_stream_.getNumSubscribers() > 0)) {
    startCamera();
  } else {
    stopCamera();
  }
}
void StereoNode::timerForceTrigger(const ros::TimerEvent& event)
{
  if (trigger_mode_ == stereo_TGR_SOFTWARE) {
    bool success = true;
    success &= l_cam_.forceTrigger();
    success &= r_cam_.forceTrigger();
    if (!success) {
      ROS_WARN("Failed to force trigger");
    }
  }
}

// Support camera calibration requests
// http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration
bool StereoNode::setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp,
                               Camera& cam, sensor_msgs::CameraInfo &msg_info)
{
  ROS_INFO("New camera info received");
  sensor_msgs::CameraInfo &info = req.camera_info;
  info.header.frame_id = msg_info.header.frame_id;

  // Sanity check: the image dimensions should match the resolution of the sensor.
  unsigned int height = cam.getHeight();
  unsigned int width = cam.getWidth();

  if (info.width != width || info.height != height) {
    rsp.success = false;
    rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
                                        "setting, camera running at resolution %ix%i.") % info.width % info.height
        % width % height).str();
    ROS_ERROR("%s", rsp.status_message.c_str());
    return true;
  }

  std::string camname = cam.getCameraName();
  std::stringstream ini_stream;
  if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, camname, info)) {
    rsp.status_message = "Error formatting camera_info for storage.";
    rsp.success = false;
  } else {
    std::string ini = ini_stream.str();
    std::fstream param_file;
    std::string filename = config_path_ + "/" + configFileName(cam, binning_);
    param_file.open(filename.c_str(), std::ios::in | std::ios::out | std::ios::trunc);

    if (param_file.is_open()) {
      param_file << ini.c_str();
      param_file.close();

      msg_info = info;
      rsp.success = true;
    } else {
      rsp.success = false;
      rsp.status_message = "file write failed";
    }
  }
  if (!rsp.success) {
    ROS_ERROR("%s", rsp.status_message.c_str());
  }
  return true;
}
bool StereoNode::setCameraInfoL(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp)
{
  return setCameraInfo(req, rsp, l_cam_, l_msg_camera_info_);
}
bool StereoNode::setCameraInfoR(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp)
{
  return setCameraInfo(req, rsp, r_cam_, r_msg_camera_info_);
}

// Try to load previously saved camera calibration from a file.
void StereoNode::loadIntrinsics(Camera &cam, sensor_msgs::CameraInfo &msg_info)
{
  char buffer[12800];

  std::string MyPath = config_path_ + "/" + configFileName(cam, binning_);
  std::fstream param_file;
  param_file.open(MyPath.c_str(), std::ios::in);

  if (param_file.is_open()) {
    param_file.read(buffer, 12800);
    param_file.close();
  }

  // Parse calibration file
  std::string camera_name;
  if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, msg_info)) {
    ROS_INFO("Calibration : %s %u", camera_name.c_str(), cam.getCameraSerialNo());
  } else {
    ROS_WARN("Failed to load intrinsics for camera from file");
  }
}

// Add properties to image message
sensor_msgs::ImagePtr StereoNode::processFrame(const char *frame, size_t size, const Camera &cam,
                                               sensor_msgs::CameraInfoPtr &info, sensor_msgs::CameraInfo &msg_info)
{
  if (binning_)
  {
    msg_info.height = cam.getHeight() / 2;
    msg_info.width = cam.getWidth() / 2;
    msg_info.roi.width = cam.getROIWidth() / 2;
    msg_info.roi.height = cam.getROIHeight() / 2;
    msg_info.roi.x_offset = cam.getROI_X() / 2;
    msg_info.roi.y_offset = cam.getROI_Y() / 2;
  }
  else
  {
    msg_info.height = cam.getHeight();
    msg_info.width = cam.getWidth();
    msg_info.roi.width = cam.getROIWidth();
    msg_info.roi.height = cam.getROIHeight();
    msg_info.roi.x_offset = cam.getROI_X();
    msg_info.roi.y_offset = cam.getROI_Y();
  }

  sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image());
  msg_image->header = msg_info.header;
  msg_image->height = cam.getROIHeight();
  msg_image->width = cam.getROIWidth();
  msg_image->encoding = Camera::colorModeToString(cam.getColorMode());
  msg_image->is_bigendian = false;
  msg_image->step = size / msg_image->height;
  //ROS_INFO("Step: %d, size: %d, height: %d", msg_image->step, size, msg_image->height);
  msg_image->data.resize(size);
  memcpy(msg_image->data.data(), frame, size);
  sensor_msgs::CameraInfoPtr msg(new sensor_msgs::CameraInfo(msg_info));
  info = msg;
  
  return msg_image;
}

void StereoNode::DrawBrightnessAOI_L(sensor_msgs::ImagePtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  // Draw the rectangle
  cv::rectangle(cv_ptr->image, cv::Rect(l_visualize_brightness_aoi_.s32X, l_visualize_brightness_aoi_.s32Y, l_visualize_brightness_aoi_.s32Width, l_visualize_brightness_aoi_.s32Height), cv::Scalar(255,0,0), 4);
  msg = cv_ptr->toImageMsg();
}

void StereoNode::DrawBrightnessAOI_R(sensor_msgs::ImagePtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  // Draw the rectangle
  cv::rectangle(cv_ptr->image, cv::Rect(r_visualize_brightness_aoi_.s32X, r_visualize_brightness_aoi_.s32Y, r_visualize_brightness_aoi_.s32Width, r_visualize_brightness_aoi_.s32Height), cv::Scalar(255,0,0), 4);
  msg = cv_ptr->toImageMsg();
}

void StereoNode::BinImg(sensor_msgs::ImagePtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  // Draw the rectangle
  cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), 0.5, 0.5, CV_INTER_AREA);
  msg = cv_ptr->toImageMsg();
}

// Timestamp and publish the image. Called by the streaming thread.
void StereoNode::publishImageL(const char *frame, size_t size, ueye::extras& extras)
{
  l_pub_extras_.publish( extras );
  
  l_msg_camera_info_.header.stamp = extras.header.stamp;
  sensor_msgs::CameraInfoPtr info;
  sensor_msgs::ImagePtr img = processFrame(frame, size, l_cam_, info, l_msg_camera_info_);
  
  if (binning_)
    BinImg(img);
  if (visualize_)
    DrawBrightnessAOI_L(img);

  // Publish Extras and Image
  l_pub_stream_.publish(img, info);
}
void StereoNode::publishImageR(const char *frame, size_t size, ueye::extras& extras)
{
  r_pub_extras_.publish( extras );
  
  r_msg_camera_info_.header.stamp = extras.header.stamp;
  sensor_msgs::CameraInfoPtr info;
  sensor_msgs::ImagePtr img = processFrame(frame, size, r_cam_, info, r_msg_camera_info_);
  
  if (binning_)
    BinImg(img);
  if (visualize_)
    DrawBrightnessAOI_R(img);
  
  // Publish Image
  r_pub_stream_.publish(img, info);
}

void StereoNode::startCamera()
{
  if (running_ || !configured_)
    return;
  // These are callback functions that are not used at the moment
  l_cam_.startVideoCapture(boost::bind(&StereoNode::publishImageL, this, _1, _2, _3));
  r_cam_.startVideoCapture(boost::bind(&StereoNode::publishImageR, this, _1, _2, _3));
  stop_publish_ = false;
  l_stamp_ready = false;
  l_img_info_ready = false;
  r_stamp_ready = false;
  r_img_info_ready = false;
  
  timer_force_trigger_.start();
  ROS_INFO("Started video stream.");
  running_ = true;
}

void StereoNode::stopCamera()
{
  timer_force_trigger_.stop();
  if (!running_)
    return;
  ROS_INFO("Stopping video stream.");
  stop_publish_ = true;
  //l_thread_.join();
  //r_thread_.join();
  l_cam_.stopVideoCapture();
  r_cam_.stopVideoCapture();
  leftPpsCount = 0;
  rightPpsCount = 0;
  ROS_INFO("Stopped video stream.");
  running_ = false;
}

void StereoNode::closeCamera()
{
  stopCamera();
  r_cam_.closeCamera();
  l_cam_.closeCamera();
}

} // namespace ueye

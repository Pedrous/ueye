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

#include <ueye/CameraNode.h>

namespace ueye
{

CameraNode::CameraNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh) :
    srv_(priv_nh), it_(node)
{
  cal_exp_ = false;
  cal_exp_latch_ = false;
  binning_ = false;
  visualize_ = false;
  running_ = false;
  configured_ = false;
  force_streaming_ = false;
  auto_exposure_ = false;
  auto_gain_ = false;
  trigger_mode_ = zoom_ = -1;
  exposure_calib_.exp_time = 0.0;
  //stamp_ ros::Time(0);
  exposure_ = 0;
  exposure_new_ = 0;
  PpsCount = 0;

  // Check for a valid uEye installation and supported version
  const char *version;
  int major, minor, build;
  if (cam_.checkVersion(major, minor, build, version)) {
    ROS_INFO("Loaded uEye SDK %s.", version);
  } else {
    ROS_WARN("Loaded uEye SDK %d.%d.%d. Expecting %s.", major, minor, build, version);
  }

  // Make sure there is at least one camera available
  int num_cameras = cam_.getNumberOfCameras();
  if (num_cameras > 0) {
    if (num_cameras == 1) {
      ROS_INFO("Found 1 uEye camera.");
    } else {
      ROS_INFO("Found %d uEye cameras.", num_cameras);
    }
  } else {
    ROS_ERROR("Found 0 uEye cameras.");
    ros::shutdown();
    return;
  }

  // Open camera with either serialNo, deviceId, or cameraId
  int id = 0;
  std::string serial_str;
  priv_nh.getParam("serialNo", serial_str);
  unsigned int serial = std::stol(serial_str);
  if (serial) {
    if (!cam_.openCameraSerNo(serial)) {
      ROS_ERROR("Failed to open uEye camera with serialNo: %u.", serial);
      ros::shutdown();
      return;
    }
  } else if (priv_nh.getParam("deviceId", id)) {
    if (!cam_.openCameraDevId(id)) {
      ROS_ERROR("Failed to open uEye camera with deviceId: %u.", id);
      ros::shutdown();
      return;
    }
  } else {
    priv_nh.getParam("cameraId", id);
    if (!cam_.openCameraCamId(id)) {
      ROS_ERROR("Failed to open uEye camera with cameraId: %u.", id);
      ros::shutdown();
      return;
    }
  }
  ROS_INFO("Opened camera %s %u", cam_.getCameraName(), cam_.getCameraSerialNo());

  // Service call for setting calibration.
  srv_cam_info_ = node.advertiseService("set_camera_info", &CameraNode::setCameraInfo, this);

  // Special publisher for images to support compression
  pub_stream_ = it_.advertiseCamera("image_raw", 0);
  
  // New added publishers by Petri
  // Special publisher for camera exposure
  pub_exposure_ = node.advertise<ueye::exposure>("exposure", 0);
  // Special publisher for camera exposure
  pub_extras_ = node.advertise<ueye::extras>("extras", 0);

  // Set up Timer
  timer_ = node.createTimer(ros::Duration(1 / 5.0), &CameraNode::timerCallback, this);
  
  // Setup Dynamic Reconfigure
  dynamic_reconfigure::Server<monoConfig>::CallbackType f = boost::bind(&CameraNode::reconfig, this, _1, _2);
  srv_.setCallback(f);	//start dynamic reconfigure
}

CameraNode::~CameraNode()
{
  closeCamera();
}

void CameraNode::handlePath(std::string &path)
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
void CameraNode::reconfig(monoConfig &config, uint32_t level)
{
  stopCamera();
  //std::cout << "reconfig Entered" << std::endl;
  
  force_streaming_ = config.force_streaming;
  handlePath(config.config_path);

  // Trigger
  if (trigger_mode_ != config.trigger) {
    stopCamera();
    TriggerMode mode;
    switch (config.trigger) {
      case mono_TGR_HARDWARE_RISING:
        mode = TRIGGER_LO_HI;
        break;
      case mono_TGR_HARDWARE_FALLING:
        mode = TRIGGER_HI_LO;
        break;
      case mono_TGR_AUTO:
      default:
        mode = TRIGGER_OFF;
        break;
    }
    if (!cam_.setTriggerMode(mode)) {
      cam_.setTriggerMode(TRIGGER_OFF);
      config.trigger = mono_TGR_AUTO;
    }
  }
  trigger_mode_ = config.trigger;

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
  if (cam_.getColorMode() != color) {
    cam_.setColorMode(color);
  }

  // Latch Auto Parameters
  if (auto_gain_ && !config.auto_gain) {
    config.gain = cam_.getHardwareGain();
  }
  auto_gain_ = config.auto_gain;
  if (auto_exposure_ && !config.auto_exposure) {
    config.exposure_time = cam_.getExposure();
  }
  auto_exposure_ = config.auto_exposure;

  // Hardware Gamma Correction
  if (cam_.getHardwareGamma() != config.hardware_gamma) {
    cam_.setHardwareGamma(&config.hardware_gamma);
  }

  // Gain Boost
  if (cam_.getGainBoost() != config.gain_boost) {
    cam_.setGainBoost(&config.gain_boost);
  }

  // Hardware Gain
  if (cam_.getAutoGain() != config.auto_gain) {
    cam_.setAutoGain(&config.auto_gain);
  }
  if (!config.auto_gain) {
    cam_.setHardwareGain(&config.gain);
  }
  
  // Subsampling or Binning
  /*if (config.hardware_subsampling != subsampling_) {
    subsampling_ = config.hardware_subsampling;
    binning_ = 1;
    config.software_binning = 1;
    zoom_ = subsampling_;
  }
  else if (config.software_binning != binning_) {
    binning_ = config.software_binning;
    subsampling_ = 1;
    config.hardware_subsampling = 1;
    zoom_ = binning_;
  }*/
  
  // Zoom
  if (cam_.getZoom() != config.zoom) {
    cam_.setZoom(&config.zoom);
  }
  /*else if (cam_.getZoom() != config.zoom) {
    int binning = 2;
    cam_.setZoom(&binning);
    if (binning == 1)
      config.binning = false
  }*/
  
  // Mirror
  if (cam_.getMirror() != config.mirror) {
    cam_.setMirror(&config.mirror);
  }

  // Extended Pixel Clock
  if (cam_.getExtendedPixelClock() != config.extended_pixel_clock) {
    cam_.setExtendedPixelClock(&config.extended_pixel_clock);
  }

  // Pixel Clock
  if (cam_.getPixelClock() != config.pixel_clock) {
    cam_.setPixelClock(&config.pixel_clock);
  }
  
  // AOI
  {
  // Width needs to be dividable by 8 and height needs to be even integer
  if (config.AOI_Width % 8 != 0)
  	config.AOI_Width -= config.AOI_Width % 8;
  if (config.AOI_Height % 2 != 0)
  	config.AOI_Height += 1;
  // Image borders cannot be outside the physical sensor
  if (config.AOI_X > 1936 - config.AOI_Width)
  	config.AOI_X = 1936 - config.AOI_Width;
  if (config.AOI_Y > 1216 - config.AOI_Height)
  	config.AOI_Y = 1216 - config.AOI_Height;
  // X needs to be dividable by 8 and Y needs to be even integer
  if (config.AOI_X % 8 != 0)
  	config.AOI_X -= config.AOI_X % 8;
  if (config.AOI_Y % 2 != 0)
  	config.AOI_Y += 1;
  
  // Save the values and submit to camera
  aoi_.s32Width = config.AOI_Width;
  aoi_.s32Height = config.AOI_Height;
  switch (config.mirror)
  {
    case NO_MIRROR:
      aoi_.s32X = config.AOI_X;
      aoi_.s32Y = config.AOI_Y;
      break;
    case MIRROR_BOTH:
      aoi_.s32X = 1936 - config.AOI_X - config.AOI_Width;
      aoi_.s32Y = 1216 - config.AOI_Y - config.AOI_Height;
      break;
    case MIRROR_UP_DOWN:
      aoi_.s32X = config.AOI_X;
      aoi_.s32Y = 1216 - config.AOI_Y - config.AOI_Height;
      break;
    case MIRROR_LEFT_RIGHT:
      aoi_.s32X = 1936 - config.AOI_X - config.AOI_Width;
      aoi_.s32Y = config.AOI_Y;
      break;
  }
  
  cam_.setAOI(aoi_);
  }
  
  // Auto Brightness AOI
  // This is specified from the mirrored images area of interest!!!!!
  {
  // Check the validity of the config values and correct accordingly:
  // Width needs to be dividable by 8 and height needs to be even integer
  if (config.brightness_AOI_Width % 8 != 0)
  	config.brightness_AOI_Width -= config.brightness_AOI_Width % 8;
  if (config.brightness_AOI_Height % 2 != 0)
  	config.brightness_AOI_Height += 1;
  // Brightness AOI borders cannot be outside the physical sensor
  if (config.brightness_AOI_X > 1936 - config.brightness_AOI_Width)
  	config.brightness_AOI_X = 1936 - config.brightness_AOI_Width;
  if (config.brightness_AOI_Y > 1216 - config.brightness_AOI_Height)
  	config.brightness_AOI_Y = 1216 - config.brightness_AOI_Height;
  // X needs to be dividable by 8 and Y needs to be even integer
  if (config.brightness_AOI_X % 8 != 0)
  	config.brightness_AOI_X -= config.brightness_AOI_X % 8;
  if (config.brightness_AOI_Y % 2 != 0)
  	config.brightness_AOI_Y += 1;
  // Brightness AOI cannot be larger than the AOI
  if (config.brightness_AOI_Width > config.AOI_Width)
    config.brightness_AOI_Width = config.AOI_Width;
  if (config.brightness_AOI_Height > config.AOI_Height)
    config.brightness_AOI_Height = config.AOI_Height;
  // Brightness AOI cannot be located outside the AOI
  if ((config.brightness_AOI_X + config.brightness_AOI_Width) > config.AOI_Width)
    config.brightness_AOI_X = config.AOI_Width - config.brightness_AOI_Width;
  if ((config.brightness_AOI_Y + config.brightness_AOI_Height) > config.AOI_Height)
    config.brightness_AOI_Y = config.AOI_Height - config.brightness_AOI_Height;
  
  brightness_aoi_.s32Width = config.brightness_AOI_Width;
  brightness_aoi_.s32Height = config.brightness_AOI_Height;
  brightness_aoi_.s32X = aoi_.s32X + config.brightness_AOI_X;
  brightness_aoi_.s32Y = aoi_.s32Y + config.brightness_AOI_Y;
  
  cam_.setAutoBrightnessAOI(brightness_aoi_);
  }
  
  visualize_brightness_aoi_.s32Width = config.brightness_AOI_Width;
  visualize_brightness_aoi_.s32Height = config.brightness_AOI_Height;
  visualize_brightness_aoi_.s32X = config.brightness_AOI_X;
  visualize_brightness_aoi_.s32Y = config.brightness_AOI_Y;
  
  // Visualize brightness AOI
  if (visualize_ != config.visualize_brightness_AOI)
    visualize_ = config.visualize_brightness_AOI;
    
  // Set binning 2x
  if (binning_ != config.software_binning)
  {
    binning_ = config.software_binning;
    loadIntrinsics();
  }

  // Frame Rate
  cam_.setFrameRate(&config.frame_rate);

  // Auto Exposure Values
  if (cam_.getAutoExposure() != config.auto_exposure) {
    cam_.setAutoExposure(&config.auto_exposure);
  }
  if (cam_.getAutoExposureReference() != config.auto_exposure_reference) {
    cam_.setAutoExposureReference(&config.auto_exposure_reference);
  }
  if (cam_.getAutoExposureMax() != config.auto_exposure_max) {
    bool enable = false;
    cam_.setAutoExposure(&enable);
    cam_.setAutoExposureMax(&config.auto_exposure_max);
    cam_.setAutoExposure(&config.auto_exposure);
  }
  if (cam_.getAutoExposureSpeed() != config.auto_exposure_speed) {
    cam_.setAutoExposureSpeed(&config.auto_exposure_speed);
  }
  if (cam_.getAutoExposureHysteresis() != config.auto_exposure_hysteresis) {
    cam_.setAutoExposureHysteresis(&config.auto_exposure_hysteresis);
  }
  if (cam_.getAutoExposureSkipFrames() != config.auto_exposure_skip_frames) {
    cam_.setAutoExposureSkipFrames(&config.auto_exposure_skip_frames);
  }
  
  // Exposure time
  if (!config.auto_exposure) {
    cam_.setExposure(&config.exposure_time);
    exposure_time_ = config.exposure_time;
  }

  // Zoom
  if (zoom_ != config.zoom) {
    zoom_ = config.zoom;
    loadIntrinsics();
  }
  
  // Exposure calibration
  if (config.calibrate_exposure == true)
  {
    config.frame_rate = 50;
    cam_.setFrameRate(&config.frame_rate);
    config.auto_exposure = false;
    cam_.setAutoExposure(&config.auto_exposure);
    config.auto_gain = false;
    cam_.setAutoGain(&config.auto_gain);
    config.exposure_time = 0.018;
    cam_.setExposure(&config.exposure_time);
    config.gain = 100;
    cam_.setHardwareGain(&config.gain);
    
    cal_exp_ = true;
    cal_exp_2_ = false;
    cal_exp_latch_ = true;
    // Print values
    exposure_calib_.exp_time = cam_.getExposure();
    exposure_calib_new_.exp_time = exposure_calib_.exp_time;
    exposure_calib_newest_.exp_time = exposure_calib_.exp_time;
    exp_increment_ = cam_.getExposureIncrement();
    ROS_INFO("EXPOSURE: %f", exposure_calib_.exp_time);
    config.calibrate_exposure = false;
    /*ROS_INFO("EXPOSURE_RANGE_MIN: %f", cam_.getExposureMin());
    ROS_INFO("EXPOSURE_RANGE_MAX: %f", cam_.getExposureMax());
    ROS_INFO("EXPOSURE_RANGE_INC: %f", cam_.getExposureIncrement());*/
    
    // Do not overwrite existing filenames
    int order = 1;
    while (1)
    { 
      std::stringstream name;
      std::string file;
      name << "/home/snowflake/Photo_calibration/ExpCal-" << cam_.getCameraName() << "-" << cam_.getCameraSerialNo() << "-" << order << ".bag";
      file = name.str();
      if( access( file.c_str(), F_OK ) == -1 )
      {
        bag.open(file, rosbag::bagmode::Write);
        break;
      }
      order++;
    }
  }
  
  // Publish exposure time and PPS control values
  if (publish_extras_ != config.publish_extras)
    publish_extras_ = config.publish_extras;
  
  msg_camera_info_.header.frame_id = config.frame_id;
  configured_ = true;
  
  startCamera();
  //std::cout << "reconfig Exited" << std::endl;
}

void CameraNode::timerCallback(const ros::TimerEvent& event)
{
  //std::cout << "timerCallback Entered" << std::endl;
  if ((pub_stream_.getNumSubscribers() > 0) || force_streaming_) {
    startCamera();
  } else if (!cal_exp_){
    stopCamera();
  }
  //std::cout << "timerCallback Exited" << std::endl;
}

// Support camera calibration requests
// http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration
bool CameraNode::setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp)
{
  //std::cout << "setCameraInfo Entered" << std::endl;
  ROS_INFO("New camera info received");
  sensor_msgs::CameraInfo &info = req.camera_info;
  info.header.frame_id = msg_camera_info_.header.frame_id;

  // Sanity check: the image dimensions should match the resolution of the sensor.
  unsigned int height = cam_.getHeight();
  unsigned int width = cam_.getWidth();

  if (info.width != width || info.height != height) {
    rsp.success = false;
    rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
                                        "setting, camera running at resolution %ix%i.") % info.width % info.height
        % width % height).str();
    ROS_ERROR("%s", rsp.status_message.c_str());
    return true;
  }

  std::string camname = cam_.getCameraName();
  std::stringstream ini_stream;
  if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, camname, info)) {
    rsp.status_message = "Error formatting camera_info for storage.";
    rsp.success = false;
  } else {
    std::string ini = ini_stream.str();
    std::fstream param_file;
    std::string filename = config_path_ + "/" + configFileName(cam_, binning_);
    param_file.open(filename.c_str(), std::ios::in | std::ios::out | std::ios::trunc);

    if (param_file.is_open()) {
      param_file << ini.c_str();
      param_file.close();

      msg_camera_info_ = info;
      rsp.success = true;
    } else {
      rsp.success = false;
      rsp.status_message = "file write failed";
    }
  }
  if (!rsp.success) {
    ROS_ERROR("%s", rsp.status_message.c_str());
  }
  //std::cout << "setCameraInfo Exited" << std::endl;
  return true;
}

// Try to load previously saved camera calibration from a file.
void CameraNode::loadIntrinsics()
{
  char buffer[12800];

  std::string MyPath = config_path_ + "/" + configFileName(cam_, binning_);
  std::fstream param_file;
  param_file.open(MyPath.c_str(), std::ios::in);

  if (param_file.is_open()) {
    param_file.read(buffer, 12800);
    param_file.close();
  }

  // Parse calibration file
  std::string camera_name;
  if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, msg_camera_info_)) {
    ROS_INFO("Loaded calibration for camera '%s'", camera_name.c_str());
  } else {
    ROS_WARN("Failed to load intrinsics for camera from file");
  }
}

// Add properties to image message
/*sensor_msgs::ImagePtr CameraNode::processFrame(const char *frame, size_t size, sensor_msgs::CameraInfoPtr &info)
{
  //std::cout << "processFrame Entered" << std::endl;
  msg_camera_info_.header.stamp = ros::Time::now();
  msg_camera_info_.height = cam_.getHeightMax();
  msg_camera_info_.width = cam_.getWidthMax();
  msg_camera_info_.roi.width = cam_.getROIWidth();
  msg_camera_info_.roi.height = cam_.getROIHeight();
  msg_camera_info_.roi.x_offset = cam_.getROI_X();
  msg_camera_info_.roi.y_offset = cam_.getROI_Y();

  sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image());
  msg_image->header = msg_camera_info_.header;
  msg_image->height = msg_camera_info_.roi.height;
  msg_image->width = msg_camera_info_.roi.width;
  msg_image->encoding = Camera::colorModeToString(cam_.getColorMode());
  msg_image->is_bigendian = false;
  msg_image->step = size / msg_image->height;
  //ROS_INFO("Size: %d", size);
  msg_image->data.resize(size);
  memcpy(msg_image->data.data(), frame, size);

  sensor_msgs::CameraInfoPtr msg(new sensor_msgs::CameraInfo(msg_camera_info_));
  info = msg;
  
  return msg_image;
}*/

sensor_msgs::ImagePtr CameraNode::processFrame(const char *frame, size_t size, sensor_msgs::CameraInfoPtr &info, sensor_msgs::CameraInfo &msg_info)
{
  if (binning_)
  {
    msg_info.height = cam_.getHeight() / 2;
    msg_info.width = cam_.getWidth() / 2;
    msg_info.roi.width = cam_.getROIWidth() / 2;
    msg_info.roi.height = cam_.getROIHeight() / 2;
    msg_info.roi.x_offset = cam_.getROI_X() / 2;
    msg_info.roi.y_offset = cam_.getROI_Y() / 2;
  }
  else
  {
    msg_info.height = cam_.getHeight();
    msg_info.width = cam_.getWidth();
    msg_info.roi.width = cam_.getROIWidth();
    msg_info.roi.height = cam_.getROIHeight();
    msg_info.roi.x_offset = cam_.getROI_X();
    msg_info.roi.y_offset = cam_.getROI_Y();
  }

  sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image());
  msg_image->header = msg_info.header;
  msg_image->height = cam_.getROIHeight();
  msg_image->width = cam_.getROIWidth();
  msg_image->encoding = Camera::colorModeToString(cam_.getColorMode());
  msg_image->is_bigendian = false;
  msg_image->step = size / msg_image->height;
  //ROS_INFO("Step: %d, size: %d, height: %d", msg_image->step, size, msg_image->height);
  msg_image->data.resize(size);
  memcpy(msg_image->data.data(), frame, size);
  sensor_msgs::CameraInfoPtr msg(new sensor_msgs::CameraInfo(msg_info));
  info = msg;
  
  return msg_image;
}

void CameraNode::DrawBrightnessAOI(sensor_msgs::ImagePtr &msg)
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
  cv::rectangle(cv_ptr->image, cv::Rect(visualize_brightness_aoi_.s32X, visualize_brightness_aoi_.s32Y, visualize_brightness_aoi_.s32Width, visualize_brightness_aoi_.s32Height), cv::Scalar(255,0,0), 4);
  msg = cv_ptr->toImageMsg();
}

void CameraNode::BinImg(sensor_msgs::ImagePtr &msg)
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

/*void CameraNode::CalExp()
{
  static unsigned int counter = 0;
  if (!cal_exp_2_ && counter >= 1)
    cal_exp_2_ = true;
  counter++;
  if (counter >= 5)
  {
    double exp_t = cam_.getExposure();
    if (exp_t > 19.9)
    {
      cal_exp_latch_ = false;
      double min_exp_t = cam_.getExposureMin();
      cam_.setExposure(&min_exp_t);
      ROS_INFO("EXPOSURE CALIBRATION FINISHED");
    }
    else
    {
      double new_exp_t = 1.029652447*exp_t;
      if ( (new_exp_t-exp_t) < exp_increment_ )
        exposure_calib_newest_.exp_time = exp_t + exp_increment_;
      else
        exposure_calib_newest_.exp_time = new_exp_t;
      cam_.setExposure(&exposure_calib_newest_.exp_time);
      ROS_INFO("EXPOSURE: %f", exposure_calib_newest_.exp_time);
    }
    counter = 0;
  }
}*/

void CameraNode::CalExp(double exposure)
{
  static unsigned int counter = 0;
  /*if (!cal_exp_2_ && counter >= 1)
    cal_exp_2_ = true;*/
  counter++;
  if (counter >= 5)
  {
    counter = 0;
    //double exp_t = cam_.getExposure();
    if (exposure > 19.9)
    {
      cal_exp_ = false;
      double min_exp_t = cam_.getExposureMin();
      cam_.setExposure(&min_exp_t);
      ROS_INFO("EXPOSURE CALIBRATION FINISHED");
      bag.close();
    }
    else
    {
      double new_exposure = 1.029652447*exposure;
      if ( (new_exposure - exposure) < exp_increment_ )
        new_exposure = exposure + exp_increment_;
      //else
      //  exposure_calib_newest_.exp_time = new_exp_t;
      cam_.setExposure(&new_exposure);
      ROS_INFO("EXPOSURE: %f", new_exposure);
    }  
  }
}

// Timestamp and publish the image. Called by the streaming thread.
void CameraNode::publishImage(const char *frame, size_t size, ros::Time stamp, int pps, double exposure, unsigned int gain, unsigned long long frame_count)
{  
  if (publish_extras_)
  {
    // Publish ppscontrol and exposure values
    extras_.pps = pps; //l_cam_.getGPIOConfiguration();
    PpsCount++;
    /*if (left_extras_.pps == 1)
    {
      ROS_INFO("Left Camera time, %f, %f", l_stamp_.toSec(), r_stamp_.toSec());
      if (leftPpsCount != 100)
        ROS_INFO("Left Camera frequency: %d Hz", leftPpsCount);
      leftPpsCount = 0;
    }*/
    extras_.gain = gain;
    extras_.frame_count = frame_count;
    if (auto_exposure_)
      extras_.exposure_time = exposure;
    else
      extras_.exposure_time = exposure_time_;*/
    
    extras_.header.stamp = stamp;
    pub_extras_.publish(extras_);
  }
  
  sensor_msgs::CameraInfoPtr info;
  sensor_msgs::ImagePtr img = processFrame(frame, size, info, msg_camera_info_);
  if (visualize_)
    DrawBrightnessAOI(img);
  if (binning_)
    BinImg(img);
  
  //exposure_calib_.header = img->header;
  //exposure_calib_.exp_time = exposure_;
  //pub_exposure_.publish(exposure_calib_);
  info->header.stamp = stamp;
  pub_stream_.publish(img, info);
  
  if (cal_exp_) {
    bag.write("exposure", stamp, extras_);
    bag.write("image", stamp, img);
    CalExp(exposure);
  }
  /*if (cal_exp_2_)
  {
    bag.write("exposure", ros::Time::now(), exposure_calib_);
    bag.write("image", ros::Time::now(), img);
    exposure_calib_.exp_time = exposure_calib_new_.exp_time;
    exposure_calib_new_.exp_time = exposure_calib_newest_.exp_time;
    cal_exp_2_ = cal_exp_;
    cal_exp_ = cal_exp_latch_;
    if (!cal_exp_2_)
      bag.close();
  }*/
}

/*void CameraNode::publishImagefromList()
{
  char *frame;
  size_t size;
  ros::Time stamp;
  int pps;
  double exposure;
  int count;
  
  while (!stop_publish_) {
    if (cam_.getImageDataFromList(&frame, size, stamp, pps, exposure, count))
    {
      
      if (publish_extras_)
      {
        // Publish ppscontrol and exposure values
        extras_.pps = pps;//r_cam_.getGPIOConfiguration();
        PpsCount++;
        if (extras_.pps == 1)
        {
          //ROS_INFO("Right Camera time, %f", r_stamp_.toSec());
          if (PpsCount != 100)
            ROS_INFO("Camera frequency: %d Hz", PpsCount);
          PpsCount = 0;
        }
        extras_.exp_time = exposure_;
        if (auto_exposure_)
          exposure_ = exposure;
        else
          exposure_ = exposure_time_;
           
        extras_.header.stamp = msg_camera_info_.header.stamp;
        pub_extras_.publish(extras_);
      }
      
      sensor_msgs::CameraInfoPtr info;
      sensor_msgs::ImagePtr img = processFrame(frame, size, cam_, info, msg_camera_info_);
      
      if (binning_)
        BinImg(img);
      if (visualize_)
        DrawBrightnessAOI(img);
      
      // Publish Image
      pub_stream_.publish(img, info);
      cam_.removeFromList();
    }
    usleep(1000);
  }
  //ROS_INFO("Right loop ended, data_ready: %d, %d", r_stamp_ready, r_img_info_ready);
}*/

void CameraNode::startCamera()
{
  if (running_ || !configured_)
    return;
  cam_.startVideoCapture(boost::bind(&CameraNode::publishImage, this, _1, _2, _3, _4, _5, _6, _7));
  stop_publish_ = false;
  //stamp_ready = false;
  //img_info_ready = false;
  //thread_ = boost::thread(&CameraNode::publishImagefromList, this);
  ROS_INFO("Started video stream.");
  running_ = true;
}

void CameraNode::stopCamera()
{
  if (!running_)
    return;
  ROS_INFO("Stopping video stream.");
  stop_publish_ = true;
  //thread_.join();
  cam_.stopVideoCapture();
  PpsCount = 0;
  ROS_INFO("Stopped video stream.");
  running_ = false;
}

void CameraNode::closeCamera()
{
  stopCamera();
  cam_.closeCamera();
}

} // namespace ueye

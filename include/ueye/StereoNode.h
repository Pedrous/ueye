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

#ifndef _STEREO_NODE_H_
#define _STEREO_NODE_H_

// Use includes from CameraNode
#include <ueye/CameraNode.h>
#include <ueye/stereoConfig.h>

// Threading
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// Msg for exposure and pps
#include "ueye/exposure.h"
#include "ueye/ppscontrol.h"
#include "ueye/extras.h"

namespace ueye
{

class StereoNode
{
public:
  StereoNode(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~StereoNode();

private:
  // ROS callbacks
  void reconfig(stereoConfig &config, uint32_t level);
  void reconfigCam(stereoConfig &config, uint32_t level, Camera &cam);
  void timerCallback(const ros::TimerEvent& event);
  void timerForceTrigger(const ros::TimerEvent& event);
  bool setCameraInfoL(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);
  bool setCameraInfoR(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);
  bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp, Camera& cam,
                     sensor_msgs::CameraInfo &msg_info);

  void loadIntrinsics(Camera &cam, sensor_msgs::CameraInfo &msg_info);
  sensor_msgs::ImagePtr processFrame(const char *frame, size_t size, const Camera &cam, sensor_msgs::CameraInfoPtr &info, sensor_msgs::CameraInfo &msg_info);
  void publishImageL(const char *frame, size_t size, ueye::extras& extras);
  void publishImageR(const char *frame, size_t size, ueye::extras& extras);
  void startCamera();
  void stopCamera();
  void closeCamera();
  void handlePath(std::string &path);
  void DrawBrightnessAOI_L(sensor_msgs::ImagePtr &msg);
  void DrawBrightnessAOI_R(sensor_msgs::ImagePtr &msg);
  void BinImg(sensor_msgs::ImagePtr &msg);

  dynamic_reconfigure::Server<stereoConfig> srv_;
  ros::Timer timer_;
  ros::Timer timer_force_trigger_;
  sensor_msgs::CameraInfo l_msg_camera_info_, r_msg_camera_info_;

  ueye::Camera l_cam_, r_cam_;
  bool running_;
  bool configured_;
  bool binning_;
  bool visualize_;
  bool force_streaming_;
  std::string config_path_;
  int trigger_mode_;
  bool auto_exposure_;
  bool auto_gain_;
  int zoom_;
  ros::Time l_stamp_, r_stamp_;
  IS_RECT l_aoi_, r_aoi_;
  IS_RECT l_brightness_aoi_, r_brightness_aoi_;
  IS_RECT l_visualize_brightness_aoi_, r_visualize_brightness_aoi_;
  double l_exposure_;
  double r_exposure_;
  double l_exposure_new_;
  double r_exposure_new_;
  //ueye::exposure exposure_calib_;
  //ueye::ppscontrol pps_;
  ueye::extras left_extras_;
  ueye::extras right_extras_;
  //bool l_firstPPScontrolValueNeeded_;
  //bool r_firstPPScontrolValueNeeded_;
  double exposure_time_;
  int leftPpsCount;
  int rightPpsCount;
  int l_frameNo;
  int r_frameNo;
  boost::thread l_thread_;
  boost::thread r_thread_;
  bool stop_publish_;
  boost::condition_variable cond_l_stamp_ready, cond_r_stamp_ready, cond_l_img_info_ready, cond_r_img_info_ready;
  bool l_stamp_ready, r_stamp_ready, l_img_info_ready, r_img_info_ready;
  boost::posix_time::time_duration timeout;
  

  // ROS topics
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher l_pub_stream_, r_pub_stream_;
  ros::ServiceServer l_srv_cam_info_, r_srv_cam_info_;
  ros::Publisher l_pub_extras_, r_pub_extras_;

  // Threading
  boost::mutex mutex_;
  boost::mutex mutex2_;
  boost::mutex mutex3_;
};

} // namespace ueye

#endif // _STEREO_NODE_H_

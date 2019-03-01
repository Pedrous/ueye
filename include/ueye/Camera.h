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

#ifndef UEYE_CAMERA_H_
#define UEYE_CAMERA_H_

#include <ueye.h>
#include <stdexcept>
#include <string>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>

namespace ueye
{

/*struct imageDataStruct
{
  char *img_mem;
  size_t size;
  ros::Time stamp;
  int pps;
  double exposure;
  int count;
  int imgID;
};*/

struct ExposureGainStruct {
  ros::Time stamp;
  double exposure;
  unsigned int gain;
};

struct uEyeException : public std::runtime_error
{
  int error_code;
  uEyeException(int code, const char* msg) :
      std::runtime_error(msg), error_code(code)
  {
  }
};

// memory needed for live display while using DIB
struct Memory
{
    Memory(void)
        : pcImageMemory(NULL)
        , lMemoryId(0)
        , lSequenceId(0)
    {
    }

    char* pcImageMemory;
    INT lMemoryId;
    INT lSequenceId;
};

enum uEyeColor
{
  MONO8 = IS_CM_MONO8,
  MONO16 = IS_CM_MONO16,
  YUV = IS_CM_UYVY_PACKED,
  YCbCr = IS_CM_CBYCRY_PACKED,
  BGR5 = IS_CM_BGR5_PACKED,
  BGR565 = IS_CM_BGR565_PACKED,
  BGR8 = IS_CM_BGR8_PACKED,
  BGRA8 = IS_CM_BGRA8_PACKED,
  BGRY8 = IS_CM_BGRY8_PACKED,
  RGB8 = IS_CM_RGB8_PACKED,
  RGBA8 = IS_CM_RGBA8_PACKED,
  RGBY8 = IS_CM_RGBY8_PACKED,
};
enum TriggerMode
{
  TRIGGER_OFF = IS_SET_TRIGGER_OFF,
  TRIGGER_HI_LO = IS_SET_TRIGGER_HI_LO,
  TRIGGER_LO_HI = IS_SET_TRIGGER_LO_HI,
  TRIGGER_SOFTWARE = IS_SET_TRIGGER_SOFTWARE,
  TRIGGER_HI_LO_SYNC = IS_SET_TRIGGER_HI_LO_SYNC,
  TRIGGER_LO_HI_SYNC = IS_SET_TRIGGER_LO_HI_SYNC,
};
enum FlashMode
{
  FLASH_OFF = IO_FLASH_MODE_OFF,
  FLASH_TRIGGER_ACTIVE_LO = IO_FLASH_MODE_TRIGGER_LO_ACTIVE,
  FLASH_TRIGGER_ACTIVE_HI = IO_FLASH_MODE_TRIGGER_HI_ACTIVE,
  FLASH_CONSTANT_HIGH = IO_FLASH_MODE_CONSTANT_HIGH,
  FLASH_CONSTANT_LOW = IO_FLASH_MODE_CONSTANT_LOW,
  FLASH_FREERUN_ACTIVE_LO = IO_FLASH_MODE_FREERUN_LO_ACTIVE,
  FLASH_FREERUN_ACTIVE_HI = IO_FLASH_MODE_FREERUN_HI_ACTIVE,
};
enum MirrorMode
{
  NO_MIRROR,
  MIRROR_UP_DOWN,
  MIRROR_LEFT_RIGHT,
  MIRROR_BOTH,
};

class Camera
{
public:
  Camera();
  ~Camera();

  // Initialization functions in order they should be called.
  static bool checkVersion(int &major, int &minor, int &build, const char *&expected);
  int getNumberOfCameras() const;
  unsigned int getSerialNumberList(std::vector<unsigned int>& serial, std::vector<unsigned int>& dev_id);
  bool openCameraCamId(unsigned int id);
  bool openCameraDevId(unsigned int id);
  bool openCameraSerNo(unsigned int serial_number);

  static const char* colorModeToString(uEyeColor mode);

  // Get Properties
  const char * getCameraName() const { return cam_info_.strSensorName; }
  unsigned int getCameraSerialNo() const { return serial_number_; }
  int getWidthMax() const { return cam_info_.nMaxWidth; }
  int getHeightMax() const { return cam_info_.nMaxHeight; }
  int getWidth() const { return cam_info_.nMaxWidth / zoom_; }
  int getHeight() const { return cam_info_.nMaxHeight / zoom_; }
  int getROIWidth() const { return aoi_.s32Width / zoom_; }
  int getROIHeight() const { return aoi_.s32Height / zoom_; }
  int getROI_X() const { return ( cam_info_.nMaxWidth - aoi_.s32Width - aoi_.s32X ) / zoom_; }
  int getROI_Y() const { return ( cam_info_.nMaxHeight - aoi_.s32Height - aoi_.s32Y ) / zoom_; }
  int getZoom() const { return zoom_; }
  int getMirror() const { return mirror_; }
  
  IS_RECT getAOI() const { return aoi_; }
  uEyeColor getColorMode() const { return color_mode_; }
  bool getAutoExposure() const { return auto_exposure_; }
  int getAutoExposureReference() const { return auto_exposure_reference_; };
  double getAutoExposureMax() const { return auto_exposure_max_; };
  int getAutoExposureSpeed() const { return auto_exposure_speed_; };
  int getAutoExposureHysteresis() const { return auto_exposure_hysteresis_; };
  int getAutoExposureSkipFrames() const { return auto_exposure_skip_frames_; };
  double getExposure();
  double getExposureMin();
  double getExposureMax();
  double getExposureRange();
  double getExposureIncrement();
  bool getHardwareGamma() const { return hardware_gamma_; }
  int getPixelClock() const { return pixel_clock_; }
  int getExtendedPixelClock() const { return extended_pixel_clock_; }
  bool getGainBoost() const { return gain_boost_; }
  bool getAutoGain() const { return auto_gain_; }
  unsigned int getHardwareGain();
  TriggerMode getTriggerMode();
  TriggerMode getSupportedTriggers();
  int getNumberOfAOIs();
  IS_SIZE_2D getMinSizeAOI();
  void getCurrentAOIinfo();
  IS_MULTI_AOI_CONTAINER * getDefaultConfAOI();
  void getAOI(IS_RECT& rectAOI);
  void getAutoBrightnessAOI(IS_RECT& rectAOI);
  int getGPIOConfiguration();
  void getFrameRate(double *rate);
  //bool getImageDataFromList(char **frame, size_t& size, ros::Time& stamp, int& pps, double& exposure, int& count);
  //bool removeFromList();
  //void clearList();

  // Set Properties
  void setColorMode(uEyeColor mode);
  void setAutoExposure(bool *enable);
  void setAutoExposureReference(int *reference);
  void setAutoExposureMax(double *max);
  void setAutoExposureSpeed(int *speed);
  void setAutoExposureHysteresis(int *hysteresis);
  void setAutoExposureSkipFrames(int *skip);
  void setExposure(double *time_ms);
  void setHardwareGamma(bool *enable);
  void setZoom(int *zoom);
  void setMirror(int *mirror);
  void setAOI(IS_RECT& rectAOI);
  void setAutoBrightnessAOI(IS_RECT& rectAOI);
  void setPixelClock(int *MHz);
  void setExtendedPixelClock(bool *enable);
  void setFrameRate(double *rate);
  void setGainBoost(bool *enable);
  void setAutoGain(bool *enable);
  void setHardwareGain(int *gain);
  bool setTriggerMode(TriggerMode mode);
  void setFlashWithGlobalParams(FlashMode mode);
  void setFlash(FlashMode mode, int delay_usec, unsigned int duration_usec);
  void setFlash(FlashMode mode);
  void setTriggerDelay(int delay_usec);
  void setCurrentAOIinfo();
  void setGPIOConfiguration();
  
  void deleteAOIidx(unsigned int idx);

  bool forceTrigger();

  typedef boost::function<void(const char *, size_t, ros::Time, int, double)> CamCaptureCB;
  void startVideoCapture(CamCaptureCB);
  void stopVideoCapture();

  void closeCamera();

private:
  inline void checkError(INT err) const {
    INT err2 = IS_SUCCESS;
    IS_CHAR* msg;
    if (err != IS_SUCCESS) {
      if (cam_ != 0) {
        is_GetError(cam_, &err2, &msg);
        if (err2 != IS_SUCCESS) {
          ROS_INFO("Before, err: %d, err2: %d", err, err2);
          throw ueye::uEyeException(err, msg);
          ROS_INFO("After");
        }
      } else {
        throw ueye::uEyeException(err, "Camera failed to initialize");
      }
    }
  }
  void initPrivateVariables();
  int getSubsampleParam(int *scale);
  int getBinningParam(int *scale);
  void flashUpdateGlobalParams();

  std::vector<char*> img_mem_;
  std::vector<int> img_mem_id_;
  std::vector<int> img_mem_seq_num_;
  void initMemoryPool(int size);
  void destroyMemoryPool();
  void captureThread(CamCaptureCB callback);
  void restartVideoCapture();
  void processFrame(char *img_mem, int img_ID, size_t size, CamCaptureCB callback);
  INT GetImageID (char* pbuf);
  INT GetImageSeqNum (char* pbuf);
  void SaveExposureAndGain();
  bool LoadExposureAndGain( ros::Time now, double& exposure, unsigned int& gain );
  
  IS_RECT aoi_;
  IS_RECT brightness_aoi_;
  uEyeColor color_mode_;
  bool auto_exposure_;
  int auto_exposure_reference_;
  double auto_exposure_max_;
  int auto_exposure_speed_;
  int auto_exposure_hysteresis_;
  int auto_exposure_skip_frames_;
  double exposure_time_;
  bool hardware_gamma_;
  bool gain_boost_;
  int zoom_;
  int mirror_;
  int pixel_clock_;
  bool auto_gain_;
  int hardware_gain_;
  double frame_rate_;
  bool flash_global_params_;
  HIDS cam_;
  SENSORINFO cam_info_;
  unsigned int serial_number_;
  //std::vector<imageDataStruct> dataList_;
  std::vector<ExposureGainStruct> ExposureGainList_;
  bool trigger;

  volatile bool streaming_;
  volatile bool stop_capture_;
  CamCaptureCB stream_callback_;
  boost::thread thread_;

  bool extended_pixel_clock_;
  IS_MULTI_AOI_CONTAINER * m_psMultiAOIs = new IS_MULTI_AOI_CONTAINER;
  
  boost::mutex mutex;
  
  // Added by me for debugging
  UEYEIMAGEINFO PrevImageInfo;
};
} //namespace ueye

#endif /* UEYE_CAMERA_H_ */

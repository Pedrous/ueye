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

#include <ueye/Camera.h>
#include <iostream>
#include "ros/ros.h"

#include <cstring>
#include <string.h>
#include <iostream>
#include <typeinfo>
#include <bitset>
#include <iomanip>
#include <ctime>
#include <time.h>
#include <stdlib.h>

// Check expected uEye SDK version in ueye.h for supported architectures
#if defined(__i386) || defined(__i386__) || defined(_M_IX86)
  #define EXPECTED_VERSION_MAJOR 4
  #define EXPECTED_VERSION_MINOR 60
  #define EXPECTED_VERSION_BUILD 5
  #if UEYE_VERSION_CODE != UEYE_VERSION(EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR, 0)
  #warning Expected ueye driver version 4.60.x. Different version found in ueye.h.
  #endif
#elif defined(__x86_64) || defined(__x86_64__) || defined(__amd64) || defined(_M_X64)
  #define EXPECTED_VERSION_MAJOR 4
  #define EXPECTED_VERSION_MINOR 60
  #define EXPECTED_VERSION_BUILD 5
  #if UEYE_VERSION_CODE != UEYE_VERSION(EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR, 0)
  #warning Expected ueye driver version 4.60.x. Different version found in ueye.h.
  #endif
#elif defined(__arm__) || defined(__TARGET_ARCH_ARM)
  #define EXPECTED_VERSION_MAJOR 4
  #define EXPECTED_VERSION_MINOR 60
  #define EXPECTED_VERSION_BUILD 0
  #if UEYE_VERSION_CODE != UEYE_VERSION(EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR, 0)
  #warning Expected ueye driver version 4.60.x. Different version found in ueye.h.
  #endif
#elif defined(__ia64) || defined(__ia64__) || defined(_M_IA64)
  #define EXPECTED_VERSION_MAJOR 0
  #define EXPECTED_VERSION_MINOR 0
  #define EXPECTED_VERSION_BUILD 0
  #warning Architecture ia64 not explicitly supported.
#elif defined(__ppc__) || defined(__ppc) || defined(__powerpc__) || defined(_ARCH_COM) || defined(_ARCH_PWR) || defined(_ARCH_PPC) || defined(_M_MPPC) || defined(_M_PPC)
  #define EXPECTED_VERSION_MAJOR 0
  #define EXPECTED_VERSION_MINOR 0
  #define EXPECTED_VERSION_BUILD 0
  #warning Architecture ppc not explicitly supported.
#else
  #define EXPECTED_VERSION_MAJOR 0
  #define EXPECTED_VERSION_MINOR 0
  #define EXPECTED_VERSION_BUILD 0
  #warning Architecture not explicitly supported. Supported: amd64, i386, arm.
#endif


namespace ueye
{

void Camera::initPrivateVariables()
{
  streaming_ = false;
  stop_capture_ = false;
  color_mode_ = MONO8;
  auto_exposure_ = false;
  exposure_time_ = 99.0;
  hardware_gamma_ = true;
  gain_boost_ = false;
  zoom_ = 1;
  pixel_clock_ = 20;
  extended_pixel_clock_ = false;
  auto_gain_ = false;
  hardware_gain_ = 100;
  frame_rate_ = 5.0;
  flash_global_params_ = false;
  serial_number_ = 0;
  cam_ = 0;
  memset(&cam_info_, 0x00, sizeof(cam_info_));
  stream_callback_ = NULL;
  aoi_.s32Width = 1936;
  aoi_.s32Height = 1216;
  aoi_.s32X = 0;
  aoi_.s32Y = 0;
  trigger = false;
  
  m_psMultiAOIs->nNumberOfAOIs = 0;
}

Camera::Camera()
{
  initPrivateVariables();
  setGPIOConfiguration();
  
  int m_nMaxNumberMultiAOIs = getNumberOfAOIs();
  /* If multi AOI is supported, allocate the necessary memeory */
  if (m_nMaxNumberMultiAOIs > 1)
  {
    /* Note: Always create the structure of the multi AOI for the maximum number of settable AOIs. */
    m_psMultiAOIs->nNumberOfAOIs = m_nMaxNumberMultiAOIs;
    m_psMultiAOIs->pMultiAOIList = new IS_MULTI_AOI_DESCRIPTOR[m_nMaxNumberMultiAOIs];
    ZeroMemory(m_psMultiAOIs->pMultiAOIList, sizeof(IS_MULTI_AOI_DESCRIPTOR) * m_nMaxNumberMultiAOIs);
  }
}

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
bool Camera::checkVersion(int &major, int &minor, int &build, const char *&expected)
{
  expected = STR(EXPECTED_VERSION_MAJOR) "." STR(EXPECTED_VERSION_MINOR) "." STR(EXPECTED_VERSION_BUILD);
  build = is_GetDLLVersion();
  major = (build >> 24) & 0x000000FF;
  minor = (build >> 16) & 0x000000FF;
  build &= 0x0000FFFF;
  if ((major == EXPECTED_VERSION_MAJOR) && (minor == EXPECTED_VERSION_MINOR) && (build == EXPECTED_VERSION_BUILD)) {
    return true;
  }
  return false;
}
#undef STR_HELPER
#undef STR

int Camera::getNumberOfCameras() const
{
  int num = 0;
  checkError(is_GetNumberOfCameras(&num));
  return num;
}

unsigned int Camera::getSerialNumberList(std::vector<unsigned int>& serial, std::vector<unsigned int>& dev_id)
{
  int num = getNumberOfCameras();
  if (num > 0) {
    UEYE_CAMERA_LIST *list = (UEYE_CAMERA_LIST *)malloc(sizeof(DWORD) + num * sizeof(UEYE_CAMERA_INFO));
    list->dwCount = num;
    if (is_GetCameraList(list) == IS_SUCCESS) {
      num = list->dwCount;
      serial.resize(num);
      dev_id.resize(num);
      for (int i = 0; i < num; i++) {
        serial[i] = atoll(list->uci[i].SerNo);
        dev_id[i] = list->uci[i].dwDeviceID;
      }
    } else {
      num = 0;
    }
    free(list);
    return num;
  }
  return 0;
}

int Camera::getNumberOfAOIs()
{
  int m_nMaxNumberMultiAOIs = 0;
  is_AOI(cam_, IS_AOI_MULTI_GET_AOI | IS_AOI_MULTI_MODE_GET_MAX_NUMBER, (void *)&m_nMaxNumberMultiAOIs, sizeof(m_nMaxNumberMultiAOIs));
  
  return m_nMaxNumberMultiAOIs;
}

IS_SIZE_2D Camera::getMinSizeAOI()
{
  IS_SIZE_2D rMinSizeAOI;
  is_AOI(cam_, IS_AOI_MULTI_GET_AOI | IS_AOI_MULTI_MODE_GET_MINIMUM_SIZE, (void *)&rMinSizeAOI, sizeof(rMinSizeAOI));
  
  return rMinSizeAOI;
}

//TODO: muuta tähän oikeat palautukset
void Camera::getCurrentAOIinfo()
{
  /* Query the current configuration */
  is_AOI(cam_, IS_AOI_MULTI_GET_AOI, (void *)m_psMultiAOIs, m_psMultiAOIs->nNumberOfAOIs * sizeof(IS_MULTI_AOI_DESCRIPTOR) + sizeof(IS_MULTI_AOI_CONTAINER));
 
  /* Count the different AOIs */
  int nUsed = 0;
  int nComplemented = 0;
  int nErronous = 0;
  int nConflicted = 0;
  int m_nMaxNumberMultiAOIs = getNumberOfAOIs();
  for (int i = 0; i < m_nMaxNumberMultiAOIs; i ++) {
    /* Count the used AOIs */
    if (!(m_psMultiAOIs->pMultiAOIList[i].nStatus & IS_AOI_MULTI_STATUS_UNUSED) && (m_psMultiAOIs->pMultiAOIList[i].nWidth > 0) && (m_psMultiAOIs->pMultiAOIList[i].nHeight > 0)) 
    {
      nUsed++;
    }
    /* Count the added AOIs */
    if (m_psMultiAOIs->pMultiAOIList[i].nStatus & IS_AOI_MULTI_STATUS_COMPLEMENT)
    {
      nComplemented++;
    }
   
    /* Count the broken AOIs */
    if (m_psMultiAOIs->pMultiAOIList[i].nStatus & IS_AOI_MULTI_STATUS_ERROR)
    {
      nErronous++;
    }
   
    /* Count the critical AOIs (for example overlapping) */
    if (m_psMultiAOIs->pMultiAOIList[i].nStatus & IS_AOI_MULTI_STATUS_CONFLICT) 
    {
      nConflicted++;
    }
  }
}

void Camera::deleteAOIidx(unsigned int idx)
{
  /* Delete AOI with the index 1 */
  if (IS_SUCCESS == is_AOI(cam_, IS_AOI_MULTI_GET_AOI, (void*)m_psMultiAOIs, m_psMultiAOIs->nNumberOfAOIs * sizeof(IS_MULTI_AOI_DESCRIPTOR) + sizeof (IS_MULTI_AOI_CONTAINER)))
  {
    m_psMultiAOIs->pMultiAOIList[idx].nPosX = 0;
    m_psMultiAOIs->pMultiAOIList[idx].nPosY = 0;
    m_psMultiAOIs->pMultiAOIList[idx].nWidth = 0;
    m_psMultiAOIs->pMultiAOIList[idx].nHeight = 0;
    m_psMultiAOIs->pMultiAOIList[idx].nStatus = IS_AOI_MULTI_STATUS_UNUSED;
    int nRet = is_AOI(cam_, IS_AOI_MULTI_SET_AOI, (void*)m_psMultiAOIs, sizeof(IS_MULTI_AOI_CONTAINER));
  }
}
   
IS_MULTI_AOI_CONTAINER * Camera::getDefaultConfAOI()
{
  IS_MULTI_AOI_CONTAINER * m_MultiAOIs;
  /* Query the default configuration */
  is_AOI(cam_, IS_AOI_MULTI_GET_AOI | IS_AOI_MULTI_MODE_GET_DEFAULT, (void*)m_MultiAOIs, sizeof(IS_MULTI_AOI_CONTAINER) + m_psMultiAOIs->nNumberOfAOIs * sizeof  (IS_MULTI_AOI_DESCRIPTOR));
  
  return m_MultiAOIs;
}

//TODO: Muuta vielä tähän niin, että syötetään oikeat AOI:t
void Camera::setCurrentAOIinfo()
{
  /* Set AOI on position (100/200) with size (160/180) */
  m_psMultiAOIs->pMultiAOIList[0].nPosX = 100;
  m_psMultiAOIs->pMultiAOIList[0].nPosY = 200;
  m_psMultiAOIs->pMultiAOIList[0].nWidth = 160;
  m_psMultiAOIs->pMultiAOIList[0].nHeight = 180;
  m_psMultiAOIs->pMultiAOIList[0].nStatus = IS_AOI_MULTI_STATUS_SETBYUSER; /* Initially defined as user-defined AOI */
   
  /* Set further AOIs on position (400/800) with size (120/140). Take care on m_nMaxNumberMultiAOIs! */
  m_psMultiAOIs->pMultiAOIList[1].nPosX = 400;
  m_psMultiAOIs->pMultiAOIList[1].nPosY = 800;
  m_psMultiAOIs->pMultiAOIList[1].nWidth = 120;
  m_psMultiAOIs->pMultiAOIList[1].nHeight = 140;
  m_psMultiAOIs->pMultiAOIList[1].nStatus = IS_AOI_MULTI_STATUS_SETBYUSER;
   
  /* Verify the configuration */
  int nRet = is_AOI(cam_, IS_AOI_MULTI_SET_AOI | IS_AOI_MULTI_MODE_ONLY_VERIFY_AOIS, (void*)m_psMultiAOIs, sizeof(IS_MULTI_AOI_CONTAINER) + sizeof(IS_MULTI_AOI_DESCRIPTOR) * m_psMultiAOIs->nNumberOfAOIs);
   
  /* If the configuration is valid, set configuration etc. */
  if (nRet == IS_SUCCESS) {
    nRet = is_AOI(cam_, IS_AOI_MULTI_SET_AOI, (void*)m_psMultiAOIs, sizeof(IS_MULTI_AOI_CONTAINER) + sizeof(IS_MULTI_AOI_DESCRIPTOR) * m_psMultiAOIs->nNumberOfAOIs);
  }
}

bool Camera::openCameraCamId(unsigned int id)
{
  if (getNumberOfCameras() < 1) {
    return false;
  }

  cam_ = id;
  checkError(is_InitCamera(&cam_, 0));

  checkError(is_GetSensorInfo(cam_, &cam_info_));
  CAMINFO info;
  checkError(is_GetCameraInfo(cam_, &info));
  serial_number_ = atoll(info.SerNo);

  setColorMode(color_mode_);
  setAutoExposure(&auto_exposure_);
  if (!auto_exposure_) {
    setExposure(&exposure_time_);
  }
  setHardwareGamma(&hardware_gamma_);
  setGainBoost(&gain_boost_);
  setAutoGain(&auto_gain_);
  if (!auto_gain_) {
    setHardwareGain(&hardware_gain_);
  }
  setZoom(&zoom_);
  setPixelClock(&pixel_clock_);
  setExtendedPixelClock(&extended_pixel_clock_);
  setFrameRate(&frame_rate_);
  return true;
}
bool Camera::openCameraDevId(unsigned int id)
{
  return openCameraCamId(id | IS_USE_DEVICE_ID);
}
bool Camera::openCameraSerNo(unsigned int serial_number)
{
  std::vector<unsigned int> serial;
  std::vector<unsigned int> dev_id;
  unsigned int num = getSerialNumberList(serial, dev_id);
  for (unsigned int i = 0; i < num; i++) {
    if (serial[i] == serial_number) {
      return openCameraDevId(dev_id[i]);
    }
  }
  return false;
}

const char* Camera::colorModeToString(uEyeColor mode)
{
  switch (mode) {
    case MONO8:
      return "mono8";
    case MONO16:
      return "mono16";
    case BGR5:
      return "bgr5";
    case BGR565:
      return "bgr565";
    case YUV:
      return "yuv422";
    case YCbCr:
      return "ycbcr422";
    case BGR8:
      return "bgr8";
    case RGB8:
      return "rgb8";
    case BGRA8:
    case BGRY8:
      return "bgra8";
    case RGBA8:
    case RGBY8:
      return "rgba8";
  }
  return "";
}

double Camera::getExposure()
{
  double time_ms;
  checkError(is_Exposure(cam_, IS_EXPOSURE_CMD_GET_EXPOSURE, &time_ms, sizeof(double)));
  return time_ms;
}
double Camera::getExposureMin()
{
  double time_ms;
  checkError(is_Exposure(cam_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, &time_ms, sizeof(double)));
  return time_ms;
}
double Camera::getExposureMax()
{
  double time_ms;
  checkError(is_Exposure(cam_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX, &time_ms, sizeof(double)));
  return time_ms;
}
double Camera::getExposureRange()
{
  double time_ms;
  checkError(is_Exposure(cam_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, &time_ms, sizeof(double)));
  return time_ms;
}
double Camera::getExposureIncrement()
{
  double time_ms;
  checkError(is_Exposure(cam_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_INC, &time_ms, sizeof(double)));
  return time_ms;
}
unsigned int Camera::getHardwareGain()
{
  hardware_gain_ = is_SetHWGainFactor(cam_, IS_GET_MASTER_GAIN_FACTOR, 0);
  return hardware_gain_;
}
TriggerMode Camera::getTriggerMode()
{
  return (TriggerMode)is_SetExternalTrigger(cam_, IS_GET_EXTERNALTRIGGER);
}
TriggerMode Camera::getSupportedTriggers()
{
  return (TriggerMode)is_SetExternalTrigger(cam_, IS_GET_SUPPORTED_TRIGGER_MODE);
}

void Camera::setColorMode(uEyeColor mode)
{
  bool restart = streaming_ && (stream_callback_ != NULL);
  stopVideoCapture();
  if (is_SetColorMode(cam_, mode) != IS_SUCCESS) {
    mode = MONO8;
    is_SetColorMode(cam_, mode);
  }
  color_mode_ = mode;
  if (restart) {
    startVideoCapture(stream_callback_);
  }
}
void Camera::setAutoExposure(bool *enable)
{
  double param1 = *enable ? 1.0 : 0.0;
  double param2 = 0;
  if (IS_SUCCESS != is_SetAutoParameter(cam_, IS_SET_ENABLE_AUTO_SHUTTER, &param1, &param2)) {
    param1 = 0;
    is_SetAutoParameter(cam_, IS_SET_ENABLE_AUTO_SHUTTER, &param1, &param2);
    *enable = false;
  }
  auto_exposure_ = *enable;
}
void Camera::setAutoExposureReference(int *reference)
{
  double param1 = *reference;
  if (IS_SUCCESS != is_SetAutoParameter(cam_, IS_SET_AUTO_REFERENCE, &param1, 0)) {
    param1 = 0;
    is_SetAutoParameter(cam_, IS_GET_AUTO_REFERENCE, &param1, 0);
    *reference = param1;
  }
  auto_exposure_reference_ = *reference;
}
void Camera::setAutoExposureMax(double *max)
{
  double param1 = *max;
  if (IS_SUCCESS != is_SetAutoParameter(cam_, IS_SET_AUTO_SHUTTER_MAX, &param1, 0)) {
    param1 = 0;
    is_SetAutoParameter(cam_, IS_GET_AUTO_SHUTTER_MAX, &param1, 0);
    *max = param1;
  }
  auto_exposure_max_ = *max;
}
void Camera::setAutoExposureSpeed(int *speed)
{
  double param1 = *speed;
  if (IS_SUCCESS != is_SetAutoParameter(cam_, IS_SET_AUTO_SPEED, &param1, 0)) {
    param1 = 0;
    is_SetAutoParameter(cam_, IS_GET_AUTO_SPEED, &param1, 0);
    *speed = param1;
  }
  auto_exposure_speed_ = *speed;
}
void Camera::setAutoExposureHysteresis(int *hysteresis)
{
  double param1 = *hysteresis;
  if (IS_SUCCESS != is_SetAutoParameter(cam_, IS_SET_AUTO_HYSTERESIS, &param1, 0)) {
    param1 = 0;
    is_SetAutoParameter(cam_, IS_GET_AUTO_HYSTERESIS, &param1, 0);
    *hysteresis = param1;
  }
  auto_exposure_hysteresis_ = *hysteresis;
}
void Camera::setAutoExposureSkipFrames(int *skip)
{
  double param1 = *skip;
  if (IS_SUCCESS != is_SetAutoParameter(cam_, IS_SET_AUTO_SKIPFRAMES, &param1, 0)) {
    param1 = 0;
    is_SetAutoParameter(cam_, IS_GET_AUTO_SKIPFRAMES, &param1, 0);
    *skip = param1;
  }
  auto_exposure_skip_frames_ = *skip;
}
void Camera::setExposure(double *time_ms)
{
  bool b = false;
  setAutoExposure(&b);
  checkError(is_Exposure(cam_, IS_EXPOSURE_CMD_SET_EXPOSURE, time_ms, sizeof(double)));
  flashUpdateGlobalParams();
  exposure_time_ = *time_ms;
}
void Camera::setHardwareGamma(bool *enable)
{
  if (*enable) {
    if (IS_SUCCESS != is_SetHardwareGamma(cam_, IS_SET_HW_GAMMA_ON)) {
      is_SetHardwareGamma(cam_, IS_SET_HW_GAMMA_OFF);
      *enable = false;
    }
  } else {
    is_SetHardwareGamma(cam_, IS_SET_HW_GAMMA_OFF);
  }
  hardware_gamma_ = *enable;
}
void Camera::setZoom(int *zoom)
{
  if (zoom_ != *zoom) {
    // Reset zoom
    is_SetSubSampling(cam_, 0);
    is_SetBinning(cam_, 0);

    // Try Subsampling then Binning
    if (IS_SUCCESS != is_SetSubSampling(cam_, getSubsampleParam(zoom))) {
      is_SetSubSampling(cam_, 0);
      if (IS_SUCCESS != is_SetBinning(cam_, getBinningParam(zoom))) {
        is_SetBinning(cam_, 0);
        *zoom = 1;
      }
    }

    // Zoom affects the frame-rate and needs a restart to change the buffer sizes
    is_HotPixel(cam_, IS_HOTPIXEL_ENABLE_CAMERA_CORRECTION, NULL, 0);
    setFrameRate(&frame_rate_);
    restartVideoCapture();
  }
  zoom_ = *zoom;
}
void Camera::setMirror(int *mirror)
{
  if (mirror_ != *mirror) {
  	switch (*mirror) {
  	
      case NO_MIRROR:
        checkError(is_SetRopEffect(cam_, IS_SET_ROP_MIRROR_UPDOWN, 0, 0));
        checkError(is_SetRopEffect(cam_, IS_SET_ROP_MIRROR_LEFTRIGHT, 0, 0));
        mirror_ = *mirror;
        break;
        
      case MIRROR_UP_DOWN:
        checkError(is_SetRopEffect(cam_, IS_SET_ROP_MIRROR_UPDOWN, 1, 0));
        checkError(is_SetRopEffect(cam_, IS_SET_ROP_MIRROR_LEFTRIGHT, 0, 0));
        mirror_ = *mirror;
        break;
        
      case MIRROR_LEFT_RIGHT:
        checkError(is_SetRopEffect(cam_, IS_SET_ROP_MIRROR_UPDOWN, 0, 0));
        checkError(is_SetRopEffect(cam_, IS_SET_ROP_MIRROR_LEFTRIGHT, 1, 0));
        mirror_ = *mirror;
        break;
        
      case MIRROR_BOTH:
        checkError(is_SetRopEffect(cam_, IS_SET_ROP_MIRROR_UPDOWN, 1, 0));
        checkError(is_SetRopEffect(cam_, IS_SET_ROP_MIRROR_LEFTRIGHT, 1, 0));
        mirror_ = *mirror;
        break;
    }
  }
}
void Camera::setAOI(IS_RECT& rectAOI) {
  if (IS_SUCCESS != is_AOI(cam_, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI)))
  {
    //ROS_INFO("Width: %d, Height: %d, X: %d, Y: %d", rectAOI.s32Width, rectAOI.s32Height, rectAOI.s32X, rectAOI.s32Y);
    is_AOI(cam_, IS_AOI_IMAGE_SET_AOI, (void*)&aoi_, sizeof(aoi_));
  }
  else
  {
    aoi_ = rectAOI;
  }
}
void Camera::setAutoBrightnessAOI(IS_RECT& rectAOI)
{
  if (IS_SUCCESS != is_AOI(cam_, IS_AOI_AUTO_BRIGHTNESS_SET_AOI, (void*)&rectAOI, sizeof(rectAOI)))
  {
    //ROS_INFO("Width: %d, Height: %d, X: %d, Y: %d", rectAOI.s32Width, rectAOI.s32Height, rectAOI.s32X, rectAOI.s32Y);
    is_AOI(cam_, IS_AOI_AUTO_BRIGHTNESS_SET_AOI, (void*)&aoi_, sizeof(aoi_));
  }
  else
  {
    brightness_aoi_ = rectAOI;
  }
}
void Camera::getAOI(IS_RECT& rectAOI)
{
  is_AOI(cam_, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI, sizeof(rectAOI));
}
void Camera::getAutoBrightnessAOI(IS_RECT& rectAOI)
{
  is_AOI(cam_, IS_AOI_AUTO_BRIGHTNESS_GET_AOI, (void*)&rectAOI, sizeof(rectAOI));
}
void Camera::setPixelClock(int *MHz)
{ 
  int pixelClockList[150];  // No camera has more than 150 different pixel clocks (uEye manual)
  int numberOfSupportedPixelClocks = 0;
  checkError(is_PixelClock(cam_, IS_PIXELCLOCK_CMD_GET_NUMBER, &numberOfSupportedPixelClocks, sizeof(numberOfSupportedPixelClocks)));
  if(numberOfSupportedPixelClocks > 0) {
    memset(pixelClockList, 0x00, sizeof(pixelClockList));
    checkError(is_PixelClock(cam_, IS_PIXELCLOCK_CMD_GET_LIST, pixelClockList, numberOfSupportedPixelClocks * sizeof(int)));
  }
  int minPixelClock = pixelClockList[0];
  int maxPixelClock = pixelClockList[numberOfSupportedPixelClocks-1];

  // As list is sorted smallest to largest...
  for(int i = 0; i < numberOfSupportedPixelClocks; i++) { 
    if(*MHz <= pixelClockList[i]) {
      *MHz = pixelClockList[i];  // ...get the closest-larger-or-equal from the list
      break;
    }  
  }

  if (*MHz < minPixelClock) {
    *MHz = minPixelClock;  // Clip to min
  }
  if (*MHz > maxPixelClock) {
    *MHz = maxPixelClock;  // Clip to max
  }
  
  checkError(is_PixelClock(cam_, IS_PIXELCLOCK_CMD_SET, MHz, sizeof(int)));
  setFrameRate(&frame_rate_);

  pixel_clock_ = *MHz;
}
void Camera::setExtendedPixelClock(bool *enable)
{
  // set extended pixel clock
  UINT extend;
  if (*enable)
  {
    extend = EXTENDED_PIXELCLOCK_RANGE_ON;
  }
  else
  {
    extend = EXTENDED_PIXELCLOCK_RANGE_OFF;
  }
  checkError(is_DeviceFeature(cam_, IS_DEVICE_FEATURE_CMD_SET_EXTENDED_PIXELCLOCK_RANGE_ENABLE, (void*)&extend, sizeof(extend)));
  extended_pixel_clock_ = *enable;
}
void Camera::setFrameRate(double *rate)
{
  checkError(is_SetFrameRate(cam_, *rate, rate));
  flashUpdateGlobalParams();
  frame_rate_ = *rate;
}
void Camera::getFrameRate(double *rate)
{
  is_GetFramesPerSecond(cam_, rate);
} 
void Camera::setGainBoost(bool *enable)
{
  if (is_SetGainBoost(cam_, IS_GET_SUPPORTED_GAINBOOST) == IS_SET_GAINBOOST_ON) {
    if (*enable)
      is_SetGainBoost(cam_, IS_SET_GAINBOOST_ON);
    else
      is_SetGainBoost(cam_, IS_SET_GAINBOOST_OFF);
    gain_boost_ = is_SetGainBoost(cam_, IS_GET_GAINBOOST) == IS_SET_GAINBOOST_ON;
  } else {
    gain_boost_ = false;
  }
  *enable = gain_boost_;
}
void Camera::setAutoGain(bool *enable)
{
  double param1 = *enable ? 1.0 : 0.0;
  double param2 = 0;
  if (IS_SUCCESS != is_SetAutoParameter(cam_, IS_SET_ENABLE_AUTO_GAIN, &param1, &param2)) {
    param1 = 0;
    is_SetAutoParameter(cam_, IS_SET_ENABLE_AUTO_GAIN, &param1, &param2);
    *enable = false;
  }
  auto_gain_ = *enable;
}
void Camera::setHardwareGain(int *gain)
{
  bool b = false;
  setAutoGain(&b);
  if (*gain < 0)
    *gain = 0;
  if (*gain > 400)
    *gain = 400;
  hardware_gain_ = is_SetHWGainFactor(cam_, IS_SET_MASTER_GAIN_FACTOR, *gain);
  *gain = hardware_gain_;
}
bool Camera::setTriggerMode(TriggerMode mode)
{
  if ((mode == 0) || (mode & getSupportedTriggers())) {
    if (is_SetExternalTrigger(cam_, mode) == IS_SUCCESS) {
      trigger = true;
      return true;
    }
  }
  return false;
}
void Camera::setFlashWithGlobalParams(FlashMode mode)
{
  UINT m = mode;
  switch (mode) {
    case FLASH_FREERUN_ACTIVE_LO:
    case FLASH_FREERUN_ACTIVE_HI:
    case FLASH_TRIGGER_ACTIVE_LO:
    case FLASH_TRIGGER_ACTIVE_HI:
      flash_global_params_ = true;
      break;

    case FLASH_CONSTANT_HIGH:
    case FLASH_CONSTANT_LOW:
      flash_global_params_ = false;
      break;

    case FLASH_OFF:
    default:
      flash_global_params_ = false;
      m = FLASH_OFF;
      break;
  }
  checkError(is_IO(cam_, IS_IO_CMD_FLASH_SET_MODE, (void*)&m, sizeof(m)));
  flashUpdateGlobalParams();
}
void Camera::setFlash(FlashMode mode, int delay_usec, unsigned int duration_usec)
{
  int num_mode = int(mode);

  checkError(is_IO(cam_, IS_IO_CMD_FLASH_SET_MODE, (void*)&num_mode, sizeof(num_mode)));

  if (mode != FLASH_OFF) {
    IO_FLASH_PARAMS params;
    memset(&params, 0, sizeof(params));

    params.s32Delay = delay_usec;
    params.u32Duration = duration_usec;

    checkError(is_IO(cam_, IS_IO_CMD_FLASH_SET_PARAMS, &params, sizeof(params)));
  }

  flash_global_params_ = false;
}
void Camera::setFlash(FlashMode mode)
{
  // If flash delay = 0 and flash duration = 0, the flash signal is
  // automatically synchronized to the exposure time.
  setFlash(mode, 0, 0);
}
void Camera::flashUpdateGlobalParams()
{
  if (flash_global_params_) {
    IO_FLASH_PARAMS params;
    checkError(is_IO(cam_, IS_IO_CMD_FLASH_GET_GLOBAL_PARAMS, (void*)&params, sizeof(params)));
    checkError(is_IO(cam_, IS_IO_CMD_FLASH_APPLY_GLOBAL_PARAMS, NULL, 0));
  }
}
void Camera::setTriggerDelay(int delay_usec)
{
  checkError(is_SetTriggerDelay(cam_, delay_usec));
}

bool Camera::forceTrigger()
{
  if (streaming_)
    return is_ForceTrigger(cam_) == IS_SUCCESS;
  return false;
}

void Camera::setGPIOConfiguration()
{
  INT nRet = IS_SUCCESS;
  IO_GPIO_CONFIGURATION gpioConfiguration;
 
  // Set configuration of GPIO1
  gpioConfiguration.u32Gpio = IO_GPIO_1;
  gpioConfiguration.u32Configuration = IS_GPIO_INPUT;
  gpioConfiguration.u32State = 0;
   
  nRet = is_IO(cam_, IS_IO_CMD_GPIOS_SET_CONFIGURATION, (void*)&gpioConfiguration, sizeof(gpioConfiguration));
}

int Camera::getGPIOConfiguration()
{
  INT nRet = IS_SUCCESS;
  IO_GPIO_CONFIGURATION gpioConfiguration;

  // Read information about GPIO1
  gpioConfiguration.u32Gpio = IO_GPIO_1;
  nRet = is_IO(cam_, IS_IO_CMD_GPIOS_GET_CONFIGURATION, (void*)&gpioConfiguration, sizeof(gpioConfiguration) );

  if (nRet == IS_SUCCESS)
  {
    if (gpioConfiguration.u32Configuration == IS_GPIO_INPUT)
    {
      // GPIO1 is currently configured as output
      return gpioConfiguration.u32State;
    }
    else
      return 3;
  }
  else
    return 2;
}

int Camera::getSubsampleParam(int *scale)
{
  int param;
  switch (*scale) {
    case 2:
      param = IS_SUBSAMPLING_2X_VERTICAL | IS_SUBSAMPLING_2X_HORIZONTAL;
      break;
    case 3:
      param = IS_SUBSAMPLING_3X_VERTICAL | IS_SUBSAMPLING_3X_HORIZONTAL;
      break;
    case 4:
      param = IS_SUBSAMPLING_4X_VERTICAL | IS_SUBSAMPLING_4X_HORIZONTAL;
      break;
    case 5:
      param = IS_SUBSAMPLING_5X_VERTICAL | IS_SUBSAMPLING_5X_HORIZONTAL;
      break;
    case 6:
      param = IS_SUBSAMPLING_6X_VERTICAL | IS_SUBSAMPLING_6X_HORIZONTAL;
      break;
    case 7:
    case 8:
      *scale = 8;
      param = IS_SUBSAMPLING_8X_VERTICAL | IS_SUBSAMPLING_8X_HORIZONTAL;
      break;
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
      *scale = 16;
      param = IS_SUBSAMPLING_16X_VERTICAL | IS_SUBSAMPLING_16X_HORIZONTAL;
      break;
    default:
      *scale = 1;
      param = IS_SUBSAMPLING_DISABLE;
      break;
  }
  return param;
}
int Camera::getBinningParam(int *scale)
{
  int param;
  switch (*scale) {
    case 2:
      param = IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL;
      break;
    case 3:
      param = IS_BINNING_3X_VERTICAL | IS_BINNING_3X_HORIZONTAL;
      break;
    case 4:
      param = IS_BINNING_4X_VERTICAL | IS_BINNING_4X_HORIZONTAL;
      break;
    case 5:
      param = IS_BINNING_5X_VERTICAL | IS_BINNING_5X_HORIZONTAL;
      break;
    case 6:
      param = IS_BINNING_6X_VERTICAL | IS_BINNING_6X_HORIZONTAL;
      break;
    case 7:
    case 8:
      *scale = 8;
      param = IS_BINNING_8X_VERTICAL | IS_BINNING_8X_HORIZONTAL;
      break;
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
      *scale = 16;
      param = IS_BINNING_16X_VERTICAL | IS_BINNING_16X_HORIZONTAL;
      break;
    default:
      *scale = 1;
      param = IS_BINNING_DISABLE;
      break;
  }
  return param;
}

void Camera::closeCamera()
{
  if (cam_ > 0) {
    // Release camera and all associated memory
    checkError(IS_SUCCESS != is_ExitCamera(cam_));
    initPrivateVariables();
  }
}

Camera::~Camera()
{
  closeCamera();
}

void Camera::initMemoryPool(int size)
{
  int bits = 32;
  switch (color_mode_) {
    case MONO8:
      bits = 8;
      break;
    case MONO16:
    case BGR5:
    case BGR565:
    case YUV:
    case YCbCr:
      bits = 16;
      break;
    case BGR8:
    case RGB8:
      bits = 24;
      break;
    case BGRA8:
    case BGRY8:
    case RGBA8:
    case RGBY8:
      bits = 32;
      break;
  }

  int width = getROIWidth();
  int height = getROIHeight();
  if (size < 2) {
    size = 2;
  }
  img_mem_.resize(size);
  img_mem_id_.resize(size);
  img_mem_seq_num_.resize(size);
  for (int i = 0; i < size; i++) {
    if (IS_SUCCESS != is_AllocImageMem(cam_, width, height, bits, &img_mem_[i], &img_mem_id_[i])) {
      throw uEyeException(-1, "Failed to initialize memory.");
    }
    //add memory to memory pool
    //if (IS_SUCCESS != is_SetImageMem(cam_, img_mem_[i], img_mem_id_[i])) {
    //  throw uEyeException(-1, "Failed to initialize memory.");
    //}
    if (IS_SUCCESS != is_AddToSequence (cam_, img_mem_[i], img_mem_id_[i])) {
      throw uEyeException(-1, "Failed to add to sequence.");
    }
    img_mem_seq_num_[i] = i + 1;
  }
  
  checkError( is_InitImageQueue(cam_, 0) );
}
void Camera::destroyMemoryPool()
{
  checkError( is_ExitImageQueue(cam_) );
  checkError( is_ClearSequence (cam_) );
  for (int i = 0; i < img_mem_.size(); i++) {
    checkError( is_FreeImageMem(cam_, img_mem_[i], img_mem_id_[i]) );
  }
  img_mem_.clear();
  img_mem_id_.clear();
  img_mem_seq_num_.clear();
}

void Camera::captureThread(CamCaptureCB callback)
{
  streaming_ = true;
  stop_capture_ = false;

  initMemoryPool(100);

  // Setup video event
  //checkError(is_EnableEvent(cam_, IS_SET_EVENT_FRAME));

  // There is a weird condition with the uEye SDK 4.30 where
  // this never returns when using IS_WAIT.
  // We are using IS_DONT_WAIT and retry every 0.1s for 2s instead
  bool capture = false;
  for (int i = 0; i < 20; ++i) {
    if (is_CaptureVideo(cam_, IS_DONT_WAIT) == IS_SUCCESS) {
      capture = true;
      break;
    }
    usleep(100000); // 100ms
  }
  if (!capture) {
    throw uEyeException(-1, "Capture could not be started.");
  }

  size_t depth = 0;
  switch (color_mode_) {
    case MONO8:
      depth = 1;
      break;
    case MONO16:
    case BGR5:
    case BGR565:
      depth = 1;
      break;
    case YUV:
    case YCbCr:
      depth = 2;
      break;
    case BGR8:
    case RGB8:
      depth = 3;
      break;
    case BGRA8:
    case BGRY8:
    case RGBA8:
    case RGBY8:
      depth = 4;
      break;
    default:
      throw uEyeException(-1, "Unsupported color mode when initializing image header.");
      return;
  }
  size_t size = (size_t)getROIWidth() * (size_t)getROIHeight() * depth;
  //ROS_INFO("Size: %d, Width: %d, Height: %d, aoi_.s32Width: %d, aoi_.s32Height: %d", size, (size_t)getROIWidth(), (size_t)getROIHeight(), aoi_.s32Width, aoi_.s32Height);
  
  char *img_mem = NULL;
  int img_ID = 0;
  UEYEIMAGEINFO ImageInfo;
  UEYE_CAPTURE_STATUS_INFO CaptureStatusInfo;
  PrevImageInfo.u64FrameNumber = 0;
  
  // Initialize thread pool service
  boost::asio::io_service ioService;
  boost::thread_group threadpool;
  boost::asio::io_service::work work(ioService);
  // This will add 2 threads to the thread pool
  threadpool.create_thread( boost::bind(&boost::asio::io_service::run, &ioService) );
  threadpool.create_thread( boost::bind(&boost::asio::io_service::run, &ioService) );
  
  while (!stop_capture_) {
    // Wait for image. Timeout after 2*FramePeriod = (2000ms/FrameRate)
    
    //INT status = is_WaitEvent(cam_, IS_SET_EVENT_FRAME, 1000);
    INT status = is_WaitForNextImage(cam_, 1000, &img_mem, &img_ID);
    if ( IS_SUCCESS == status) {
      // Assign the task for the thread pool
      ioService.post(boost::bind(&Camera::processFrame, this, img_mem, img_ID, size, callback));
    }
    else {
      ROS_INFO("Dropped, %d", status);
      is_CaptureStatus (cam_, IS_CAPTURE_STATUS_INFO_CMD_GET, (void*)&CaptureStatusInfo, sizeof(CaptureStatusInfo));
      ROS_INFO("Capturestatus Total Errors: %d \n  No Mem: %d \n  Conv fail: %d \n  image lock: %d \n  no buf: %d \n  dev not ready: %d \n  transfer fail: %d \n  timeout: %d \n  buf overrun: %d \n  missed img: %d \n capt failed: %d", 
                CaptureStatusInfo.dwCapStatusCnt_Total,
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_NO_DEST_MEM],
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_CONVERSION_FAILED],
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_IMAGE_LOCKED],
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_OUT_OF_BUFFERS],
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_DEVICE_NOT_READY],
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_USB_TRANSFER_FAILED],
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_DEV_TIMEOUT],
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_ETH_BUFFER_OVERRUN],
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_DEV_MISSED_IMAGES],
                CaptureStatusInfo.adwCapStatusCnt_Detail[IS_CAP_STATUS_DEV_FRAME_CAPTURE_FAILED] );
    }
        
    /*if (trigger && ppsLock)
    {
      if (IoStatus[1] == 1)
      {
        //ROS_INFO("img mem at Camera.cpp %p", img_mem);
        //ROS_INFO("imgData.img_mem at Camera.cpp %p", imgData.img_mem);
        {
          frameCount++;
          imgData.count = frameCount;
          boost::mutex::scoped_lock lock(mutex);
          dataList_.push_back(imgData);
          //ROS_INFO("dataList Size %d", dataList_.size());
        }
        ppsLock = false;
        //dataList_.push_back({img_mem, size, ros::Time::now(), getGPIOConfiguration(), getExposure()});
        
        //callback(img_mem, size, ros::Time::now(), getGPIOConfiguration(), getExposure());
      }
    }
    else
    { 
      //ROS_INFO("img mem at Camera.cpp %p", img_mem);
      //ROS_INFO("imgData.img_mem at Camera.cpp %p", imgData.img_mem);
      frameCount++;
      imgData.count = frameCount;
      boost::mutex::scoped_lock lock(mutex);
      dataList_.push_back(imgData);
      //ROS_INFO("dataList Size %d", dataList_.size());
    }*/ 
  }
  // Clear the polled exposure and gain values remaining in the list
  clearExposureGainList();
  
  // Stop thread pool and join threads
  ioService.stop();
  threadpool.join_all();

  // Stop video event
  //checkError(is_DisableEvent(cam_, IS_SET_EVENT_FRAME));
  checkError(is_StopLiveVideo(cam_, IS_WAIT));

  destroyMemoryPool();
  streaming_ = false;
}

void Camera::processFrame(char *img_mem, int img_ID, size_t size, CamCaptureCB callback) {
  ros::Time received = ros::Time::now();
  //double exposure = getExposure();
  //unsigned int gain = getHardwareGain();
  //SaveExposureAndGain();
  
  auto now = std::chrono::system_clock::now();
  auto seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
  auto timer = std::chrono::system_clock::to_time_t(now);
  
  putenv("TZ=Europe/Finland");
  std::tm bt = *std::localtime(&timer);
  
  UEYEIMAGEINFO ImageInfo;
  ros::Time stamp;
  
  if ( IS_SUCCESS == is_GetImageInfo(cam_, img_ID, &ImageInfo, sizeof(ImageInfo)) ) {
    /*ROS_INFO("%02d:%03d, IO: %d, Buffers: %d/%d, Frame number: %llu, Process time: %d, Frame_rate: %f",
    ImageInfo.TimestampSystem.wSecond,
    ImageInfo.TimestampSystem.wMilliseconds,
    ImageInfo.dwIoStatus,
    ImageInfo.dwImageBuffersInUse,
    ImageInfo.dwImageBuffers,
    ImageInfo.u64FrameNumber,
    ImageInfo.dwHostProcessTime,
    dblFPS);*/
    
    if ( bt.tm_sec == ImageInfo.TimestampSystem.wSecond ) { 
      stamp = ros::Time(seconds_since_epoch.count(), ( ImageInfo.TimestampSystem.wMilliseconds ) * 1000000);
    }
    else {
      stamp = ros::Time(seconds_since_epoch.count() - 1, ( ImageInfo.TimestampSystem.wMilliseconds ) * 1000000 );
    }
  
    std::bitset<3> IoStatus (ImageInfo.dwIoStatus);
    double exposure = 0;
    unsigned int gain = 0;
    GetExposureGain( stamp, exposure, gain );
    //ROS_INFO("time elapsed: %f", (received - stamp).nsec/1000000.0 );
    callback(img_mem, size, stamp, !IoStatus[0], exposure); // gain, ImageInfo.u64FrameNumber);
  }
  
  int img_seq_num = GetImageSeqNum(img_mem);
  checkError( is_UnlockSeqBuf(cam_, img_seq_num, img_mem) );
}

INT Camera::GetImageID (char* pbuf) {
  if (!pbuf)
      return 0;
  for (int i = 0; i < img_mem_.size(); i++) {
    if (img_mem_[i] == pbuf)
      return img_mem_id_[i];
  }

  return 0;
}

INT Camera::GetImageSeqNum (char* pbuf) {
  if (!pbuf)
      return 0;
  for (int i = 0; i < img_mem_.size(); i++) {
    if (img_mem_[i] == pbuf)
      return img_mem_seq_num_[i];
  }
  
  return 0;
}

void Camera::SaveExposureAndGain() {
  boost::mutex::scoped_lock lock(mutex);
  ExposureGainList_.push_back({ros::Time::now(), getExposure(), getHardwareGain()});
}

bool Camera::LoadExposureAndGain( ros::Time trigger_time, double& exposure, unsigned int& gain) {
  boost::mutex::scoped_lock lock(mutex);
  if ( !ExposureGainList_.empty() ) {
    int placeholder = -1;
    int ms_min = 1e3*1.0/frame_rate_ + 1;
    //ROS_INFO("frame_rate: %f, nsec min: %d", frame_rate_, nsec_min);
    for (int i = 0; i < ExposureGainList_.size(); i++) {
      ros::Duration diff = trigger_time - ExposureGainList_[i].stamp;
      if ( diff.sec == 0 ) {
        int ms = abs( diff.nsec / 1000000);
        if ( ms <= ms_min ) {
          ms_min = ms;
          placeholder = i;
        }
      }
    }
    
    if (placeholder > -1) {
      ROS_INFO("list length: %d, placeholder is %d, ms_min: %d", ExposureGainList_.size(), placeholder, ms_min);
      exposure = ExposureGainList_[placeholder].exposure;
      gain = ExposureGainList_[placeholder].gain;
      ExposureGainList_.erase(ExposureGainList_.begin());
      
      return true;
    }
    else {
      ROS_INFO("NOT FOUND, ms_min: %d", ms_min);
      for (int i = 0; i < ExposureGainList_.size(); i++) {
        ros::Duration diff = trigger_time - ExposureGainList_[i].stamp;
        if ( diff.sec == 0 ) {
          int ms = diff.nsec / 1000000;
          ROS_INFO("place %d: %d ms, %d ns", i, ms, diff.nsec);
        }
      }
      return false;
    }
    
    //exposure = ExposureGainList_[0].exposure;
    //gain = ExposureGainList_[0].gain;
    //ExposureGainList_.erase(ExposureGainList_.begin());
    //return true;
  }
    
  else
    return false;
}

void Camera::clearExposureGainList() {
  boost::mutex::scoped_lock lock(mutex);
  ExposureGainList_.clear();
}

/*bool Camera::removeFirstElementOfExposureGainList()
{
  checkError( is_UnlockSeqBuf(cam_, dataList_[0].imgID, dataList_[0].img_mem) );
  //ROS_INFO("is_UnlockSeqBuf Success");
  boost::mutex::scoped_lock lock(mutex);
  dataList_.erase(dataList_.begin());
}*/

/*bool Camera::getImageDataFromList(char **frame, size_t& size, ros::Time& stamp, int& pps, double& exposure, int& count)
{
  boost::mutex::scoped_lock lock(mutex);
  //ROS_INFO("getImageDataFromList %d", dataList_.size());
  if ( ! dataList_.empty() )
  {
    *frame = dataList_[0].img_mem;
    //ROS_INFO("Frame at datalist at Camera.cpp %p", frame);
    size = dataList_[0].size;
    stamp = dataList_[0].stamp;
    pps = dataList_[0].pps;
    exposure = dataList_[0].exposure;
    count = dataList_[0].count;
    
    return true;
  }
  
  else
    return false;
}

void Camera::clearList()
{
  boost::mutex::scoped_lock lock(mutex);
  dataList_.clear();
}

bool Camera::removeFromList()
{
  checkError( is_UnlockSeqBuf(cam_, dataList_[0].imgID, dataList_[0].img_mem) );
  //ROS_INFO("is_UnlockSeqBuf Success");
  boost::mutex::scoped_lock lock(mutex);
  dataList_.erase(dataList_.begin());
}*/

void Camera::GetExposureGain( ros::Time trigger_time, double& exposure, unsigned int& gain) {
  boost::mutex::scoped_lock lock(mutex);
  if ( !ExposureGainList_.empty() ) {
    //int placeholder = -1;
    //int nsec_min = 1e9*1.0/frame_rate_;
    //ROS_INFO("frame_rate: %f, nsec min: %d", frame_rate_, nsec_min);
    //std::vector<ExposureGainStruct>::reverse_iterator best_it;
    int i = 0;
    int prev = int(round(ExposureGainList_[0].stamp.nsec/1000000.0)) - 1;
    //for (std::vector<ExposureGainStruct>::iterator it = ExposureGainList_.begin(); it != ExposureGainList_.end(); ++it ) {
    for (auto element : ExposureGainList_) {
      int stamp = int(round(element.stamp.nsec/1000000.0));
      //ROS_INFO("list length: %d, element: %d, time: %f", ExposureGainList_.size(), i, round(element.stamp.nsec/1000000.0) );
      i++;
      if ( (stamp - prev) != 1 ) {
        if ( (stamp - prev) != -999 )
          ROS_INFO("ERROR: %d", stamp - prev);
      }
      prev = stamp;
      //ros::Duration diff = trigger_time - element.stamp;
      //ROS_INFO("list length: %d, element: %d, diff: %f", ExposureGainList_.size(), i, diff.nsec/1000000.0 );
      //int nsec = abs( diff.nsec );
      /*if ( diff.nsec < 1000000 ) {
        //best_it = it;
        //ROS_INFO("list length: %d, element: %d, diff: %f", ExposureGainList_.size(), i, diff.nsec/1000000.0 );
        exposure = element.exposure;
        gain = element.gain;
        //i++;
        //ExposureGainList_.erase( ExposureGainList_.begin(), it );
        
        break;
      }*/
    }
  }
}

void Camera::PollExposureGain() {
  bool LOCK = false;
  while (!stop_capture_) {
    //ROS_INFO("LIST SIZE: %d", ExposureGainList_.size() );
    ros::Time now = ros::Time::now();
    int usec = now.nsec % 1000000;
    
    if ( usec < 500000 && !LOCK ) {
      boost::mutex::scoped_lock lock(mutex);
      {
        ExposureGainList_.push_back({now, getExposure(), getHardwareGain()});
      }
      
      LOCK = true;
      //ROS_INFO("IF, Time %d.%09d, %d usecs", now.sec, now.nsec, usec);
    }
    else if ( usec >= 500000 && LOCK) {
      LOCK = false;
    }
    else {
      //ROS_INFO("ELSE, Time %d.%09d, %d usecs", now.sec, now.nsec, usec);
      usleep( 20 );
    }
  }
}

void Camera::startVideoCapture(CamCaptureCB callback)
{
  stream_callback_ = callback;
  //clearList();
  captureThread_ = boost::thread(&Camera::captureThread, this, callback);
  pollExposureGainThread_ = boost::thread(&Camera::PollExposureGain, this);
}
void Camera::stopVideoCapture()
{
  stop_capture_ = true;
  if (pollExposureGainThread_.joinable()) {
    pollExposureGainThread_.join();
  }
  if (captureThread_.joinable()) {
    forceTrigger();
    captureThread_.join();
  }
}
void Camera::restartVideoCapture()
{
  if (streaming_) {
    if (stream_callback_ != NULL) {
      stopVideoCapture();
      startVideoCapture(stream_callback_);
    }
  }
}

} //namespace ueye

/*
Copyright © 2003 - 2025, e-con Systems India Private Limited. All rights reserved.
This file contains proprietary information; they are provided under a license agreement containing restrictions on use
and disclosure and are also protected by copyright, patent, and other intellectual and industrial property laws. Please
refer the licensing agreement with e-con Systems to understand the restrictions. Reverse engineering, disassembly, or
decompilation of the Programs is prohibited. Except as may be expressly permitted in your license agreement for these
Programs, no part of these Programs may be reproduced or transmitted in any form or by any means, electronic or
mechanical, for any purpose. The Programs are not intended for use in any inherently dangerous applications. It shall be
the licensee's responsibility to take all appropriate fail-safe, backup, redundancy and other measures to ensure the
safe use of such applications if the Programs are used for such purposes, and we disclaim liability for any damages
caused by such use of the Programs.
*/

#ifdef _WIN32
#pragma once
#define _CRT_SECURE_NO_WARNINGS  // to use scanf instead of scanf_s (scanf_s is not working in Linux).
#include <SDKDDKVer.h>
#include <Windows.h>
#include <conio.h>
#include <tchar.h>

#include <fstream>
#include <mutex>
#include <string>

#include "DepthVista.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/rgbd/depth.hpp"
#include "strsafe.h"
#endif

#ifdef __linux__
#include <pthread.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <mutex>
#include <atomic>
#include "DepthVista.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/rgbd/depth.hpp"
#endif

#define LOG_HIGH_DEBUG 1
#define LOG_CRITICAL_DEBUG 3
#define LOG_ESSENTIAL_DEBUG 7

#define PRE_VGA_WIDTH 640
#define PRE_VGA_HEIGHT 480
#define PRE_RGB_VGA_WIDTH 640
#define PRE_RGB_VGA_HEIGHT 480
#define PRE_RGB_HD_WIDTH 1280
#define PRE_RGB_HD_HEIGHT 720
// Preview RGB Full HD Image Resolution 1920x1080
#define PRE_RGB_FULL_HD_WIDTH 1920
#define PRE_RGB_FULL_HD_HEIGHT 1080

// Preview RGB Full HD Image Resolution 1920x1080
#define PRE_RGB_1200p_WIDTH 1920
#define PRE_RGB_1200p_HEIGHT 1200
#define MAX_FILENAME 240

#define FAR_MODE_MIN 1000
#define FAR_MODE_MAX 6000
#define NEAR_MODE_MIN 200
#define NEAR_MODE_MAX 1200

#define METER_TO_MILLIMETER 1000

#define MIN_AVG_X 4
#define MIN_AVG_Y 4
#define MAX_AVG_X 200
#define MAX_AVG_Y 200
#define EXIT 0

#define SERIAL_NUM_LENGTH 16
#define CONF_THRESHOLD_MAX 500
#define INTEGRATION_TIME_MIN 50
#define INTEGRATION_TIME_MAX 800
#define INTEGRATION_TIME_MAX_VGA_50_FPS 270


// Changed by Abishek on 15/03/2023
// Removed Pcl dependencies for Saving Ply file and replaced with file write
#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

#include <stdio.h>

#include <iostream>
#include <thread>

using namespace std;
using namespace cv;

struct color_point_t {
  float xyz[3];
  uint8_t rgb[3];
  uint16_t confidence;
  color_point_t() : xyz{0.0f, 0.0f, 0.0f}, rgb{0, 0, 0}, confidence{0} {}
};

enum exploreOptions { notInitialize = -1, singleCam = 0, multipleCam = 1 };

// Variable Declarations
uint32_t currentDeviceIndex = 0;
uint32_t maxDeviceIndex = 0;
int16_t exploreOpt = -1;
uint32_t devices;
std::vector<DeviceInfo> gDevicesList;
std::vector<uint16_t> dataModeVec;
std::vector<uint16_t> depthRangeVec;
std::vector<int> multiDev;
std::vector<DeviceInfo> successCam;
std::vector<int> selecteddevindex;
bool previewThreadCtrlFlag[8] = {false};
bool streamThreadCtrlFlag[8] = {false};

ToFFrame ToFIrFrame;
ToFFrame ToFDepthColorMapFrame;
ToFFrame ToFDepthRawFrame;
ToFFrame ToFRGBFrame;

std::vector<Frames> depthVistaFrames;
int commonCamId = -1;
int streaming_index = -1;

Mat Depthcolormap[8];
Mat UYVYFrame[8];
Mat IRFrame[8];
Mat IRRawImg_Y16[8];
Mat rgbFrame[8];
Mat DepthImg_Y16[8];
Mat confidence[8];
uint16_t Depth_min = 0, Depth_max = 4095;
int depth_offset;
char keyPressed = '\0';
bool bPreview[8], bSwitch;
bool saveFrames[8] = {false};
bool startSavingFrames[8] = {false};
bool DepthFrameSaveStatus[8] = {false};
bool IRFrameSaveStatus[8] = {false};
bool RGBFrameSaveStatus[8] = {false};
bool Depthstreamstarted = false;
bool mouseCallbckSync[8] = {false};
int callBckDeviceId[8] = {0};
std::vector<UVCProp> uvc_prop;
char depthFrameFileNameBuf[MAX_FILENAME], IRFrameFileNameBuf[MAX_FILENAME], RGBFrameFileNameBuf[MAX_FILENAME],
    depthConfFrameFileNameBuf[MAX_FILENAME], depthRawFrameFileNameBuf[MAX_FILENAME], IRRawFrameFileNameBuf[MAX_FILENAME];
char PLY3DFileNameBuf[MAX_FILENAME];
mutex guardMutex;
mutex previewMutex[8];
std::vector<DeviceHandle> deviceHandleList;
vector<bool> calibParamObtained;
vector<bool> rgbdMappingflag;
int depth_min_val, depth_max_val;
bool streamStarted = false;
ostringstream depthVistaRGBWindowName[8];
ostringstream depthVistaColorMapWindowName[8];
ostringstream depthVistaIRWindowName[8];
ostringstream confidenceWindowName[8];
int tempDeviceIndex[8];

string post_processing[3] = {"Spatial", "Temporal", "Edge detection"};

std::atomic<bool> g_needRelist{false};

#ifdef _WIN32
bool bDetach = false;
thread previewThread[8];
thread streamThread[8];
HANDLE saveEvent[8];
HANDLE m_hEvent[8];
#elif __linux__
pthread_t previewThread[8];
pthread_t streamThread[8];
struct timespec max_wait;
struct timespec max_wait_console = {0};
pthread_mutex_t wait_mutex[8] = {PTHREAD_MUTEX_INITIALIZER};
pthread_cond_t retrieve_cond[8] = {PTHREAD_COND_INITIALIZER};
#endif

// Function Declarations
bool listDevices();
bool exploreCam();
int tofControlSettings();
int postProcessing();
bool bPreviewSet(int tid, bool bPrev, int deviceIndex);
void startStream();
void mouseCallBck(int event, int xVar, int yVar, int flages, void* userdata);
int savePLYfiles(IntrinsicCalibParams Intrinsic, Mat Depth, Mat rgbFrame, Mat conFrame, char* fileName);
#ifdef __linux__
void* stream(void* arg);
void* preview(void* arg);
#elif _WIN32
void stream(int deviceIndex);
void preview(int deviceIndex);
#endif
bool selectStreamingMode();
bool selectGetDepthValue();
bool uvcControlMenu();
int depthRangeMenu();
int imuEmbedDataMenu();
int irGainMenu();
int tofCoringMenu();
int filtersMenu(uint8_t filterID);
int flyingPixelFilter();
int staticDefect();
int coldDefect();
int hotDefect();
bool rgbdMapping();
void registerNotificationCb(int notificationId, DeviceHandle handle);
int getTemperature();
int exposureControlMenu();
int serialNo();
int laserSafetyMenu();
int fpsControlMenu();

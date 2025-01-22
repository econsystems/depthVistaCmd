#ifdef _WIN32
#pragma once
#define _CRT_SECURE_NO_WARNINGS   // to use scanf instead of scanf_s (scanf_s is not working in Linux).
#include <Windows.h>
#include <SDKDDKVer.h>
#include <tchar.h>
#include "strsafe.h"
#include <string>
#include <conio.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/rgbd/depth.hpp"
#include "opencv2/opencv.hpp"
#include "DepthVistaSDK/DepthVista.h"
#include <fstream>
#include <mutex>
#endif



#ifdef __linux__
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/rgbd/depth.hpp"
#include "opencv2/opencv.hpp"
#include "DepthVista.h"
#include <string.h>
#include <pthread.h>
#include <mutex>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <fstream>
#endif

#define LOG_HIGH_DEBUG              1
#define LOG_CRITICAL_DEBUG          3
#define LOG_ESSENTIAL_DEBUG         7

#define PRE_VGA_WIDTH					640
#define PRE_VGA_HEIGHT					480
#define PRE_RGB_VGA_WIDTH				640
#define PRE_RGB_VGA_HEIGHT				480
#define PRE_RGB_HD_WIDTH				1280
#define PRE_RGB_HD_HEIGHT				720
// Preview RGB Full HD Image Resolution 1920x1080
#define PRE_RGB_FULL_HD_WIDTH			1920
#define PRE_RGB_FULL_HD_HEIGHT			1080

// Preview RGB Full HD Image Resolution 1920x1080
#define PRE_RGB_1200p_WIDTH				1920
#define PRE_RGB_1200p_HEIGHT			1200
#define MAX_FILENAME					240

#define FAR_MODE_MIN					1000
#define FAR_MODE_MAX					6000
#define NEAR_MODE_MIN					200
#define NEAR_MODE_MAX					1200

#define METER_TO_MILLIMETER				1000

#define MIN_AVG_X                           4
#define MIN_AVG_Y                           4
#define MAX_AVG_X                           600
#define MAX_AVG_Y                           400

#include <stdio.h>
#include <iostream>
#include <thread>

using namespace std;
using namespace cv;


struct color_point_t
{
        float xyz[3];
        uint8_t rgb[3];
};

enum exploreOptions
{
	notInitialize = -1,
	singleCam = 0,
	multipleCam  = 1
};


#define EXIT				0
string post_processing[3] = { "Spatial","Temporal","Edge detection" };

//Variable Declarations
uint32_t				currentDeviceIndex = 0;
uint32_t				maxDeviceIndex = 0;
int16_t					exploreOpt = -1;
uint32_t                devices;
DeviceInfo*				gDevicesList;
DataMode                dataMode;
uint16_t                depthRange = 1;
uint8_t                 planarizationState[8] = { 0 };
uint8_t                 undistortionState[8] = { 0 };
bool					previewThreadCtrlFlag[8] = {false};
bool					streamThreadCtrlFlag[8] = {false};

ToFFrame                ToFIrFrame;
ToFFrame                ToFDepthColorMapFrame;
ToFFrame                ToFDepthRawFrame;
ToFFrame                ToFRGBFrame;

Frames                  depthVistaFrames;

Mat                     Depthcolormap[8];
Mat                     UYVYFrame[8];
Mat                     IRFrame[8];
Mat                     rgbFrame[8];
Mat                     DepthImg_Y16[8];


uint16_t                Depth_min = 0, Depth_max = 4095;
int                     depth_offset;
char                    keyPressed = '\0';
bool                    bPreview[8], bSwitch;
bool                    saveFrames[8] = { false  };
bool                    startSavingFrames[8] = { false  };
bool                    DepthFrameSaveStatus[8] = { false };
bool                    IRFrameSaveStatus[8] = { false };
bool                    RGBFrameSaveStatus[8] = { false};
bool                    Depthstreamstarted = false;
bool					mouseCallbckSync[8] = {false};
int 					callBckDeviceId[8] = {0};
std::vector<UVCProp>	uvc_prop;

char                    depthFrameFileNameBuf[MAX_FILENAME], IRFrameFileNameBuf[MAX_FILENAME], RGBFrameFileNameBuf[MAX_FILENAME], depthRawFrameFileNameBuf[MAX_FILENAME];
char                    PLY3DFileNameBuf[MAX_FILENAME];

mutex								guardMutex;
mutex								previewMutex[8];

std::vector <DeviceHandle> deviceHandleList;
vector<bool>						calibParamObtained;
vector<bool>						rgbdMappingflag;

std::vector<color_point_t>			points;
int									depth_min_val,depth_max_val;

bool        streamStarted = false;
ostringstream			depthVistaRGBWindowName[8];
ostringstream			depthVistaColorMapWindowName[8];
ostringstream			depthVistaIRWindowName[8];



#ifdef _WIN32

bool					bDetach = false;
thread					previewThread[8];
thread					streamThread[8];


HANDLE					saveEvent[8];
HANDLE					m_hEvent[8];



#elif __linux__

pthread_t				previewThread[8];
pthread_t				streamThread[8];
struct timespec 		max_wait;
pthread_mutex_t 		wait_mutex[8] = {PTHREAD_MUTEX_INITIALIZER};
pthread_cond_t  		retrieve_cond[8]  = {PTHREAD_COND_INITIALIZER};

#endif

int						tempDeviceIndex[8];
//Function Declarations
bool listDevices();
bool exploreCam();
bool tofControlSettings();
int postProcessing();
void registerNotificationCb(int id, int index);
// void registerFrameCb(int id, int index);

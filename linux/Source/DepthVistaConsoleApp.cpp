#ifdef _WIN32
#pragma once
#define _CRT_SECURE_NO_WARNINGS   // to use scanf instead of scanf_s (scanf_s is not working in Linux).
#include <Windows.h>
#include <SDKDDKVer.h>
#include <tchar.h>
#include <string>
#include <conio.h>
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "DepthVistaSDK/DepthVista.h"
#include <mutex>

#endif

#ifdef __linux__
#include "opencv2/opencv.hpp"
#include "DepthVistaSDK/DepthVista.h"
#include <string.h>
#include <pthread.h>
#include <mutex>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#endif

#define PRE_RGB_VGA_WIDTH 640
#define PRE_RGB_VGA_HEIGHT 480
#define PRE_RGB_HD_WIDTH 1280
#define PRE_RGB_HD_HEIGHT 720
// Preview RGB Full HD Image Resolution 1920x1080
#define PRE_RGB_FULL_HD_WIDTH 1920
#define PRE_RGB_FULL_HD_HEIGHT 1080

// Preview RGB Full HD Image Resolution 1920x1080
#define PRE_RGB_ORIGINAL_WIDTH 1920
#define PRE_RGB_ORIGINAL_HEIGHT 1200
#include <stdio.h>
#include <iostream>
#include <thread>

using namespace std;
using namespace cv;

#define EXIT				0
#define SDK_VERSION			"1.0.0.1"

//Variable Declarations
uint32_t                devices;
DeviceInfo*				gDevicesList;
DataMode                cDataMode = Depth_IR_RGB_VGA_Mode;
uint16_t                cDepthRange = 1;
uint8_t                 cPlanarizationState = 0;
ToFFrame                ToFIrFrame;
ToFFrame                ToFDepthColorMapFrame;
ToFFrame                ToFDepthRawFrame;
ToFFrame                ToFRGBFrame;
Mat                     Depthcolormap = cv::Mat(480, 640, CV_8UC3);
Mat                     UYVYFrame = cv::Mat(480, 640, CV_8UC2);
Mat                     IRFrame = cv::Mat(480, 640, CV_16UC1);
Mat                     rgbFrame;
Mat                     DepthImg_Y16 = cv::Mat(480, 640, CV_16UC1);
uint16_t                Depth_min = 0, Depth_max = 4095;
int                     depth_range;
char                    keyPressed = '\0';
bool                    bPreview, bSwitch;
bool                    saveFrames = false;
bool                    startSavingFrames = false;
bool                    DepthFrameSaveStatus = false;
bool                    IRFrameSaveStatus = false;
bool                    RGBFrameSaveStatus = false;
char                    depthFrameFileNameBuf[240], IRFrameFileNameBuf[240], RGBFrameFileNameBuf[240], depthRawFrameFileNameBuf[240];
mutex                   guardMutex;

#ifdef _WIN32

bool					bDetach = false;
thread					previewThread;
HANDLE					saveEvent;

#elif __linux__

pthread_t				threadId;

#endif


//Function Declarations
bool listDevices();
bool exploreCam();

/**
* @brief 				Setting and Checking preview state
* @param[in]            tid	Id based on which the function acts as setting or checking preview status
* @param[in]            bPrev   preview status
* @return				bool	the state of preview
*/
bool bPreviewSet(int tid, bool bPrev)
{
    std::lock_guard<std::mutex> guard(guardMutex);

    if (tid == 1)
    {
        bPreview = bPrev;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return bPreview;
}

/**
* @brief 		Starting preview with current dataMode and depthRange
* @return		void
*/
void startStream()
{
    if (cDataMode == Depth_IR_RGB_HD_Mode || cDataMode == RGB_HD_Mode)
    {
        UYVYFrame.release();
        UYVYFrame = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_8UC2);
    }
    else if (cDataMode == RGB_Full_HD_Mode)
    {
        UYVYFrame.release();
        UYVYFrame = cv::Mat(PRE_RGB_FULL_HD_HEIGHT, PRE_RGB_FULL_HD_WIDTH, CV_8UC2);
    }
    else if (cDataMode == RGB_Original_Mode)
    {
        UYVYFrame.release();
        UYVYFrame = cv::Mat(PRE_RGB_ORIGINAL_HEIGHT, PRE_RGB_ORIGINAL_WIDTH, CV_8UC2);
    }
    else if (cDataMode == Depth_IR_RGB_VGA_Mode || cDataMode == RGB_VGA_Mode)
    {
        UYVYFrame.release();
        UYVYFrame = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_8UC2);
    }

    if (cDepthRange == 0)
    {
        Depth_min = 200;
        Depth_max = 1500;
    }
    else
    {
        Depth_min = 500;
        Depth_max = 3250;
    }
    depth_range = (Depth_max - Depth_min) * 0.20;
    UpdateColorMap(Depth_min, Depth_max + depth_range, 4);
    bSwitch = false;
    bPreviewSet(1, true);
}


#ifdef _WIN32
/**
* @brief 		Preview function for Windows
* @return		void
*/
void preview()
{
    uint32_t failCount = 0;
    struct timeval tv, res;
    struct tm tm;

    while (true)
    {
        while (bPreviewSet(2, true))
        {
            if (GetNextFrame() == Result::Ok) {
                if (saveFrames)
                {
                    startSavingFrames = true;
                    time_t t = time(0);
                    localtime_s(&tm, &t);
                }
                // To get Depth Color Map frame
                if (cDataMode != IR_Mode && cDataMode <= Depth_IR_RGB_HD_Mode) {
                    if (GetToFFrame(FrameType::DepthColorMap, &ToFDepthColorMapFrame) > 0) {
                        Depthcolormap.data = (uchar*)ToFDepthColorMapFrame.frame_data;
                        namedWindow("DepthVista DepthColorMap", WINDOW_AUTOSIZE);
                        imshow("DepthVista DepthColorMap", Depthcolormap);

                        if (startSavingFrames)
                        {
                            sprintf(depthFrameFileNameBuf, "DepthVista_Depth_%d_%d_%d_%d_%d_%d.bmp", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
                            DepthFrameSaveStatus = imwrite(depthFrameFileNameBuf, Depthcolormap);
							if (DepthFrameSaveStatus)
								cout << endl << "DepthColorMap frame is successfully saved as " << depthFrameFileNameBuf << endl;
							else
								cout << endl << "Saving DepthColorMap Frame Failed" << endl;
                            if (GetToFFrame(FrameType::DepthRawFrame, &ToFDepthRawFrame) > 0) {
                                DepthImg_Y16.data = ToFDepthRawFrame.frame_data;
                            }
                            sprintf(depthRawFrameFileNameBuf, "DepthVista_raw_%d_%d_%d_%d_%d_%d.raw", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
                            FILE* fp;
                            fp = fopen(depthRawFrameFileNameBuf, "wb+");
                            fwrite(DepthImg_Y16.data, DepthImg_Y16.total() * DepthImg_Y16.channels() * DepthImg_Y16.elemSize1(), 1, fp);
                            fclose(fp);
							cout << endl << "Raw frame is successfully saved as " << depthRawFrameFileNameBuf << endl;

                        }
                    }
                }
                // To get RGB frame
                if (cDataMode >= Depth_IR_RGB_VGA_Mode && cDataMode != Raw_Mode) {
                    if (GetToFFrame(FrameType::RGBFrame, &ToFRGBFrame) > 0) {
                        UYVYFrame.data = ToFRGBFrame.frame_data;
                        cv::cvtColor(UYVYFrame, rgbFrame, cv::COLOR_YUV2BGR_UYVY);
                        namedWindow("DepthVista RGB Frame", WINDOW_AUTOSIZE);
                        imshow("DepthVista RGB Frame", rgbFrame);

                        if (startSavingFrames)
                        {
							sprintf(RGBFrameFileNameBuf, "DepthVista_RGB_%d_%d_%d_%d_%d_%d.bmp", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
                            RGBFrameSaveStatus = imwrite(RGBFrameFileNameBuf, rgbFrame);
							if (RGBFrameSaveStatus)
								cout << endl << "RGB frame is successfully saved as " << RGBFrameFileNameBuf << endl;
							else
								cout << endl << "Saving RGB Frame Failed" << endl;
                        }
                    }
                }
                // To get IR frame
                if (cDataMode != Depth_Mode && cDataMode <= Depth_IR_RGB_HD_Mode) {
                    if (GetToFFrame(FrameType::IRPreviewFrame, &ToFIrFrame) > 0) {
                        IRFrame.data = ToFIrFrame.frame_data;
                        namedWindow("DepthVista IR Frame", WINDOW_AUTOSIZE);
                        imshow("DepthVista IR Frame", IRFrame);

                        if (startSavingFrames)
                        {
							sprintf(IRFrameFileNameBuf, "DepthVista_IR_%d_%d_%d_%d_%d_%d.png", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
                            IRFrameSaveStatus = imwrite(IRFrameFileNameBuf, IRFrame);
							if (IRFrameSaveStatus)
								cout << endl << "IR frame is successfully saved as " << IRFrameFileNameBuf << endl;
							else
								cout << endl << "Saving IR Frame Failed" << endl;
                        }
                    }
                }
                if (saveFrames)
                {
                    startSavingFrames = saveFrames = false;
                                        SetEvent(saveEvent);
                }
            }

            keyPressed = waitKey(5);
            while (bSwitch)
            {
                bSwitch = false;
                destroyAllWindows();
            }
        }
    }
}
#elif __linux__

/**
* @brief 		Preview function for Linux
* @return		void
*/
void *stream(void* arg)
{
    uint32_t failCount = 0;
    struct timeval tv, res;
    struct tm* tm;
    char cwd[256];



    while (true)
    {
        while (bPreviewSet(2, true))
        {
            if (GetNextFrame() == Result::Ok) {
                if (saveFrames)
                {
                    startSavingFrames = true;
                    time_t t = time(0);
                    tm = localtime(&t);
                    getcwd(cwd, sizeof(cwd));
                }
                // To get Depth Color Map frame
                if (cDataMode != IR_Mode && cDataMode <= Depth_IR_RGB_HD_Mode) {
                    if (GetToFFrame(FrameType::DepthColorMap, &ToFDepthColorMapFrame) > 0) {
                        Depthcolormap.data = (uchar*)ToFDepthColorMapFrame.frame_data;
                        namedWindow("DepthVista DepthColorMap", WINDOW_AUTOSIZE);
                        imshow("DepthVista DepthColorMap", Depthcolormap);

                        if (startSavingFrames)
                        {
                            sprintf(depthFrameFileNameBuf, "%s/DepthVista_Depth_%d_%d_%d_%d_%d_%d.bmp", cwd, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
                            DepthFrameSaveStatus = imwrite(depthFrameFileNameBuf, Depthcolormap);
                            if(DepthFrameSaveStatus)
                                cout << endl << "DepthColorMap frame is successfully saved as " << depthFrameFileNameBuf << endl;
                            else
								cout << endl << "Saving DepthColorMap Frame Failed" << endl;
                            if (GetToFFrame(FrameType::DepthRawFrame, &ToFDepthRawFrame) > 0) {
                                DepthImg_Y16.data = ToFDepthRawFrame.frame_data;
                            }
							sprintf(depthRawFrameFileNameBuf, "%s/DepthVista_raw_%d_%d_%d_%d_%d_%d.raw", cwd, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
                            FILE* fp;
                            fp = fopen(depthRawFrameFileNameBuf, "wb+");
                            fwrite(DepthImg_Y16.data, DepthImg_Y16.total() * DepthImg_Y16.channels() * DepthImg_Y16.elemSize1(), 1, fp);
                            fclose(fp);
                            cout << endl << "Raw Depth frame is successfully saved as " << depthRawFrameFileNameBuf << endl;

                        }
                    }
                }
                // To get RGB frame
                if (cDataMode >= Depth_IR_RGB_VGA_Mode && cDataMode != Raw_Mode) {
                    if (GetToFFrame(FrameType::RGBFrame, &ToFRGBFrame) > 0) {
                        UYVYFrame.data = ToFRGBFrame.frame_data;
                        cv::cvtColor(UYVYFrame, rgbFrame, cv::COLOR_YUV2BGR_UYVY);
                        namedWindow("DepthVista RGB Frame", WINDOW_AUTOSIZE);
                        imshow("DepthVista RGB Frame", rgbFrame);

                        if (startSavingFrames)
                        {
                            sprintf(RGBFrameFileNameBuf, "%s/DepthVista_RGB_%d_%d_%d_%d_%d_%d.bmp", cwd, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
							RGBFrameSaveStatus = imwrite(RGBFrameFileNameBuf, rgbFrame);
                            if(RGBFrameSaveStatus)
								cout << endl << "RGB frame is successfully saved as " << RGBFrameFileNameBuf << endl;
                            else
								cout << endl << "Saving RGB Frame Failed" << endl;
                        }
                    }
                }
                // To get IR frame
                if (cDataMode != Depth_Mode && cDataMode <= Depth_IR_RGB_HD_Mode) {
                    if (GetToFFrame(FrameType::IRPreviewFrame, &ToFIrFrame) > 0) {
                        IRFrame.data = ToFIrFrame.frame_data;
                        namedWindow("DepthVista IR Frame", WINDOW_AUTOSIZE);
                        imshow("DepthVista IR Frame", IRFrame);

                        if (startSavingFrames)
                        {
							sprintf(IRFrameFileNameBuf, "%s/DepthVista_IR_%d_%d_%d_%d_%d_%d.png", cwd, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
							IRFrameSaveStatus = imwrite(IRFrameFileNameBuf, IRFrame);
                            if (IRFrameSaveStatus)
								cout << endl << "IR frame is successfully saved as " << IRFrameFileNameBuf << endl;
                            else
								cout << endl << "Saving IR Frame Failed" << endl;
                        }
                    }
                }
                if (saveFrames)
                {
                    startSavingFrames = saveFrames = false;
                }
            }

            keyPressed = waitKey(5);
            while (bSwitch)
            {
                bSwitch = false;
                destroyAllWindows();
            }
        }
    }
}
#endif





int main()
{
    //Basic Introduction about the Application
    cout << endl << "e-con's Sample Application for DepthVista " << endl;
    cout << endl << "Demonstrates the working of e-con's DepthVistaSDK" << endl;
    cout << endl << "\t" << "DepthVista SDK-Version = " << SDK_VERSION << "\n\n";

    //Initialize DepthVista
    if (Initialize() < 0) {
        pError();
    }

    //Open a Camera Device
    if (!(listDevices()))
    {
        cout << endl << "List Devices Information Failed" << endl;
        cout << endl << '\t' << "Press Any key to exit the Application: " << '\t';
#ifdef _WIN32
        _getch();
#endif
        return 0;
    }

#ifdef _WIN32
	previewThread = thread(&preview);
#elif __linux__
    pthread_create(&threadId, NULL, stream, NULL);
#endif

    if (!(exploreCam()))
    {
        cout << endl << "Camera Exploration Failed" << endl;
        return 0;
    }

#ifdef _WIN32
	previewThread.detach();
#endif

    bPreviewSet(1, false);
    if (CloseDevice() > 0)
    {
        if (DeInitialize() > 0)
        {
            destroyAllWindows();
        }
    }

    return 0;
}


/**
* @brief 		Listing and Opening Camera device
* @return		bool    return true on successful listing and opening device, else retuns fail.
*/
bool listDevices()
{
    int camId = -1;
    //List total Number of Devices
    bSwitch = true;
    devices = 0;
    if (GetDeviceCount(&devices) < 0) {
        pError("GetDeviceCount");
        return false;
    }

    if (devices < 0)
    {
        cout << endl << "No Camera Devices Connected to the port" << endl;
        return false;
    }
    else
    {
        gDevicesList = new DeviceInfo[devices];
        if (GetDeviceListInfo(devices, gDevicesList) < 0) {
            pError("GetDeviceListInfo");
            cout << endl << "Device Information couldn't be Retrieved" << endl;
        }
    }

    cout << endl << "Number of Camera Devices Connected to the Port : " << devices << endl;
    cout << endl << "Camera Devices Connected to the PC Port : " << endl << endl;
    cout << '\t' << "0 - Exit" << endl;

    //List the Camera Names
    for (int eachDevice = 0; eachDevice < devices; eachDevice++)
    {
        cout << '\t' << eachDevice + 1 << " . " << gDevicesList->deviceName << endl;
    }

    while ((camId < 0) || (camId > devices))
    {
        printf("\n Pick a Camera Device to Explore : \t");
        scanf("%d", &camId);
        while (getchar() != '\n' && getchar() != EOF)
        {

        }
    }

    switch (camId)
    {
    case EXIT:
        bPreviewSet(1, false);
#ifdef _WIN32
        if (bDetach)
			previewThread.detach();
#endif
        if (DeInitialize() > 0)
        {
            destroyAllWindows();
        }

        exit(0);
        break;

    default:
        bSwitch = true;
        bPreviewSet(1, false);
        if (IsOpened() > 0)
        {
            if (CloseDevice() > 0)
            {
                if (DeInitialize() > 0)
                {
                    destroyAllWindows();
                }
            }
        }
        if (OpenDevice(camId - 1) < 0) {
            pError("OpenDevice:");
            return false;
        }
        break;
    }
#ifdef _WIN32
    bDetach = true;
	saveEvent = CreateEvent(NULL, FALSE, FALSE, L"frameCaptureEvent");
#endif
    return true;
}


/**
* @brief 		Setting Streaming Mode of the Device
* @return		bool    return true on successfully setting the stream mode of the device, else retuns fail.
*/
bool selectStreamingMode()
{
    cout << endl << "Total Number of Streaming Modes Supported by the Camera:  " << '\t' << "9" << endl;

    cout << '\t' << "0 - Exit" << endl;
    cout << '\t' << "1 - Back" << endl;
    cout << '\t' << "2 - Main Menu" << endl;
    cout << '\t' << "3 - Depth IR Mode" << endl;
    cout << '\t' << "4 - Depth Mode" << endl;
    cout << '\t' << "5 - IR Mode" << endl;
    cout << '\t' << "6 - Depth IR RGB(VGA)Mode" << endl;
    cout << '\t' << "7 - Depth IR RGB(HD)Mode" << endl;
    cout << '\t' << "8 - RGB(VGA) Mode" << endl;
    cout << '\t' << "9 - RGB(HD) Mode" << endl;
    cout << '\t' << "10 - RGB(Full HD) Mode" << endl;
    cout << '\t' << "11 - RGB(1200p) Mode" << endl;
    int option = -1;
    while ((option < 0) || (option >= 12))
    {
        printf("\n Pick a Relevant Streaming Mode: \t");
        scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF)
        {
        }
    }

    switch (option)
    {
    case EXIT:
        bSwitch = true;
        bPreviewSet(1, false);
#ifdef _WIN32
        if (bDetach)
			previewThread.detach();
#endif
        if (CloseDevice() > 0)
        {
            if (DeInitialize() > 0)
            {
                destroyAllWindows();

            }
        }
        exit(0);

    case 1:
    case 2:
        exploreCam();
        break;

    case 3:
        bSwitch = true;
        bPreviewSet(1, false);
        if (cDataMode != option - 3)
        {
            if (SetDataMode((DataMode)(option - 3)) < 0) {
                pError("SetDataMode");
                return false;
            }
            cDataMode = (DataMode)(option - 3);
        }
        break;

    default:
        bSwitch = true;
        bPreviewSet(1, false);
        if (cDataMode != option - 2)
        {
            if (SetDataMode((DataMode)(option - 2)) < 0) {
                pError("SetDataMode");
                return false;
            }
            cDataMode = (DataMode)(option - 2);
        }
        break;
    }
    startStream();
    return true;
}


/**
* @brief 		Setting Depth Range of the Device
* @return		bool    return true on successfully setting the depthRange of the device, else retuns fail.
*/
bool depthRangeMenu()
{
    cout << endl << "Total Number of Depth Range Supported by the Camera:  " << '\t' << "2" << endl;

    cout << '\t' << "0 - Exit" << endl;
    cout << '\t' << "1 - Back" << endl;
    cout << '\t' << "2 - Main Menu" << endl;
    cout << '\t' << "3 - Near Mode" << endl;
    cout << '\t' << "4 - Far Mode" << endl;

    int option = -1;
    while ((option < 0) || (option >= 5))
    {
        printf("\n Pick a Relevant Depth Mode: \t");
        scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF)
        {
        }
    }

    switch (option)
    {
    case EXIT:
        bSwitch = true;
        bPreviewSet(1, false);
        #ifdef _WIN32
        if (bDetach)
			previewThread.detach();
#endif
        if (CloseDevice() > 0)
        {
            if (DeInitialize() > 0)
            {
                destroyAllWindows();
            }
        }
        exit(0);

    case 1:
    case 2:
        exploreCam();
        break;

    default:
        if (cDepthRange != option - 3 && cDataMode <= Depth_IR_RGB_HD_Mode)
        {
            if (SetDepthRange(option - 3) < 0) {
                pError("SetDepthRange");
                return false;
            }
            cDepthRange = option - 3;
        }
        break;
    }
    if (cDepthRange == 0)
    {
        Depth_min = 200;
        Depth_max = 1500;
    }
    else
    {
        Depth_min = 500;
        Depth_max = 3250;
    }
    depth_range = (Depth_max - Depth_min) * 0.20;
    UpdateColorMap(Depth_min, Depth_max + depth_range, 4);
    return true;
}

/**
* @brief 		Planarizing Depth
* @return		bool    return true on successfully Planarizing Depth, else retuns fail.
*/
bool depthPlanarization()
{
    cout << endl;
    cout << '\t' << "0 - Exit" << endl;
    cout << '\t' << "1 - Back" << endl;
    cout << '\t' << "2 - Main Menu" << endl;
    cout << '\t' << "3 - Planarization OFF" << endl;
    cout << '\t' << "4 - Planarization ON" << endl;

    int option = -1;
    while ((option < 0) || (option >= 5))
    {
        printf("\n Pick a Relevant Option: \t");
        scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF)
        {
        }
    }

    switch (option)
    {
    case EXIT:
        bSwitch = true;
        bPreviewSet(1, false);
        #ifdef _WIN32
        if (bDetach)
			previewThread.detach();
#endif
        if (CloseDevice() > 0)
        {
            if (DeInitialize() > 0)
            {
                destroyAllWindows();

            }
        }
        exit(0);

    case 1:
    case 2:
        exploreCam();
        break;

    default:
        if (cPlanarizationState != option - 3)
        {
            if (SetPlanarization(option - 3) < 0) {
                pError("SetPlanarization");
                return false;
            }
            cPlanarizationState = option - 3;
        }
        break;
    }
    return true;
}

/**
* @brief 		Explore the controls of Device
* @return		bool    return true on success, else retuns fail.
*/
bool exploreCam()
{
    uint8_t gMajorVersion = 0, gMinorVersion1 = 0;
    uint16_t gMinorVersion2 = 0, gMinorVersion3 = 0;
    uint64_t uniqueID;
    while (true)
    {
        int choice = -1;
        if (GetDataMode(&cDataMode) < 0) {
            pError("GetDataMode");
        }
        if (GetDepthRange(&cDepthRange) < 0) {
            pError("GetDepthRange");
        }
        cout << endl << '\t' << "0 - Exit" << endl;
        cout << '\t' << "1 - Back" << endl;
        cout << '\t' << "2 - Streaming Mode" << endl;
        cout << '\t' << "3 - Depth Range" << endl;
        cout << '\t' << "4 - Planarization" << endl;
        cout << '\t' << "5 - Capture Frames" << endl;
        cout << '\t' << "6 - Unique ID" << endl;
        cout << '\t' << "7 - Read Firmware Version" << endl;


        while ((choice < 0) || (choice >= 8))
        {
            printf("\n Pick a Relevant Choice of Camera Properties : \t");
            scanf("%d", &choice);
            while (getchar() != '\n' && getchar() != EOF)
            {
            }
        }

        switch (choice)
        {
        case EXIT:
            bPreviewSet(1, false);
            #ifdef _WIN32
            if (bDetach)
				previewThread.detach();
#endif
            if (CloseDevice() > 0)
            {
                if (DeInitialize() > 0)
                {
                    destroyAllWindows();
                }
            }
            exit(0);

        case 1:
            if (!listDevices())
            {
                cout << endl << "List Devices Information failed" << endl;
                return false;
            }
            cout << endl << "Connected Devices were Listed" << endl;
            break;

        case 2:
            if (!selectStreamingMode())
            {
                cout << endl << "Data Mode Selection Failed" << endl;
                return false;
            }
            cout << endl << "Data Mode Selected" << endl;
            break;

        case 3:
            if (!depthRangeMenu())
            {
                cout << endl << "Depth Range Selection Failed" << endl;
                return false;
            }
            cout << endl << "Depth Range Selected" << endl;
            break;

        case 4:
            if (!depthPlanarization())
            {
                cout << endl << "Depth Planarization Failed" << endl;
                return false;
            }
            cout << endl << "Depth Planarization success" << endl;
            break;

        case 5:
            if (bPreviewSet(3, true))
            {
                memset(depthFrameFileNameBuf, 0, 240);
                memset(RGBFrameFileNameBuf, 0, 240);
                memset(IRFrameFileNameBuf, 0, 240);
                memset(depthRawFrameFileNameBuf, 0, 240);
                saveFrames = true;
                #ifdef _WIN32
				while (1)
				{
					DWORD dWait = WaitForSingleObject(saveEvent, 100);
					if (dWait != WAIT_TIMEOUT)
					{
						break;
					}
				}
				cout << "After wait" << endl;
#elif __linux__
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
#endif
            }
            else
                cout << endl << "Image Capture Failed, Please start preview before capturing frames" << endl;
            cout << endl << "Frame Capture ends" << endl;

            break;

        case 6:
            GetUniqueID(&uniqueID);
            cout << endl << "Unique ID of the Camera is " << uniqueID << endl;
            break;

        case 7:
            readFirmwareVersion(&gMajorVersion, &gMinorVersion1, &gMinorVersion2, &gMinorVersion3);
            cout << endl << "Firmware Version : " << (uint16_t)gMajorVersion << "." << (uint16_t)gMinorVersion1 << "." << gMinorVersion2 << "." << gMinorVersion3 << endl;
            break;

        }
    }

    return true;
}

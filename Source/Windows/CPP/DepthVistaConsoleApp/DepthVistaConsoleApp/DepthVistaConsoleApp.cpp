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

#include "DepthVistaConsoleApp.h"

/**
 * @brief 				Setting and Checking preview state
 * @param[in]            tid	Id based on which the function acts as setting or checking preview status
 * @param[in]            bPrev   preview status
 * @return				bool	the state of preview
 */
bool bPreviewSet(int tid, bool bPrev, int deviceIndex) {
  const std::lock_guard<std::mutex> guard(guardMutex);

  if (tid == 1) {
    bPreview[deviceIndex] = bPrev;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return bPreview[deviceIndex];
}

/**
 * @brief 		Starting preview with current dataMode and depthRange
 * @return		void
 */
void startStream() {
  for (int deviceCnt = 0; deviceCnt < static_cast<int>(deviceHandleList.size()); deviceCnt++) {
    int camAccessId = -1;
    int index_for_buffers = -1;
    DeviceInfo devInfo;
    //depthRangeVec.clear();

    if (exploreOpt == exploreOptions::multipleCam) {
      camAccessId = deviceCnt;
      index_for_buffers = deviceCnt;
      devInfo = successCam[deviceCnt];
    } else if (exploreOpt == exploreOptions::singleCam) {
      camAccessId = static_cast<int>(currentDeviceIndex);
      index_for_buffers = 0;
      devInfo = gDevicesList[currentDeviceIndex];
    }

    if (devInfo.devType == DeviceType::ITOF_USB || devInfo.devType == DeviceType::ITOF_GMSL) {
      if (dataModeVec[index_for_buffers] == Depth_IR_Conf_HD_Mode) {
        if (IRFrame[index_for_buffers].empty()) {
          // std::cout<<"ir empty true hd\n";
          IRFrame[index_for_buffers].release();
          IRFrame[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
        }
        if (IRRawImg_Y16[index_for_buffers].empty()) {
          IRRawImg_Y16[index_for_buffers].release();
          IRRawImg_Y16[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
        }
        if (DepthImg_Y16[index_for_buffers].empty()) {
          DepthImg_Y16[index_for_buffers].release();
          DepthImg_Y16[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
        }
        if (Depthcolormap[index_for_buffers].empty()) {
          // std::cout<<"depth colormap empty true hd\n";
          Depthcolormap[index_for_buffers].release();
          Depthcolormap[index_for_buffers] = cv::Mat(960, 1280, CV_8UC3);
        }
        if (confidence[index_for_buffers].empty()) {
          confidence[index_for_buffers].release();
          confidence[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
        }
        if (IRRawImg_Y16[index_for_buffers].empty()) {
          IRRawImg_Y16[index_for_buffers].release();
          IRRawImg_Y16[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
        }
      } else if (dataModeVec[index_for_buffers] == Depth_IR_Conf_Mode) {
        if (IRFrame[index_for_buffers].empty()) {
          // std::cout<<"ir empty true vga\n";
          IRFrame[index_for_buffers].release();
          IRFrame[index_for_buffers] = cv::Mat(480, 640, CV_16UC1);
        }
        if (DepthImg_Y16[index_for_buffers].empty()) {
          // std::cout<<"depth raw empty true vga\n";
          DepthImg_Y16[index_for_buffers].release();
          DepthImg_Y16[index_for_buffers] = cv::Mat(480, 640, CV_16UC1);
        }
        if (Depthcolormap[index_for_buffers].empty()) {
          // std::cout<<"depth colormap empty true vga\n";
          Depthcolormap[index_for_buffers].release();
          Depthcolormap[index_for_buffers] = cv::Mat(480, 640, CV_8UC3);
        }
        if (confidence[index_for_buffers].empty()) {
          confidence[index_for_buffers].release();
          confidence[index_for_buffers] = cv::Mat(480, 640, CV_16UC1);
        }
        if (IRRawImg_Y16[index_for_buffers].empty()) {
          IRRawImg_Y16[index_for_buffers].release();
          IRRawImg_Y16[index_for_buffers] = cv::Mat(480, 640, CV_16UC1);
        }
      }
    } 
    else {
      if (GetDepthRange(deviceHandleList[deviceCnt], &depthRangeVec[deviceCnt]) < 0) {
        pError("GetDepthRange");
      }
      if (depthRangeVec[deviceCnt] == DepthRange::NearRange) {
        depth_min_val = Depth_min = NEAR_MODE_MIN;
        depth_max_val = Depth_max = NEAR_MODE_MAX;
      } else if (depthRangeVec[deviceCnt] == DepthRange::FarRange) {
        depth_min_val = Depth_min = FAR_MODE_MIN;
        depth_max_val = Depth_max = FAR_MODE_MAX;
      }
      if (IRFrame[index_for_buffers].empty()) {
        IRFrame[index_for_buffers] = cv::Mat(PRE_VGA_HEIGHT, PRE_VGA_WIDTH, CV_16UC1);
      }
      if (DepthImg_Y16[index_for_buffers].empty()) {
        DepthImg_Y16[index_for_buffers] = cv::Mat(PRE_VGA_HEIGHT, PRE_VGA_WIDTH, CV_16UC1);
      }
      if (Depthcolormap[index_for_buffers].empty()) {
        Depthcolormap[index_for_buffers].release();
        Depthcolormap[index_for_buffers] = cv::Mat(PRE_VGA_HEIGHT, PRE_VGA_WIDTH, CV_8UC3);
      }
    }

    if (dataModeVec[index_for_buffers] == RGB_HD_Mode || dataModeVec[index_for_buffers] == Depth_IR_RGB_HD_Mode) {
      UYVYFrame[index_for_buffers].release();
      UYVYFrame[index_for_buffers] = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_8UC2);
      if (dataModeVec[index_for_buffers] == Depth_IR_RGB_HD_Mode) {
        if (rgbdMappingflag[camAccessId]) {
          Depthcolormap[index_for_buffers].release();
          Depthcolormap[index_for_buffers] = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_8UC3);
          DepthImg_Y16[index_for_buffers].release();
          DepthImg_Y16[index_for_buffers] = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_16UC1);
        } else {
          Depthcolormap[index_for_buffers].release();
          Depthcolormap[index_for_buffers] = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_8UC3);
          DepthImg_Y16[index_for_buffers].release();
          DepthImg_Y16[index_for_buffers] = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_16UC1);
        }
      }
    } else if (dataModeVec[index_for_buffers] == RGB_Full_HD_Mode) {
      UYVYFrame[index_for_buffers].release();
      UYVYFrame[index_for_buffers] = cv::Mat(PRE_RGB_FULL_HD_HEIGHT, PRE_RGB_FULL_HD_WIDTH, CV_8UC2);
    } else if (dataModeVec[index_for_buffers] == RGB_1200p_Mode) {
      UYVYFrame[index_for_buffers].release();
      UYVYFrame[index_for_buffers] = cv::Mat(PRE_RGB_1200p_HEIGHT, PRE_RGB_1200p_WIDTH, CV_8UC2);
    } else if (dataModeVec[index_for_buffers] != Depth_IR_RGB_HD_Mode &&
               dataModeVec[index_for_buffers] <= RGB_VGA_Mode) {
      UYVYFrame[index_for_buffers].release();
      UYVYFrame[index_for_buffers] = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_8UC2);
      if (devInfo.devType == DeviceType::ITOF_USB || devInfo.devType == DeviceType::ITOF_GMSL) {
        if (dataModeVec[index_for_buffers] == Depth_IR_Conf_Mode) {
          Depthcolormap[index_for_buffers].release();
          Depthcolormap[index_for_buffers] = cv::Mat(480, 640, CV_8UC3);
          DepthImg_Y16[index_for_buffers].release();
          DepthImg_Y16[index_for_buffers] = cv::Mat(480, 640, CV_16UC1);
          IRFrame[index_for_buffers].release();
          IRFrame[index_for_buffers] = cv::Mat(480, 640, CV_16UC1);
          IRRawImg_Y16[index_for_buffers].release();
          IRRawImg_Y16[index_for_buffers] = cv::Mat(480, 640, CV_16UC1);
          confidence[index_for_buffers].release();
          confidence[index_for_buffers] = cv::Mat(480, 640, CV_16UC1);
          depth_min_val = Depth_min = 200;
          depth_max_val = Depth_max = 6000;
        } else if (dataModeVec[index_for_buffers] == Depth_IR_Conf_HD_Mode) {
          Depthcolormap[index_for_buffers].release();
          Depthcolormap[index_for_buffers] = cv::Mat(960, 1280, CV_8UC3);
          DepthImg_Y16[index_for_buffers].release();
          DepthImg_Y16[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
          IRFrame[index_for_buffers].release();
          IRFrame[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
          IRRawImg_Y16[index_for_buffers].release();
          IRRawImg_Y16[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
          confidence[index_for_buffers].release();
          confidence[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
          depth_min_val = Depth_min = 200;
          depth_max_val = Depth_max = 2000;
        }
      } else {
        Depthcolormap[index_for_buffers].release();
        Depthcolormap[index_for_buffers] = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_8UC3);
        DepthImg_Y16[index_for_buffers].release();
        DepthImg_Y16[index_for_buffers] = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_16UC1);
      }
    } else if (dataModeVec[index_for_buffers] == Depth_IR_Conf_HD_Mode) {  // Depth_IR_Conf_HD_Mode

      if (devInfo.devType == DeviceType::ITOF_USB || devInfo.devType == DeviceType::ITOF_GMSL) {
        Depthcolormap[index_for_buffers].release();
        Depthcolormap[index_for_buffers] = cv::Mat(960, 1280, CV_8UC3);
        DepthImg_Y16[index_for_buffers].release();
        DepthImg_Y16[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
        IRFrame[index_for_buffers].release();
        IRFrame[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
        IRRawImg_Y16[index_for_buffers].release();
        IRRawImg_Y16[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
        confidence[index_for_buffers].release();
        confidence[index_for_buffers] = cv::Mat(960, 1280, CV_16UC1);
        depth_min_val = Depth_min = 200;
        depth_max_val = Depth_max = 2000;
      }
    }
    /*if (GetDepthRange(deviceHandleList[deviceCnt], &depthRangeVec[deviceCnt]) < 0) {
      pError("GetDepthRange");
    }
    if (depthRangeVec[deviceCnt] == DepthRange::NearRange) {
      depth_min_val = Depth_min = NEAR_MODE_MIN;
      depth_max_val = Depth_max = NEAR_MODE_MAX;
    } else if (depthRangeVec[deviceCnt] == DepthRange::FarRange) {
      depth_min_val = Depth_min = FAR_MODE_MIN;
      depth_max_val = Depth_max = FAR_MODE_MAX;
    }*/
    depth_offset = static_cast<int>((Depth_max - Depth_min) * 0.20);
    UpdateColorMap(deviceHandleList[deviceCnt], Depth_min, Depth_max + depth_offset, 4);
    bSwitch = false;
    bPreviewSet(1, true, index_for_buffers);
  }
}

void mouseCallBck(int event, int xVar, int yVar, int flages, void* userdata) {
  const int deviceId = *static_cast<int*>(userdata);
  flages = 1;
  if (event == EVENT_LBUTTONDOWN) {
    DepthPtr liveDepthPtr;
    liveDepthPtr.X = xVar;
    liveDepthPtr.Y = yVar;
    SetDepthPos(deviceHandleList[deviceId], liveDepthPtr);
    mouseCallbckSync[deviceId] = false;
  }
}

/**
* @brief 		Reads the RGBD Calibration data from the Device
* @Params		Mat unregisteredIntrinsic	:		Intrinsic data of the unregistered cam matrix
                                                                Mat registeredIntrinsic		:
Intrinsic data of the registered cam matrix Mat registeredDistortion	:		Distortion data of the
registered cam matrix Mat transform				:		Transform matrix from Unregistered to
registered cam Mat unregisteredDepth		:		Unregistered depth data Mat rgbFrame :
Registered rgb data char* fileName				:		PLY file name
* @return		int		 1 on success
*						-1 on no valid depth data found for creating ply file
                                                                                                -2 on invalid input
parameters.
*/
int savePLYfiles(IntrinsicCalibParams Intrinsic, Mat Depth, Mat rgbFrame, Mat confFrame, char* fileName) {
  std::vector<color_point_t> points;
  if (Depth.empty()) {
    return -2;
  }
  double focalLengthx = NAN;
  double focalLengthy = NAN;
  double principlePointx = NAN;
  double principlePointy = NAN;

  focalLengthx = Intrinsic.fx;
  focalLengthy = Intrinsic.fy;
  principlePointx = Intrinsic.cx;
  principlePointy = Intrinsic.cy;

  cv::Mat DepthFloat;
  const float zoom_factor = 1.0;
  float depthValue = NAN;
  uint16_t raw_depth = 0;
  float xVal = NAN;
  float yVal = NAN;
  float zVal = NAN;
  color_point_t pclPoint;
  points.clear();
  Depth.convertTo(DepthFloat, CV_32F);

  for (int deviceCnt = 0; deviceCnt < static_cast<int>(deviceHandleList.size()); deviceCnt++) {
    int camAccessId = -1;
    int index_for_buffers = -1;
    DeviceInfo devInfo;

    if (exploreOpt == exploreOptions::multipleCam) {
      camAccessId = deviceCnt;
      index_for_buffers = deviceCnt;
      devInfo = successCam[deviceCnt];
    } else if (exploreOpt == exploreOptions::singleCam) {
      camAccessId = static_cast<int>(currentDeviceIndex);
      index_for_buffers = 0;
      devInfo = gDevicesList[currentDeviceIndex];
    }

    for (int row = 0; row < DepthFloat.rows; row++) {
      for (int col = 0; col < DepthFloat.cols; col++) {
        depthValue = DepthFloat.at<float>(row, col) / zoom_factor;
        raw_depth = Depth.at<uint16_t>(row, col);

        //std::cout << "raw_depth: " << raw_depth << " min: " << depth_min_val << " max: " << depth_max_val << "\n";

        if (raw_depth > depth_max_val || raw_depth < depth_min_val) {
          continue;
        }

        zVal = depthValue;
        xVal = static_cast<float>((col - principlePointx) * depthValue / focalLengthx);
        yVal = static_cast<float>((row - principlePointy) * depthValue / focalLengthy);
        

        pclPoint.xyz[0] = xVal / METER_TO_MILLIMETER;
        pclPoint.xyz[1] = -yVal / METER_TO_MILLIMETER;
        pclPoint.xyz[2] = -zVal / METER_TO_MILLIMETER;
        pclPoint.rgb[0] = rgbFrame.at<cv::Vec3b>(row, col)[0];
        pclPoint.rgb[1] = rgbFrame.at<cv::Vec3b>(row, col)[1];
        pclPoint.rgb[2] = rgbFrame.at<cv::Vec3b>(row, col)[2];
        if (devInfo.devType == DeviceType::ITOF_GMSL || devInfo.devType == DeviceType::ITOF_USB)
            pclPoint.confidence = confFrame.at<uint16_t>(row, col);
        points.push_back(pclPoint);
      }
    }
    //imwrite("Test.png", rgbFrame * 16);
    if (points.empty()) {
      std::cout << "empty data : " << points.data() <<"\n";
      return -1;
    }

    // save to the ply file
      std::ofstream ofs(fileName);  // text mode first
      ofs << PLY_START_HEADER << "\n";
      ofs << PLY_ASCII << "\n";
      ofs << PLY_ELEMENT_VERTEX << " " << points.size() << "\n";
      ofs << "property float x" << "\n";
      ofs << "property float y" << "\n";
      ofs << "property float z" << "\n";
      ofs << "property uchar red" << "\n";
      ofs << "property uchar green" << "\n";
      ofs << "property uchar blue" << "\n";
      if (devInfo.devType == DeviceType::ITOF_GMSL || devInfo.devType == DeviceType::ITOF_USB)
        ofs << "property ushort intensity" << "\n";
      ofs << PLY_END_HEADER << "\n";
      std::stringstream stringStream;
      for (const auto& point : points) {
        // image data is BGR
        stringStream << point.xyz[0] << " " << point.xyz[1] << " " << point.xyz[2];
        stringStream << " " << static_cast<float>(point.rgb[2]) << " " << static_cast<float>(point.rgb[1]) << " "
                     << static_cast<float>(point.rgb[0]);
        if (devInfo.devType == DeviceType::ITOF_GMSL || devInfo.devType == DeviceType::ITOF_USB)
            stringStream << " " << static_cast<float>(point.confidence);
        stringStream << "\n";
      }
      ofs.write(stringStream.str().c_str(), static_cast<std::streamsize>(stringStream.str().length()));
      ofs.close();
    }
  
  return 1;
}

#ifdef _WIN32
/**
 * @brief 		Preview function for Windows
 * @return		void
 */

void stream(int deviceIndex) {
  while (streamThreadCtrlFlag[deviceIndex]) {
    while (bPreviewSet(2, true, deviceIndex)) {
      Result res = GetNextFrame(deviceHandleList[deviceIndex]);
      if (res == Result::Ok) {
        // To get RGB frame
        previewMutex[deviceIndex].lock();
        Result GetFramesRes = GetFrames(deviceHandleList[deviceIndex], &depthVistaFrames[deviceIndex]);
        int fpsVal = GetFramesPerSecond(deviceHandleList[deviceIndex]);
        if (GetFramesRes > 0) {
          if ((dataModeVec[deviceIndex] >= Depth_IR_RGB_VGA_Mode || dataModeVec[deviceIndex] == IR_RGB_HD_Mode ||
               dataModeVec[deviceIndex] == Depth_RGB_HD_Mode) &&
              (dataModeVec[deviceIndex] != Depth_IR_Conf_HD_Mode && dataModeVec[deviceIndex] != Depth_Conf_175Mhz &&
               dataModeVec[deviceIndex] != Depth_Conf_200Mhz)) {
            if (depthVistaFrames[deviceIndex].rgb.frame_data != NULL && UYVYFrame[deviceIndex].data != NULL) {
              if (depthVistaFrames[deviceIndex].rgb.size <=
                  (UYVYFrame[deviceIndex].total() * UYVYFrame[deviceIndex].elemSize())) {
                memcpy(UYVYFrame[deviceIndex].data, depthVistaFrames[deviceIndex].rgb.frame_data,
                       depthVistaFrames[deviceIndex].rgb.size);
              }
            }
          }
          if ((dataModeVec[deviceIndex] != IR_Mode && dataModeVec[deviceIndex] <= Depth_IR_RGB_HD_Mode) ||
              dataModeVec[deviceIndex] == Depth_RGB_HD_Mode || dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode ||
              dataModeVec[deviceIndex] == Depth_IR_Conf_Mode || dataModeVec[deviceIndex] == Depth_Conf_175Mhz ||
              dataModeVec[deviceIndex] == Depth_Conf_200Mhz) {
            Depthstreamstarted = true;
            if (depthVistaFrames[deviceIndex].depth_colormap.frame_data != NULL &&
                Depthcolormap[deviceIndex].data != NULL) {
              if (depthVistaFrames[deviceIndex].depth_colormap.size <=
                  (Depthcolormap[deviceIndex].total() * Depthcolormap[deviceIndex].elemSize())) {
                memcpy(Depthcolormap[deviceIndex].data, depthVistaFrames[deviceIndex].depth_colormap.frame_data,
                       depthVistaFrames[deviceIndex].depth_colormap.size);
              }
            }
            if (depthVistaFrames[deviceIndex].raw_depth.frame_data != NULL && DepthImg_Y16[deviceIndex].data != NULL) {
              if (depthVistaFrames[deviceIndex].raw_depth.size <=
                  (DepthImg_Y16[deviceIndex].total() * DepthImg_Y16[deviceIndex].elemSize())) {
                memcpy(DepthImg_Y16[deviceIndex].data, depthVistaFrames[deviceIndex].raw_depth.frame_data,
                       depthVistaFrames[deviceIndex].raw_depth.size);
              }
            }
            if (dataModeVec[deviceIndex] == Depth_IR_Conf_Mode || dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode ||
                dataModeVec[deviceIndex] == Depth_Conf_175Mhz || dataModeVec[deviceIndex] == Depth_Conf_200Mhz) {
              if (depthVistaFrames[deviceIndex].confidence_frame.frame_data != NULL &&
                  confidence[deviceIndex].data != NULL) {
                if (depthVistaFrames[deviceIndex].confidence_frame.size <=
                    (confidence[deviceIndex].total() * confidence[deviceIndex].elemSize())) {
                  memcpy(confidence[deviceIndex].data, depthVistaFrames[deviceIndex].confidence_frame.frame_data,
                         depthVistaFrames[deviceIndex].confidence_frame.size);
                }
              }
            }
          }
          if ((dataModeVec[deviceIndex] != Depth_Mode && dataModeVec[deviceIndex] <= Depth_IR_RGB_HD_Mode) ||
              dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode) {
            if (depthVistaFrames[deviceIndex].ir.frame_data != NULL && IRFrame[deviceIndex].data != NULL) {
              if (depthVistaFrames[deviceIndex].ir.size <=
                  (IRFrame[deviceIndex].total() * IRFrame[deviceIndex].elemSize())) {
                memcpy(IRFrame[deviceIndex].data, depthVistaFrames[deviceIndex].ir.frame_data,
                       depthVistaFrames[deviceIndex].ir.size);
              }
            }
            if (depthVistaFrames[deviceIndex].ir.frame_data != NULL && IRRawImg_Y16[deviceIndex].data != NULL) {
              if (depthVistaFrames[deviceIndex].ir.size <=
                  (IRRawImg_Y16[deviceIndex].total() * IRRawImg_Y16[deviceIndex].elemSize())) {
                memcpy(IRRawImg_Y16[deviceIndex].data, depthVistaFrames[deviceIndex].ir.frame_data,
                       depthVistaFrames[deviceIndex].ir.size);
              }
            }
          }
        }

        previewMutex[deviceIndex].unlock();
        if (streamStarted == false) {
          streamStarted = true;
        }
        SetEvent(m_hEvent[deviceIndex]);
      } else {
        PrintLog(LOG_ESSENTIAL_DEBUG, "\nDepthVistaConsole:Getnextframe failed\n");
      }
    }
    Depthstreamstarted = false;
  }
}

void preview(int deviceIndex) {
  uint32_t failCount = 0;
  struct timeval tv, res;
  struct tm tm;

  bool rgbWindowOpen = false;
  bool rgbwindowcreated = false;
  bool colorMapWindowOpen = false;
  bool irWindowOpen = false;

  while (previewThreadCtrlFlag[deviceIndex]) {
    while (bPreviewSet(2, true, deviceIndex)) {
      if (streamStarted) {
        if (saveFrames[deviceIndex]) {
          startSavingFrames[deviceIndex] = true;
          time_t t = time(0);
          localtime_s(&tm, &t);
        }

        DWORD dWait = WaitForSingleObject(m_hEvent[deviceIndex], 600);
        if (dWait == WAIT_TIMEOUT) {
          PrintLog(LOG_ESSENTIAL_DEBUG, "\nDepthVistaConsole:TimeoutError in renderthread");
          continue;
        } else {
          // To get RGB frame
          if (previewMutex[deviceIndex].try_lock()) {
            PrintLog(LOG_HIGH_DEBUG,
                     "\nDepthVistaConsole:Data inside preview : " + std::to_string(dataModeVec[deviceIndex]));
            if ((dataModeVec[deviceIndex] >= Depth_IR_RGB_VGA_Mode || dataModeVec[deviceIndex] == IR_RGB_HD_Mode ||
                 dataModeVec[deviceIndex] == Depth_RGB_HD_Mode) &&
                dataModeVec[deviceIndex] && !UYVYFrame[deviceIndex].empty() &&
                dataModeVec[deviceIndex] != Depth_IR_Conf_HD_Mode && dataModeVec[deviceIndex] != Depth_Conf_175Mhz &&
                dataModeVec[deviceIndex] != Depth_Conf_200Mhz) {
              cv::cvtColor(UYVYFrame[deviceIndex], rgbFrame[deviceIndex], cv::COLOR_YUV2BGR_UYVY);

              if (rgbWindowOpen) {
                try {
                  cv::destroyWindow(depthVistaRGBWindowName[deviceIndex].str().c_str());
                  rgbWindowOpen = false;
                  rgbwindowcreated = true;
                } catch (const cv::Exception& e) {
                  std::cerr << "OpenCV destroyWindow error: " << e.what() << std::endl;
                }
              }

              depthVistaRGBWindowName[deviceIndex].str("");
              depthVistaRGBWindowName[deviceIndex].clear();
              depthVistaRGBWindowName[deviceIndex] << "DepthVista RGB Frame " << deviceIndex;
              namedWindow(depthVistaRGBWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);

              imshow(depthVistaRGBWindowName[deviceIndex].str().c_str(), rgbFrame[deviceIndex]);

              if (startSavingFrames[deviceIndex]) {
                sprintf(RGBFrameFileNameBuf, "DepthVistaCam%d_RGB_%d_%d_%d_%d_%d_%d.bmp", deviceIndex,
                        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
                RGBFrameSaveStatus[deviceIndex] = imwrite(RGBFrameFileNameBuf, rgbFrame[deviceIndex]);
                if (RGBFrameSaveStatus[deviceIndex]) {
                  std::cout << "\n" << "RGB frame is successfully saved as " << RGBFrameFileNameBuf << "\n";
                } else {
                  std::cout << "\n" << "Saving RGB Frame Failed" << "\n";
                }
              }
            } else if (rgbwindowcreated) {
              destroyWindow(depthVistaRGBWindowName[deviceIndex].str().c_str());
              rgbWindowOpen = false;
              rgbwindowcreated = false;
            }

            // To get Depth Color Map frame and confidence frame
            if ((dataModeVec[deviceIndex] != IR_Mode && dataModeVec[deviceIndex] <= Depth_IR_RGB_HD_Mode) ||
                dataModeVec[deviceIndex] == Depth_RGB_HD_Mode || dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode ||
                dataModeVec[deviceIndex] == Depth_Conf_175Mhz || dataModeVec[deviceIndex] == Depth_Conf_200Mhz) {
              Depthstreamstarted = true;
              if (!Depthcolormap[deviceIndex].empty()) {
                depthVistaColorMapWindowName[deviceIndex].str("");
                depthVistaColorMapWindowName[deviceIndex].clear();
                depthVistaColorMapWindowName[deviceIndex] << "DepthVista Color Map Frame " << deviceIndex;
                namedWindow(depthVistaColorMapWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);
                if (colorMapWindowOpen) {
                  try {
                    cv::destroyWindow(depthVistaColorMapWindowName[deviceIndex].str().c_str());
                    colorMapWindowOpen = false;
                  } catch (const cv::Exception& e) {
                    std::cerr << "OpenCV destroyWindow error: " << e.what() << std::endl;
                  }
                }
                imshow(depthVistaColorMapWindowName[deviceIndex].str().c_str(), Depthcolormap[deviceIndex]);
                if (!mouseCallbckSync[deviceIndex]) {
                  callBckDeviceId[deviceIndex] = deviceIndex;
                  mouseCallbckSync[deviceIndex] = true;
                }
                setMouseCallback(depthVistaColorMapWindowName[deviceIndex].str(), mouseCallBck,
                                 static_cast<void*>(&callBckDeviceId[deviceIndex]));
              }

              if (startSavingFrames[deviceIndex]) {
                sprintf(depthFrameFileNameBuf, "DepthVistaCam%d_Depth_%d_%d_%d_%d_%d_%d.bmp", deviceIndex,
                        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
                DepthFrameSaveStatus[deviceIndex] = imwrite(depthFrameFileNameBuf, Depthcolormap[deviceIndex]);
                if (DepthFrameSaveStatus[deviceIndex]) {
                  std::cout << "\n" << "DepthColorMap frame is successfully saved as " << depthFrameFileNameBuf << "\n";
                } else {
                  std::cout << "\n" << "Saving DepthColorMap Frame Failed" << "\n";
                }

                sprintf(depthRawFrameFileNameBuf, "DepthVistaCam%d_raw_%d_%d_%d_%d_%d_%d.raw", deviceIndex,
                        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

                const std::unique_ptr<FILE, decltype(&fclose)> rawFrameFilePtr(
                    fopen(static_cast<char*>(depthRawFrameFileNameBuf), "wb+"), fclose);
                if (rawFrameFilePtr) {
                  fwrite(DepthImg_Y16[deviceIndex].data,
                         DepthImg_Y16[deviceIndex].total() * DepthImg_Y16[deviceIndex].channels() *
                             DepthImg_Y16[deviceIndex].elemSize1(),
                         1, rawFrameFilePtr.get());
                  std::cout << "\n"
                            << "Raw Depth frame is successfully saved as "
                            << static_cast<char*>(depthRawFrameFileNameBuf) << "\n";
                } else {
                  std::cout << "\n" << "unable to open depth raw frame for writing" << "\n";
                }

                if (dataModeVec[deviceIndex] == Depth_IR_Conf_Mode ||
                    (dataModeVec[deviceIndex] >= Depth_IR_Conf_HD_Mode && dataModeVec[deviceIndex] <= Depth_Conf_200Mhz)){
                 
                  sprintf(depthConfFrameFileNameBuf, "DepthVistaCam%d_confidence_%d_%d_%d_%d_%d_%d.raw", deviceIndex,
                          tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

                  const std::unique_ptr<FILE, decltype(&fclose)> confFrameFilePtr(
                      fopen(static_cast<char*>(depthConfFrameFileNameBuf), "wb+"), fclose);
                  if (confFrameFilePtr) {
                    fwrite(confidence[deviceIndex].data,
                           confidence[deviceIndex].total() * confidence[deviceIndex].channels() *
                               confidence[deviceIndex].elemSize1(),
                           1, confFrameFilePtr.get());

                    cout << "\n"
                         << "confidence frame is successfully saved as "
                         << static_cast<char*>(depthConfFrameFileNameBuf) << "\n";
                  } else {
                    cout << "\n" << "unable to open depth conf frame for writing" << "\n";
                  }
                }

                sprintf(PLY3DFileNameBuf, "DepthVistaCam%d_PLY_%d_%d_%d_%d_%d_%d.ply", deviceIndex, tm.tm_year + 1900,
                        tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
                CalibrationParams devCalibrationData;
                Result readCalibRes = GetDeviceCalibrationParams(deviceHandleList[deviceIndex], &devCalibrationData);
                if (readCalibRes == Result::Ok) {
                  if (rgbdMappingflag[deviceIndex]) {
                    if (dataModeVec[deviceIndex] == DataMode::Depth_IR_RGB_HD_Mode ||
                        dataModeVec[deviceIndex] == DataMode::Depth_RGB_HD_Mode) {
                      std::thread saveThread([=]() {
                        savePLYfiles(devCalibrationData.rgbCamHDIntrinsic, DepthImg_Y16[deviceIndex],
                                     rgbFrame[deviceIndex], confidence[deviceIndex], PLY3DFileNameBuf);
                      });
                      saveThread.detach();
                    } else {
                      std::thread saveThread([=]() {
                        savePLYfiles(devCalibrationData.rgbCamVGAIntrinsic, DepthImg_Y16[deviceIndex],
                                     rgbFrame[deviceIndex], confidence[deviceIndex], PLY3DFileNameBuf);
                      });
                      saveThread.detach();
                    }
                  } else {
                    if (dataModeVec[deviceIndex] >= DataMode::Depth_IR_Conf_HD_Mode &&
                         dataModeVec[deviceIndex] <= DataMode::Depth_Conf_200Mhz) {
                      std::thread saveThread([=]() {
                        savePLYfiles(devCalibrationData.depthCamHDIntrinsic, DepthImg_Y16[deviceIndex],
                                     Depthcolormap[deviceIndex], confidence[deviceIndex], PLY3DFileNameBuf);
                      });
                      saveThread.detach();
                    } else {
                      std::thread saveThread([=]() {
                        savePLYfiles(devCalibrationData.depthCamVGAIntrinsic, DepthImg_Y16[deviceIndex],
                                     Depthcolormap[deviceIndex], confidence[deviceIndex], PLY3DFileNameBuf);
                      });
                      saveThread.detach();
                    }
                  }
                  std::cout << "\n" << "3D PLY is saved as " << PLY3DFileNameBuf << "\n";

                } else {
                  std::cout << "3D files are not saved because Calibration Data is Not Found in the device\n";
                }
              }

            } else if (colorMapWindowOpen) {
              destroyWindow(depthVistaColorMapWindowName[deviceIndex].str().c_str());
              colorMapWindowOpen = false;
            }
            // To get IR frame
            if ((dataModeVec[deviceIndex] != Depth_Mode && dataModeVec[deviceIndex] <= Depth_IR_RGB_HD_Mode &&
                 !IRFrame[deviceIndex].empty()) ||
                dataModeVec[deviceIndex] == IR_RGB_HD_Mode || dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode) {
              depthVistaIRWindowName[deviceIndex].str("");
              depthVistaIRWindowName[deviceIndex].clear();
              depthVistaIRWindowName[deviceIndex] << "DepthVista IR Frame " << deviceIndex;
              namedWindow(depthVistaIRWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);

              if (irWindowOpen) {
                try {
                  cv::destroyWindow(depthVistaIRWindowName[deviceIndex].str().c_str());
                  irWindowOpen = false;
                } catch (const cv::Exception& e) {
                  std::cerr << "OpenCV destroyWindow error: " << e.what() << std::endl;
                }
              }

              imshow(depthVistaIRWindowName[deviceIndex].str().c_str(), IRFrame[deviceIndex]);

              if (startSavingFrames[deviceIndex]) {
                sprintf(IRFrameFileNameBuf, "DepthVistaCam%d_IR_%d_%d_%d_%d_%d_%d.png", deviceIndex, tm.tm_year + 1900,
                        tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
                IRFrameSaveStatus[deviceIndex] = imwrite(IRFrameFileNameBuf, IRFrame[deviceIndex]);
                if (IRFrameSaveStatus[deviceIndex])
                  std::cout << "\n" << "IR frame is successfully saved as " << IRFrameFileNameBuf << "\n";
                else
                  std::cout << "\n" << "Saving IR Frame Failed" << "\n";

                sprintf(IRRawFrameFileNameBuf, "DepthVistaCam%d_IR_raw_%d_%d_%d_%d_%d_%d.raw", deviceIndex,
                        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

                const std::unique_ptr<FILE, decltype(&fclose)> rawIRFrameFilePtr(
                    fopen(static_cast<char*>(IRRawFrameFileNameBuf), "wb+"), fclose);
                if (rawIRFrameFilePtr) {
                  fwrite(IRRawImg_Y16[deviceIndex].data,
                         IRRawImg_Y16[deviceIndex].total() * IRRawImg_Y16[deviceIndex].channels() *
                             IRRawImg_Y16[deviceIndex].elemSize1(),
                         1, rawIRFrameFilePtr.get());
                  std::cout << "\n"
                            << "Raw IR frame is successfully saved as " << static_cast<char*>(IRRawFrameFileNameBuf)
                            << "\n";
                } else {
                  std::cout << "\n" << "unable to open IR raw frame for writing" << "\n";
                }
              }
            } else if (irWindowOpen) {
              destroyWindow(depthVistaIRWindowName[deviceIndex].str().c_str());
              irWindowOpen = false;
            }

            previewMutex[deviceIndex].unlock();
            if (startSavingFrames[deviceIndex]) {
              startSavingFrames[deviceIndex] = saveFrames[deviceIndex] = false;
              PrintLog(LOG_CRITICAL_DEBUG, "\nDepthVistaConsole:Inside startSavingFrames saveFrames[" +
                                               std::to_string(deviceIndex) +
                                               "] : " + std::to_string(saveFrames[deviceIndex]));
              SetEvent(saveEvent[deviceIndex]);
            }

          } else {
            PrintLog(LOG_ESSENTIAL_DEBUG, "\nDepthVistaConsole:Trylock failed ");
          }
        }
      }

      keyPressed = waitKey(5);
      while (bSwitch) {
        bSwitch = false;
        //destroyAllWindows();
      }
    }
  }
  Depthstreamstarted = false;
}

#elif __linux__
void* stream(void* arg) {
  const int deviceIndex = *static_cast<int*>(arg);
  int fps = 0;
  int baseTemperature = 0;
  int laserTemperature = 0;
  Result getTemperatureResult = Result::Others;
  struct timeval time_val = {0};
  struct timeval time_val_m = {0};
  struct timeval res = {0};
  gettimeofday(&time_val_m, nullptr);
  while (streamThreadCtrlFlag[deviceIndex]) {
    while (bPreviewSet(2, true, deviceIndex)) {
      gettimeofday(&time_val, nullptr);
      timersub(&time_val, &time_val_m, &res);
      if (res.tv_sec != 0) {
        fps = GetFramesPerSecond(deviceHandleList[deviceIndex]);
        PrintLog(LOG_ESSENTIAL_DEBUG,
                 "\nDepthVistaConsole:FPS for index " + to_string(deviceIndex) + " : " + to_string(fps));

        getTemperatureResult = GetTemperature(deviceHandleList[deviceIndex], &baseTemperature, &laserTemperature);
        if (getTemperatureResult == Result::Ok) {
          PrintLog(LOG_ESSENTIAL_DEBUG, "\nDepthVistaConsole:Temperature for index " + to_string(deviceIndex) +
                                            " : Base ----> " + to_string(baseTemperature) + " & Laser ----> " +
                                            to_string(laserTemperature));
        } else {
          PrintLog(LOG_ESSENTIAL_DEBUG, "\nDepthVistaConsole:Get Temperature failed with return value of " +
                                            to_string(getTemperatureResult));
        }
        time_val_m = time_val;
      }
      const Result GetNextFrameRes = GetNextFrame(deviceHandleList[deviceIndex]);
      if (GetNextFrameRes == Result::Ok) {
        previewMutex[deviceIndex].lock();
        const Result GetFramesRes = GetFrames(deviceHandleList[deviceIndex], &depthVistaFrames[deviceIndex]);
        if (GetFramesRes > 0) {
          if (dataModeVec[deviceIndex] >= Depth_IR_RGB_VGA_Mode && (dataModeVec[deviceIndex] != Depth_IR_C1_C2_Mode) &&
              (dataModeVec[deviceIndex] != Depth_IR_Conf_HD_Mode) && (dataModeVec[deviceIndex] != Depth_Conf_175Mhz) &&
              (dataModeVec[deviceIndex] != Depth_Conf_200Mhz)) {
            memcpy(UYVYFrame[deviceIndex].data, depthVistaFrames[deviceIndex].rgb.frame_data,
                   depthVistaFrames[deviceIndex].rgb.size);
          }
          if ((dataModeVec[deviceIndex] != IR_Mode && dataModeVec[deviceIndex] <= Depth_IR_RGB_HD_Mode) ||
              (dataModeVec[deviceIndex] >= Depth_IR_C1_C2_Mode &&
               dataModeVec[deviceIndex] <= Depth_IR_C1_C2_RGB_HD_Mode) ||
              dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode || dataModeVec[deviceIndex] == Depth_Conf_175Mhz ||
              dataModeVec[deviceIndex] == Depth_Conf_200Mhz) {
            Depthstreamstarted = true;
            memcpy(DepthImg_Y16[deviceIndex].data, depthVistaFrames[deviceIndex].raw_depth.frame_data,
                   depthVistaFrames[deviceIndex].raw_depth.size);
            memcpy(Depthcolormap[deviceIndex].data, depthVistaFrames[deviceIndex].depth_colormap.frame_data,
                   depthVistaFrames[deviceIndex].depth_colormap.size);
            if (dataModeVec[deviceIndex] == Depth_IR_Conf_Mode || dataModeVec[deviceIndex] == Depth_IR_C1_C2_Mode ||
                dataModeVec[deviceIndex] == Depth_IR_C1_C2_RGB_HD_Mode ||
                dataModeVec[deviceIndex] == Depth_IR_C1_RGB_HD_Mode ||
                dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode || dataModeVec[deviceIndex] == Depth_Conf_175Mhz ||
                dataModeVec[deviceIndex] == Depth_Conf_200Mhz) {
              memcpy(confidence[deviceIndex].data, depthVistaFrames[deviceIndex].confidence_frame.frame_data,
                     depthVistaFrames[deviceIndex].confidence_frame.size);
              // confidence[deviceIndex].data = depthVistaFrames[deviceIndex].confidence_frame.frame_data;
            }
          }
          if ((dataModeVec[deviceIndex] != Depth_Mode && dataModeVec[deviceIndex] <= Depth_IR_RGB_HD_Mode) ||
              dataModeVec[deviceIndex] == Depth_IR_C1_C2_Mode ||
              dataModeVec[deviceIndex] == Depth_IR_C1_C2_RGB_HD_Mode ||
              dataModeVec[deviceIndex] == Depth_IR_C1_RGB_HD_Mode || dataModeVec[deviceIndex] == IR_RGB_HD_Mode ||
              dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode) {
            memcpy(IRFrame[deviceIndex].data, depthVistaFrames[deviceIndex].ir.frame_data,
                   depthVistaFrames[deviceIndex].ir.size);
            memcpy(IRRawImg_Y16[deviceIndex].data, depthVistaFrames[deviceIndex].ir.frame_data,
                   depthVistaFrames[deviceIndex].ir.size);
          }
        }

        previewMutex[deviceIndex].unlock();

        if (!streamStarted) {
          streamStarted = true;
        }
        pthread_cond_signal(&retrieve_cond[deviceIndex]);
      }
    }
    Depthstreamstarted = false;
  }
  return nullptr;
}

void* preview(void* arg) {
  std::unique_ptr<struct tm> timeStruct = std::make_unique<struct tm>();
  std::array<char, 256> cwd = {};
  const int deviceIndex = *static_cast<int*>(arg);
  // int camId = -1;
  int camAccessId = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    camAccessId = deviceIndex;
  } else if (exploreOpt == exploreOptions::singleCam) {
    camAccessId = static_cast<int>(currentDeviceIndex);
  }

  while (previewThreadCtrlFlag[deviceIndex]) {
    // cout<<"15\n";
    while (bPreviewSet(2, true, deviceIndex)) {
      // cout<<"16\n";
      if (streamStarted) {
        // cout<<"17\n";
        if (saveFrames[deviceIndex]) {
          startSavingFrames[deviceIndex] = true;
          const time_t currentTime = time(nullptr);
          localtime_r(&currentTime, timeStruct.get());
          if (getcwd(cwd.data(), cwd.size()) == nullptr) {
            std::cerr << "Error: Unable to get the current working directory." << "\n";
          }
        }
        // To get RGB frame
        clock_gettime(CLOCK_REALTIME, &max_wait_console);
        max_wait_console.tv_nsec = max_wait_console.tv_nsec + (static_cast<long>(200) * 1000 * 1000);
        pthread_mutex_lock(&wait_mutex[deviceIndex]);
        const int timed_wait_rv =
            pthread_cond_timedwait(&retrieve_cond[deviceIndex], &wait_mutex[deviceIndex], &max_wait_console);
        if (timed_wait_rv == ETIMEDOUT) {
          pthread_mutex_unlock(&wait_mutex[deviceIndex]);
        } else {
          pthread_mutex_unlock(&wait_mutex[deviceIndex]);
          if (previewMutex[deviceIndex].try_lock()) {
            // cout<<"18\n";
            if (dataModeVec[deviceIndex] >= Depth_IR_RGB_VGA_Mode &&
                dataModeVec[deviceIndex] != Depth_IR_Conf_HD_Mode) {
              // cout<<"19\n";
              if (!UYVYFrame[deviceIndex].empty()) {
                cv::cvtColor(UYVYFrame[deviceIndex], rgbFrame[deviceIndex], cv::COLOR_YUV2BGR_UYVY);
                depthVistaRGBWindowName[deviceIndex].str("");
                depthVistaRGBWindowName[deviceIndex].clear();
                depthVistaRGBWindowName[deviceIndex] << "DepthVista RGB Frame " << deviceIndex + 1;
                namedWindow(depthVistaRGBWindowName[deviceIndex].str(), WINDOW_AUTOSIZE);
                imshow(depthVistaRGBWindowName[deviceIndex].str(), rgbFrame[deviceIndex]);
                if (startSavingFrames[deviceIndex]) {
                  sprintf(static_cast<char*>(RGBFrameFileNameBuf), "%s/DepthVistaCam%d_RGB_%d_%d_%d_%d_%d_%d.bmp",
                          cwd.data(), deviceIndex, timeStruct->tm_year + 1900, timeStruct->tm_mon + 1,
                          timeStruct->tm_mday, timeStruct->tm_hour, timeStruct->tm_min, timeStruct->tm_sec);
                  RGBFrameSaveStatus[deviceIndex] =
                      imwrite(static_cast<char*>(RGBFrameFileNameBuf), rgbFrame[deviceIndex]);
                  if (RGBFrameSaveStatus[deviceIndex]) {
                    cout << "\n"
                         << "RGB frame is successfully saved as " << static_cast<char*>(RGBFrameFileNameBuf) << "\n";
                  } else {
                    cout << "\n" << "Saving RGB Frame Failed" << "\n";
                  }
                }
              }
            }

            // To get Depth Color Map frame
            // cout<<"colormap width and height : "<<Depthcolormap[deviceIndex].cols<<"
            // "<<Depthcolormap[deviceIndex].rows<<std::endl;
            if ((dataModeVec[deviceIndex] != IR_Mode && dataModeVec[deviceIndex] <= Depth_IR_RGB_HD_Mode) ||
                dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode) {
              Depthstreamstarted = true;
              if (!Depthcolormap[deviceIndex].empty()) {
                depthVistaColorMapWindowName[deviceIndex].str("");
                depthVistaColorMapWindowName[deviceIndex].clear();
                depthVistaColorMapWindowName[deviceIndex] << "DepthVista Color Map Frame " << deviceIndex + 1;
                namedWindow(depthVistaColorMapWindowName[deviceIndex].str(), WINDOW_AUTOSIZE);
                
                imshow(depthVistaColorMapWindowName[deviceIndex].str(), Depthcolormap[deviceIndex]);
                if (!mouseCallbckSync[deviceIndex]) {
                  callBckDeviceId[deviceIndex] = deviceIndex;
                  mouseCallbckSync[deviceIndex] = true;
                }
                setMouseCallback(depthVistaColorMapWindowName[deviceIndex].str(), mouseCallBck,
                                 static_cast<void*>(&callBckDeviceId[deviceIndex]));
              }

              // if (!confidence[deviceIndex].empty()) {
              //   confidenceWindowName[deviceIndex].str("");
              //   confidenceWindowName[deviceIndex].clear();
              //   confidenceWindowName[deviceIndex] << "Confidence Frame " << deviceIndex;
              //   namedWindow(confidenceWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);
              //   /*if (colorMapWindowOpen) {
              //     try {
              //       cv::destroyWindow(confidenceWindowName[deviceIndex].str().c_str());
              //       colorMapWindowOpen = false;
              //     } catch (const cv::Exception& e) {
              //       std::cerr << "OpenCV destroyWindow error: " << e.what() << std::endl;
              //     }
              //   }*/

              //  imshow(confidenceWindowName[deviceIndex].str().c_str(), confidence[deviceIndex]);
              //  if (!mouseCallbckSync[deviceIndex]) {
              //    callBckDeviceId[deviceIndex] = deviceIndex;
              //    mouseCallbckSync[deviceIndex] = true;
              //  }
              //  /*setMouseCallback(depthVistaColorMapWindowName[deviceIndex].str(), mouseCallBck,
              //                   static_cast<void*>(&callBckDeviceId[deviceIndex]));*/
              //}

              if (startSavingFrames[deviceIndex]) {
                sprintf(static_cast<char*>(depthFrameFileNameBuf), "%s/DepthVistaCam%d_Depth_%d_%d_%d_%d_%d_%d.bmp",
                        cwd.data(), deviceIndex, timeStruct->tm_year + 1900, timeStruct->tm_mon + 1,
                        timeStruct->tm_mday, timeStruct->tm_hour, timeStruct->tm_min, timeStruct->tm_sec);
                DepthFrameSaveStatus[deviceIndex] =
                    imwrite(static_cast<char*>(depthFrameFileNameBuf), Depthcolormap[deviceIndex]);
                if (DepthFrameSaveStatus[deviceIndex]) {
                  cout << "\n"
                       << "DepthColorMap frame is successfully saved as " << static_cast<char*>(depthFrameFileNameBuf)
                       << "\n";
                } else {
                  cout << "\n" << "Saving DepthColorMap Frame Failed" << "\n";
                }
                sprintf(static_cast<char*>(depthRawFrameFileNameBuf), "%s/DepthVistaCam%d_raw_%d_%d_%d_%d_%d_%d.raw",
                        cwd.data(), deviceIndex, timeStruct->tm_year + 1900, timeStruct->tm_mon + 1,
                        timeStruct->tm_mday, timeStruct->tm_hour, timeStruct->tm_min, timeStruct->tm_sec);

                const std::unique_ptr<FILE, decltype(&fclose)> rawFrameFilePtr(
                    fopen(static_cast<char*>(depthRawFrameFileNameBuf), "wb+"), fclose);
                if (rawFrameFilePtr) {
                  fwrite(DepthImg_Y16[deviceIndex].data,
                         DepthImg_Y16[deviceIndex].total() * DepthImg_Y16[deviceIndex].channels() *
                             DepthImg_Y16[deviceIndex].elemSize1(),
                         1, rawFrameFilePtr.get());

                  cout << "\n"
                       << "Raw Depth frame is successfully saved as " << static_cast<char*>(depthRawFrameFileNameBuf)
                       << "\n";
                } else {
                  cout << "\n" << "unable to open depth raw frame for writing" << "\n";
                }
                if (gDevicesList[camAccessId].devType == DeviceType::ITOF_USB ||
                    gDevicesList[camAccessId].devType == DeviceType::ITOF_GMSL) {
                  sprintf(static_cast<char*>(depthConfFrameFileNameBuf),
                          "%s/DepthVistaCam%d_confidence_%d_%d_%d_%d_%d_%d.raw", cwd.data(), deviceIndex,
                          timeStruct->tm_year + 1900, timeStruct->tm_mon + 1, timeStruct->tm_mday, timeStruct->tm_hour,
                          timeStruct->tm_min, timeStruct->tm_sec);

                  const std::unique_ptr<FILE, decltype(&fclose)> confFrameFilePtr(
                      fopen(static_cast<char*>(depthConfFrameFileNameBuf), "wb+"), fclose);
                  if (confFrameFilePtr) {
                    fwrite(confidence[deviceIndex].data,
                           confidence[deviceIndex].total() * confidence[deviceIndex].channels() *
                               confidence[deviceIndex].elemSize1(),
                           1, confFrameFilePtr.get());

                    cout << "\n"
                         << "confidence frame is successfully saved as "
                         << static_cast<char*>(depthConfFrameFileNameBuf) << "\n";
                  } else {
                    cout << "\n" << "unable to open depth conf frame for writing" << "\n";
                  }
                }

                sprintf(static_cast<char*>(PLY3DFileNameBuf), "%s/DepthVistaCam%d_3D_%d_%d_%d_%d_%d_%d.ply", cwd.data(),
                        deviceIndex, timeStruct->tm_year + 1900, timeStruct->tm_mon + 1, timeStruct->tm_mday,
                        timeStruct->tm_hour, timeStruct->tm_min, timeStruct->tm_sec);
                CalibrationParams devCalibrationData;
                const Result readCalibRes =
                    GetDeviceCalibrationParams(deviceHandleList[deviceIndex], &devCalibrationData);
                if (readCalibRes == Result::Ok) {
                  if (rgbdMappingflag[deviceIndex]) {
                    if (dataModeVec[deviceIndex] == DataMode::Depth_IR_RGB_HD_Mode ||
                        dataModeVec[deviceIndex] == DataMode::Depth_RGB_HD_Mode) {
                      std::thread saveThread([=]() {
                        savePLYfiles(devCalibrationData.rgbCamHDIntrinsic, DepthImg_Y16[deviceIndex],
                                     rgbFrame[deviceIndex], confidence[deviceIndex], PLY3DFileNameBuf);
                      });
                      saveThread.detach();
                    } else {
                      std::thread saveThread([=]() {
                        savePLYfiles(devCalibrationData.rgbCamVGAIntrinsic, DepthImg_Y16[deviceIndex],
                                     rgbFrame[deviceIndex], confidence[deviceIndex], PLY3DFileNameBuf);
                      });
                      saveThread.detach();
                    }
                  } else {
                    if (dataModeVec[deviceIndex] >= DataMode::Depth_IR_Conf_HD_Mode &&
                         dataModeVec[deviceIndex] <= DataMode::Depth_Conf_200Mhz) {
                      std::thread saveThread([=]() {
                        savePLYfiles(devCalibrationData.depthCamHDIntrinsic, DepthImg_Y16[deviceIndex],
                                     Depthcolormap[deviceIndex], confidence[deviceIndex], PLY3DFileNameBuf);
                      });
                      saveThread.detach();
                    } else {
                      std::thread saveThread([=]() {
                        savePLYfiles(devCalibrationData.depthCamVGAIntrinsic, DepthImg_Y16[deviceIndex],
                                     Depthcolormap[deviceIndex], confidence[deviceIndex], PLY3DFileNameBuf);
                      });
                      saveThread.detach();
                    }
                  cout << "\n" << "3D PLY is saved as " << PLY3DFileNameBuf << "\n";
                  }
                } else {
                  cout << "3D files are not saved because Calibration Data is Not Found in the device\n";
                }
              }
            }
            // To get IR frame
            // cout<<"24\n";
            if ((dataModeVec[deviceIndex] != Depth_Mode && dataModeVec[deviceIndex] <= Depth_IR_RGB_HD_Mode) ||
                dataModeVec[deviceIndex] == Depth_IR_Conf_HD_Mode) {
              if ((!IRFrame[deviceIndex].empty())) {
                // cout<<"25\n";
                depthVistaIRWindowName[deviceIndex].str("");
                depthVistaIRWindowName[deviceIndex].clear();
                // cout<<"26\n";
                depthVistaIRWindowName[deviceIndex] << "DepthVista IR Frame " << deviceIndex + 1;
                namedWindow(depthVistaIRWindowName[deviceIndex].str(), WINDOW_AUTOSIZE);
                // cout<<"27\n";
                imshow(depthVistaIRWindowName[deviceIndex].str(), IRFrame[deviceIndex]);

                if (startSavingFrames[deviceIndex]) {
                  sprintf(static_cast<char*>(IRFrameFileNameBuf), "%s/DepthVistaCam%d_IR_%d_%d_%d_%d_%d_%d.png",
                          cwd.data(), deviceIndex, timeStruct->tm_year + 1900, timeStruct->tm_mon + 1,
                          timeStruct->tm_mday, timeStruct->tm_hour, timeStruct->tm_min, timeStruct->tm_sec);
                  IRFrameSaveStatus[deviceIndex] =
                      imwrite(static_cast<char*>(IRFrameFileNameBuf), IRFrame[deviceIndex]);
                  if (IRFrameSaveStatus[deviceIndex]) {
                    cout << "\n"
                         << "IR frame is successfully saved as " << static_cast<char*>(IRFrameFileNameBuf) << "\n";
                  } else {
                    cout << "\n" << "Saving IR Frame Failed" << "\n";
                  }

                  sprintf(static_cast<char*>(IRRawFrameFileNameBuf), "%s/DepthVistaCam%d_IR_raw_%d_%d_%d_%d_%d_%d.raw",
                          cwd.data(), deviceIndex, timeStruct->tm_year + 1900, timeStruct->tm_mon + 1,
                          timeStruct->tm_mday, timeStruct->tm_hour, timeStruct->tm_min, timeStruct->tm_sec);

                  const std::unique_ptr<FILE, decltype(&fclose)> rawIRFrameFilePtr(
                      fopen(static_cast<char*>(IRRawFrameFileNameBuf), "wb+"), fclose);
                  if (rawIRFrameFilePtr) {
                    fwrite(IRRawImg_Y16[deviceIndex].data,
                           IRRawImg_Y16[deviceIndex].total() * IRRawImg_Y16[deviceIndex].channels() *
                               IRRawImg_Y16[deviceIndex].elemSize1(),
                           1, rawIRFrameFilePtr.get());

                    cout << "\n"
                         << "Raw IR frame is successfully saved as " << static_cast<char*>(IRRawFrameFileNameBuf)
                         << "\n";
                  } else {
                    cout << "\n" << "unable to open IR raw frame for writing" << "\n";
                  }
                }
              }
            }
            previewMutex[deviceIndex].unlock();

            if (startSavingFrames[deviceIndex]) {
              startSavingFrames[deviceIndex] = saveFrames[deviceIndex] = false;
            }
          }
        }
      }
      keyPressed = static_cast<char>(waitKey(5));
      while (bSwitch) {
        bSwitch = false;
        destroyAllWindows();
      }
    }
    //Depthstreamstarted = false;
  }

  return nullptr;
}
#endif

#ifdef __linux__
// Forward declarations for GLib log API.
// glib-2.0 is already linked (via linkopts), so no header is needed.
extern "C" {
  typedef unsigned int guint;
  typedef unsigned int GLogLevelFlags;
  typedef void (*GLogFunc)(const char*, GLogLevelFlags, const char*, void*);
  guint g_log_set_handler(const char* log_domain, GLogLevelFlags log_levels,
                           GLogFunc log_func, void* user_data);
}
static void suppress_gtk_log(const char*, GLogLevelFlags, const char*, void*) {}
#endif

int main() {
#ifdef __linux__
  // Suppress "Failed to load module canberra-gtk-module" warning.
  // OpenCV is built with WITH_GTK=ON, so GTK initialises when a window is
  // first created. GTK_MODULES env var AND the XSETTINGS channel (pushed by
  // the desktop session) can both request canberra-gtk-module. Installing
  // a silent GLib log handler for the "Gtk" domain drops the message before
  // it reaches stderr, regardless of where GTK obtained the module name.
  unsetenv("GTK_MODULES");
  // G_LOG_LEVEL_MESSAGE = 1 << 5
  g_log_set_handler("Gtk", static_cast<GLogLevelFlags>(1 << 5), suppress_gtk_log, nullptr);
#endif
  // Basic Introduction about the Application
  cout << "\n" << "e-con's Sample Application for DepthVista " << "\n";
  cout << "\n" << "Demonstrates the working of e-con's DepthVistaSDK" << "\n";
  uint8_t gMajorVersion = 0;
  uint8_t gMinorVersion1 = 0;
  uint16_t gMinorVersion2 = 0;
  if (GetSDKVersion(&gMajorVersion, &gMinorVersion1, &gMinorVersion2) <= 0) {
    cout << "\n" << "\t" << "SDK version Failed" << "\n";
  }
  cout << "\n"
       << "\t" << "DepthVista SDK-Version = " << static_cast<uint16_t>(gMajorVersion) << "."
       << static_cast<uint16_t>(gMinorVersion1) << "." << gMinorVersion2 << "\n\n";

  // Initialize DepthVista
  if (Initialize() < 0) {
    pError();
  }

  // Open a Camera Device
  if (!(listDevices())) {
    cout << "\n" << "List Devices Information Failed" << "\n";
    return 0;
  }

  if (!(exploreCam())) {
    return 0;
  }
  return 0;
}

/**
 * @brief 		Listing and Opening Camera device
 * @return		bool    return true on successful listing and opening device, else retuns fail.
 */
bool listDevices() {
  bool listDeviceState = false;
  commonCamId = -1;
  devices = 0;
  uint32_t deviceCount = 0;
  if (GetDeviceCount(&deviceCount) < 0) {
    pError("GetDeviceCount");
    return false;
  }
  devices = static_cast<int>(deviceCount);
  depthVistaFrames.resize(devices);
  rgbdMappingflag.clear();
  rgbdMappingflag.resize(devices);
  maxDeviceIndex = devices;
  gDevicesList.clear();
  if (devices < 0) {
    cout << "\n" << "No Camera Devices Connected to the port" << "\n";
    return false;
  } else {
    gDevicesList.resize(devices);
    if (GetDeviceListInfo(devices, gDevicesList.data()) < 0) {
      pError("GetDeviceListInfo");
    }
  }
  exploreOpt = exploreOptions::notInitialize;
  bSwitch = true;

  for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
    bPreviewSet(1, false, deviceInd);
    previewThreadCtrlFlag[deviceInd] = false;
    streamThreadCtrlFlag[deviceInd] = false;
    SetRGBDMapping(deviceHandleList[deviceInd], 0);
    if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
    }
    
  }
  destroyAllWindows();
  multiDev.clear();
  deviceHandleList.clear();
  successCam.clear();
  selecteddevindex.clear();
  while (!listDeviceState) {
     
    cout << "\n" << "Number of Camera Devices Connected to the Port : " << devices << "\n";
    int exploreOption = -1;

    if (exploreOpt == exploreOptions::notInitialize && devices > 1) {
      cout << "\n" << "How would you like to explore the devices" << "\n";

      cout << '\t' << "0  - Exit" << "\n";
      cout << '\t' << "1  - Single Device" << "\n";
      cout << '\t' << "2  - Multiple Devices" << "\n";

      while ((exploreOption < 0) || (exploreOption > 2)) {
        printf("\n Pick a relavent Option: \t");
        int ret = scanf("%d", &exploreOption);
        if (ret != 1) {
          printf("Invalid input.\n");
          int c;
          while ((c = getchar()) != '\n' && c != EOF);
          exploreOption = -1;
        }
      }
      if (exploreOption == 0) {
        commonCamId = 0;
      } else {
        exploreOpt = static_cast<int16_t>(exploreOption - 1);
      }
    }
    if (exploreOption != 0) {
      if (exploreOpt == exploreOptions::singleCam || devices == 1) {
        // This is to initialize exploreOpt when devices == 1
        exploreOpt = 0;
        cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
        cout << '\t' << "0 - Exit" << "\n";

        // List the Camera Names
        for (uint32_t eachDevice = 0; eachDevice < devices; eachDevice++) {
          std::string deviceName = static_cast<char*>(gDevicesList[eachDevice].deviceName);
          if (gDevicesList[eachDevice].devType == DeviceType::ITOF_GMSL) {
            deviceName = "STURDeCAM13_TOF";
          }
          cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
               << static_cast<char*>(gDevicesList[eachDevice].serialNo) << "  "
               << static_cast<char*>(gDevicesList[eachDevice].devicePath) << ")\n";
        }
        commonCamId = -1;
        while ((commonCamId < 0) || (commonCamId > static_cast<int>(devices))) {
          printf("\n Pick a Camera Device to Explore : \t");
          commonCamId = 1;
          int ret = scanf("%d", &commonCamId);

          if (ret != 1) {
            printf("Invalid input.\n");
            int c;
            while ((c = getchar()) != '\n' && c != EOF);
            commonCamId = -1;
          }
        }
      } 
      else if (exploreOpt == exploreOptions::multipleCam) {
        int noOfCam = -1;
        while ((noOfCam <= 0) || (noOfCam > static_cast<int>(devices))) {
          printf("\n Enter the number of cameras to stream not more than %d : \t", devices);
          int ret = scanf("%d", &noOfCam);
          if (ret != 1) {
            //printf("Invalid input.\n");
            int c;
            while ((c = getchar()) != '\n' && c != EOF);
            noOfCam = -1;
          }
        }
        if (noOfCam == 1) {
          exploreOpt = exploreOptions::singleCam;
        }
        cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";

        for (uint32_t eachDevice = 0; eachDevice < devices; eachDevice++) {
          std::string deviceName = static_cast<char*>(gDevicesList[eachDevice].deviceName);
          if (gDevicesList[eachDevice].devType == DeviceType::ITOF_GMSL) {
            deviceName = "STURDeCAM13_TOF";
          }
          cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
               << static_cast<char*>(gDevicesList[eachDevice].serialNo) << "  "
               << static_cast<char*>(gDevicesList[eachDevice].devicePath) << ")\n";
        }
        int camSelect = -1;
        for (int i = 1; i < noOfCam + 1; i++) {
          while ((camSelect < 0) || (camSelect > static_cast<int>(devices))) {
            printf("\n Enter the index of camera %d for multiple device streaming : \t", i);
            int ret = scanf("%d", &camSelect);
            if (ret != 1) {
              //printf("Invalid input.\n");
              int c;
              while ((c = getchar()) != '\n' && c != EOF);
              camSelect = -1;
            } else {
              selecteddevindex.push_back(camSelect);
            }
          }
          multiDev.emplace_back(camSelect - 1);
          camSelect = -1;
        }
        if (!multiDev.empty()) {
          commonCamId = multiDev[0] + 1;
        }
        if (!multiDev.empty()) {
          commonCamId = multiDev[0] + 1;
        }
      }
    }
    switch (commonCamId) {
      case EXIT:
        if (DeInitialize() > 0) {
          destroyAllWindows();
        }

        std::quick_exit(0);
        break;

      default:
        if (exploreOpt != exploreOptions::singleCam) {
#ifdef _WIN32
          for (int itr = 0; itr < devices; itr++) {
            bSwitch = true;
            bPreviewSet(1, false, itr);
            streamStarted = false;
            previewThreadCtrlFlag[itr] = false;
            streamThreadCtrlFlag[itr] = false;
            if (bDetach) {
              if (streamThread[itr].joinable()) {
                streamThread[itr].detach();
              }
              if (previewThread[itr].joinable()) {
                previewThread[itr].detach();
              }
            }
            PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:after detach");
          }
#elif __linux__
          for (int itr = 0; itr < devices; itr++) {
            bSwitch = true;
            bPreviewSet(1, false, itr);
            streamStarted = false;
            previewThreadCtrlFlag[itr] = false;
            streamThreadCtrlFlag[itr] = false;
            pthread_join(streamThread[itr], nullptr);
            pthread_join(previewThread[itr], nullptr);
          }
#endif
        }

        currentDeviceIndex = commonCamId - 1;
        DeviceHandle devHandle;

        if (exploreOpt == exploreOptions::singleCam) {
#ifdef _WIN32
          bSwitch = true;
          bPreviewSet(1, false, 0);
          streamStarted = false;
          previewThreadCtrlFlag[0] = false;
          streamThreadCtrlFlag[0] = false;
          if (bDetach) {
            if (streamThread[0].joinable()) {
              streamThread[0].detach();
            }
            if (previewThread[0].joinable()) {
              previewThread[0].detach();
            }
          }
#elif __linux__
          bSwitch = true;
          bPreviewSet(1, false, 0);
          streamStarted = false;
          previewThreadCtrlFlag[0] = false;
          streamThreadCtrlFlag[0] = false;
          pthread_join(streamThread[0], nullptr);
          pthread_join(previewThread[0], nullptr);
#endif
          const Result odRes = OpenDevice(gDevicesList[currentDeviceIndex], &devHandle);

          if (odRes < 0) {
            if (odRes == Result::CameraAlreadyOpen) {
              previewThreadCtrlFlag[0] = true;
              streamThreadCtrlFlag[0] = true;
              tempDeviceIndex[0] = 0;
#ifdef _WIN32
              previewThread[0] = thread(&preview, 0);
              streamThread[0] = thread(&stream, 0);
#elif __linux__
              pthread_create(&streamThread[0], nullptr, stream, &tempDeviceIndex[0]);
              pthread_create(&previewThread[0], nullptr, preview, &tempDeviceIndex[0]);
#endif
              startStream();
              listDeviceState = true;
              return true;
            } else {
              pError("OpenDevice:");
              listDeviceState = false;
              break;
            }
          } else {
            listDeviceState = true;
          }

          deviceHandleList.insert(deviceHandleList.begin(), devHandle);

          RegisterNotificationCallback(devHandle, &registerNotificationCb);
          previewThreadCtrlFlag[0] = true;
          streamThreadCtrlFlag[0] = true;

#ifdef _WIN32
          previewThread[0] = thread(&preview, 0);
          streamThread[0] = thread(&stream, 0);

#elif __linux__

          tempDeviceIndex[0] = 0;
          pthread_create(&streamThread[0], nullptr, stream, &tempDeviceIndex[0]);
          pthread_create(&previewThread[0], nullptr, preview, &tempDeviceIndex[0]);
#endif
          // startStream();
        }

        if (exploreOpt != exploreOptions::singleCam) {
          deviceHandleList.clear();
          dataModeVec.clear();
          depthRangeVec.clear();
          listDeviceState = true;
          std::vector<int> failedOpen;
          for (const int deviceNO : multiDev) {
            DeviceHandle devHandle;
            const Result OpenDeviceRes = OpenDevice(gDevicesList[deviceNO], &devHandle);
 
            if (OpenDeviceRes < 0) {
              pError("OpenDevice:");
              failedOpen.emplace_back(deviceNO);
            } else {
              deviceHandleList.push_back(devHandle);
              successCam.push_back(gDevicesList[deviceNO]);
              RegisterNotificationCallback(deviceHandleList[deviceNO], &registerNotificationCb);
            }
          }
          if (deviceHandleList.empty()) {
            listDeviceState = false;
            break;
          }
          currentDeviceIndex = 0;
#ifdef _WIN32
          for (int deviceNO = 0; deviceNO < deviceHandleList.size(); deviceNO++) {
            streamThreadCtrlFlag[deviceNO] = true;
            previewThreadCtrlFlag[deviceNO] = true;
            previewThread[deviceNO] = thread(&preview, deviceNO);
            streamThread[deviceNO] = thread(&stream, deviceNO);
          }
#elif __linux__
          for (int deviceNO = 0; deviceNO < maxDeviceIndex; deviceNO++) {
            streamThreadCtrlFlag[deviceNO] = true;
            previewThreadCtrlFlag[deviceNO] = true;
            tempDeviceIndex[deviceNO] = deviceNO;
            pthread_create(&streamThread[deviceNO], nullptr, stream, &tempDeviceIndex[deviceNO]);
            pthread_create(&previewThread[deviceNO], nullptr, preview, &tempDeviceIndex[deviceNO]);
          }
#endif
        }
        dataModeVec.clear();
        depthRangeVec.clear();
        for (int i = 0; i < deviceHandleList.size(); i++) {
          DeviceInfo devInfo;
          if (exploreOpt == exploreOptions::singleCam) {
            devInfo = gDevicesList[currentDeviceIndex];
          } else {
            devInfo = successCam[i];
          }

          if (devInfo.devType != DeviceType::ITOF_USB && devInfo.devType != DeviceType::ITOF_GMSL) {
            SetAvgIRDisplay(deviceHandleList[i], 1);
            SetPlanarization(deviceHandleList[i], 1);
          }
          DataMode tempDataMode = DataMode::Depth_IR_Conf_Mode;
          const Result resss = GetDataMode(deviceHandleList[i], &tempDataMode);

          if((tempDataMode != DataMode::IR_Mode&&tempDataMode <= DataMode::Depth_IR_RGB_HD_Mode) || tempDataMode == DataMode::Depth_IR_Conf_HD_Mode){
            Depthstreamstarted = true;
          }
          else{
            Depthstreamstarted = false;
          }

          // cout<<"getdatamode : "<<resss<<"  "<<tempDataMode <<endl;
          if (resss < 0) {
            pError("GetDataMode");
          }
          dataModeVec.emplace_back(tempDataMode);
          uint16_t tempDepthRange = 0;
          if (GetDepthRange(deviceHandleList[i], &tempDepthRange) < 0) {
            pError("GetDepthRange");
          }
          depthRangeVec.emplace_back(tempDepthRange);
          // if(gDevicesList[camId - 1].devType != DeviceType::ITOF_USB && gDevicesList[camId - 1].devType
          // != DeviceType::ITOF_GMSL) {
          SetAvgRegion(deviceHandleList[i], AvgRegion::CustomPtr);
          // }
        }
        startStream();
        break;
    }
  }
#ifdef _WIN32
  bDetach = true;
  saveEvent[0] = CreateEvent(NULL, FALSE, FALSE, L"frameCaptureEvent0");
  saveEvent[1] = CreateEvent(NULL, FALSE, FALSE, L"frameCaptureEvent1");
  m_hEvent[0] = CreateEvent(NULL, FALSE, FALSE, L"renderEvent0");
  m_hEvent[1] = CreateEvent(NULL, FALSE, FALSE, L"renderEvent1");
#endif
  return true;
}

/**
 * @brief 		Setting Streaming Mode of the Device
 * @return		bool    return true on successfully setting the stream mode of the device, else retuns fail.
 */
bool selectStreamingMode() {
  int option = -1;
  int maxVal = 0;
  int camId = -1;
  int camAccessId = -1;
  uint16_t tempDataMode = 0;
  bool startstreamFlag = false;

  if (exploreOpt == exploreOptions::multipleCam) {
    camId = 1;
    camAccessId = camId;
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    // cout<<"common cam id : "<<camId<<endl;
    camAccessId = 1;
  }
  switch (camId) {
    case 0:
      return true;
    default:
      bool allItof = true;
      if (exploreOpt == exploreOptions::multipleCam) {
        if (successCam.empty()) {
          allItof = false;
        }
        for (const auto& dev : successCam) {
          if (dev.devType != DeviceType::ITOF_USB && dev.devType != DeviceType::ITOF_GMSL) {
            allItof = false;
            break;
          }
        }
      } else {  // single cam
        if (gDevicesList[camId - 1].devType != DeviceType::ITOF_USB &&
            gDevicesList[camId - 1].devType != DeviceType::ITOF_GMSL) {
          allItof = false;
        }
      }

      if (allItof) {
        cout << "\n" << "Total Number of Streaming Modes Supported by the Camera:  " << '\t' << "2" << "\n";
        cout << '\t' << "0  - Exit" << "\n";
        cout << '\t' << "1  - Back" << "\n";
        cout << '\t' << "2  - Main Menu" << "\n";
        cout << '\t' << "3  - VGA_LongRange_6m" << "\n";
        cout << '\t' << "4  - 1.2MP_ShortRange_2m" << "\n";
        maxVal = 5;
      } else {
        bool allCmos = true;
        if (exploreOpt == exploreOptions::multipleCam) {
          if (successCam.empty()) {
            allCmos = false;
          }
          for (const auto& dev : successCam) {
            if (dev.devType != DeviceType::CMOS_MIPI && dev.devType != DeviceType::CMOS_USB_IRD) {
              allCmos = false;
              break;
            }
          }
        } else {  // single cam
          if (gDevicesList[camId - 1].devType != DeviceType::CMOS_MIPI &&
              gDevicesList[camId - 1].devType != DeviceType::CMOS_USB_IRD) {
            allCmos = false;
          }
        }

        if (allCmos) {
          cout << "\n" << "Total Number of Streaming Modes Supported by the Camera:  " << '\t' << "2" << "\n";
          cout << '\t' << "0  - Exit" << "\n";
          cout << '\t' << "1  - Back" << "\n";
          cout << '\t' << "2  - Main Menu" << "\n";
          cout << '\t' << "3  - Depth IR Mode" << "\n";
          cout << '\t' << "4  - Depth Mode" << "\n";
          maxVal = 5;
        } else {
          cout << "\n" << "Total Number of Streaming Modes Supported by the Camera:  " << '\t' << "9" << "\n";
          cout << '\t' << "0  - Exit" << "\n";
          cout << '\t' << "1  - Back" << "\n";
          cout << '\t' << "2  - Main Menu" << "\n";
          cout << '\t' << "3  - Depth IR Mode" << "\n";
          cout << '\t' << "4  - Depth Mode" << "\n";
          cout << '\t' << "5  - IR Mode" << "\n";
          cout << '\t' << "6  - Depth IR RGB VGA Mode" << "\n";
          cout << '\t' << "7  - Depth IR RGB HD Mode" << "\n";
          cout << '\t' << "8  - RGB VGA Mode" << "\n";
          cout << '\t' << "9  - RGB HD Mode" << "\n";
          cout << '\t' << "10 - RGB Full HD Mode" << "\n";
          cout << '\t' << "11 - RGB 1920x1200 Mode" << "\n";
          maxVal = 12;
        }
      }
      break;
  }
  char input[100];
  int tempOption;

  // Clear leftover newline from previous scanf
  // int c;
  // while ((c = getchar()) != '\n' && c != EOF);

  while (1) {
      printf("\n Pick a Relevant Streaming Mode:\t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //     continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      // Remove newline
      input[strcspn(input, "\n")] = '\0';

      // Empty input check
      if (strlen(input) == 0) {
          printf("\n Invalid input!\n");
          continue;
      }

      char extra;

      // Strict integer validation
      if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
          printf("\n Invalid input! Numbers only.\n");
          continue;
      }

      // Range validation
      if (tempOption < 0 || tempOption >= maxVal) {
          printf("\n Please enter a valid option between 0 and %d.\n",
                maxVal - 1);
          continue;
      }

      option = tempOption;
      break;
  }

  switch (option) {
    case EXIT:
      bSwitch = true;

      for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
        bPreviewSet(1, false, deviceInd);
        previewThreadCtrlFlag[deviceInd] = false;
        streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
        if (bDetach) {
          if (streamThread[deviceInd].joinable()) {
            streamThread[deviceInd].detach();
          }
          if (previewThread[deviceInd].joinable()) {
            previewThread[deviceInd].detach();
          }
        }
#endif
        if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
          cout << "Device Close success\n";
        }
      }
      if (DeInitialize() > 0) {
        destroyAllWindows();
      }

      std::quick_exit(0);

    case 1:
    case 2:
      break;

    case 3: {
      bool allItof = true;
      if (exploreOpt == exploreOptions::multipleCam) {
        if (successCam.empty()) {
          allItof = false;
        }
        for (const auto& dev : successCam) {
          if (dev.devType != DeviceType::ITOF_USB && dev.devType != DeviceType::ITOF_GMSL) {
            allItof = false;
            break;
          }
        }
      } else {  // single cam
        if (gDevicesList[camId - 1].devType != DeviceType::ITOF_USB &&
            gDevicesList[camId - 1].devType != DeviceType::ITOF_GMSL) {
          allItof = false;
        }
      }
      if (allItof) {
        tempDataMode = option - 2;
      } else {
        tempDataMode = option - 3;
      }
    }
      bSwitch = true;
      if (exploreOpt == exploreOptions::multipleCam) {
        for (int itr = 0; itr < static_cast<int>(deviceHandleList.size()); itr++) {
          if (dataModeVec[itr] != tempDataMode) {
            startstreamFlag = true;
            if (SetAvgRegion(deviceHandleList[itr], AvgRegion::Center) < 0) {
              cout << "SetAvgRegion failed\n" << "\n";
            }
            bPreviewSet(1, false, itr);
            streamStarted = false;
#ifdef __linux__
            destroyAllWindows();
#endif
            if (SetDataMode(deviceHandleList[itr], static_cast<DataMode>(tempDataMode)) < 0) {
              pError("SetDataMode");
              continue;
            }
            dataModeVec[itr] = tempDataMode;
            if (SetAvgRegion(deviceHandleList[itr], AvgRegion::CustomPtr) < 0) {
              cout << "SetAvgRegion failed\n" << "\n";
            }
            DataMode cur_mode;
            GetDataMode(deviceHandleList[camAccessId - 1],&cur_mode);
            if((cur_mode != DataMode::IR_Mode&&cur_mode <= DataMode::Depth_IR_RGB_HD_Mode) || cur_mode == DataMode::Depth_IR_Conf_HD_Mode){
              Depthstreamstarted = true;
            }
            else{
              Depthstreamstarted = false;
            }
          }
        }
        if (startstreamFlag) {
          startStream();
        }
      } else if (exploreOpt == exploreOptions::singleCam) {
        if (dataModeVec[camAccessId - 1] != tempDataMode) {
          // if(gDevicesList[camId - 1].devType != DeviceType::ITOF_USB && gDevicesList[camId - 1].devType
          // != DeviceType::ITOF_GMSL) {
          if (SetAvgRegion(deviceHandleList[camAccessId - 1], AvgRegion::Center) < 0) {
            cout << "SetAvgRegion failed\n" << "\n";
          }
          // }
          bPreviewSet(1, false, camAccessId - 1);
          streamStarted = false;
#ifdef __linux__
          destroyAllWindows();
#endif
          if (SetDataMode(deviceHandleList[camAccessId - 1], static_cast<DataMode>(tempDataMode)) < 0) {
            pError("SetDataMode");
            return false;
          }
          dataModeVec[camAccessId - 1] = tempDataMode;
          // if(gDevicesList[camId - 1].devType != DeviceType::ITOF_USB && gDevicesList[camId - 1].devType
          // != DeviceType::ITOF_GMSL) {
          if (SetAvgRegion(deviceHandleList[camAccessId - 1], AvgRegion::CustomPtr) < 0) {
            cout << "SetAvgRegion failed\n" << "\n";
          }
          DataMode cur_mode;
          GetDataMode(deviceHandleList[camAccessId - 1],&cur_mode);
          if((cur_mode != DataMode::IR_Mode&&cur_mode <= DataMode::Depth_IR_RGB_HD_Mode) || cur_mode == DataMode::Depth_IR_Conf_HD_Mode){
            Depthstreamstarted = true;
          }
          else{
            Depthstreamstarted = false;
          }
          startStream();
        }
      }

      break;

    default: {
      bool allItof = true;
      if (exploreOpt == exploreOptions::multipleCam) {
        if (successCam.empty()) {
          allItof = false;
        }
        for (const auto& dev : successCam) {
          if (dev.devType != DeviceType::ITOF_USB && dev.devType != DeviceType::ITOF_GMSL) {
            allItof = false;
            break;
          }
        }
      } else {  // single cam
        if (gDevicesList[camId - 1].devType != DeviceType::ITOF_USB &&
            gDevicesList[camId - 1].devType != DeviceType::ITOF_GMSL) {
          allItof = false;
        }
      }
      if (allItof) {
        if (option == 4) {
          tempDataMode = 20;
        }
      } else {
        tempDataMode = option - 2;
      }
    }
      bSwitch = true;
      if (exploreOpt == exploreOptions::multipleCam) {
        for (int itr = 0; itr < static_cast<int>(deviceHandleList.size()); itr++) {
          if (dataModeVec[itr] != tempDataMode) {
            startstreamFlag = true;
            if (SetAvgRegion(deviceHandleList[itr], AvgRegion::Center) < 0) {
              cout << "SetAvgRegion failed\n" << "\n";
            }
            bPreviewSet(1, false, itr);
            streamStarted = false;
#ifdef __linux__
            destroyAllWindows();
#endif
            if (SetDataMode(deviceHandleList[itr], static_cast<DataMode>(tempDataMode)) < 0) {
              pError("SetDataMode");
              continue;
            }
            dataModeVec[itr] = tempDataMode;
            if (SetAvgRegion(deviceHandleList[itr], AvgRegion::CustomPtr) < 0) {
              cout << "SetAvgRegion failed\n" << "\n";
            }
            DataMode cur_mode;
            GetDataMode(deviceHandleList[camAccessId - 1],&cur_mode);
            if((cur_mode != DataMode::IR_Mode&&cur_mode <= DataMode::Depth_IR_RGB_HD_Mode) || cur_mode == DataMode::Depth_IR_Conf_HD_Mode){
              Depthstreamstarted = true;
            }
            else{
              Depthstreamstarted = false;
            }
          }
        }
        if (startstreamFlag) {
          startStream();
        }
      } else if (exploreOpt == exploreOptions::singleCam) {
        if (dataModeVec[camAccessId - 1] != tempDataMode) {
          // if(gDevicesList[camId - 1].devType != DeviceType::ITOF_USB && gDevicesList[camId - 1].devType
          // != DeviceType::ITOF_GMSL) {
          if (SetAvgRegion(deviceHandleList[camAccessId - 1], AvgRegion::Center) < 0) {
            cout << "SetAvgRegion failed\n" << "\n";
          }
          // }
          bPreviewSet(1, false, camAccessId - 1);
          streamStarted = false;
#ifdef __linux__
          destroyAllWindows();
#endif
          if (SetDataMode(deviceHandleList[camAccessId - 1], static_cast<DataMode>(tempDataMode)) < 0) {
            pError("SetDataMode");
            return false;
          }
          dataModeVec[camAccessId - 1] = tempDataMode;
          // if(gDevicesList[camId - 1].devType != DeviceType::ITOF_USB && gDevicesList[camId - 1].devType
          // != DeviceType::ITOF_GMSL) {
          if (SetAvgRegion(deviceHandleList[camAccessId - 1], AvgRegion::CustomPtr) < 0) {
            cout << "SetAvgRegion failed\n" << "\n";
          }
          DataMode cur_mode;
          GetDataMode(deviceHandleList[camAccessId - 1],&cur_mode);
          if((cur_mode != DataMode::IR_Mode&&cur_mode <= DataMode::Depth_IR_RGB_HD_Mode) || cur_mode == DataMode::Depth_IR_Conf_HD_Mode){
            Depthstreamstarted = true;
          }
          else{
            Depthstreamstarted = false;
          }
          startStream();
        }
      }

      break;
  }
  return true;
}

/**
 * @brief 		Setting depth value Mode of the Device
 * @return		bool    return true on successfully setting the stream mode of the device, else retuns fail.
 */
bool selectGetDepthValue() {
  int avgDepth = 0;
  int stdDepth = 0;
  int avgIR = 0;
  int stdIR = 0;
  DepthPtr cor;
  int avgX = -1;
  int avgY = -1;

  cout << '\t' << "0 - Exit" << "\n";
  cout << '\t' << "1 - Back" << "\n";
  cout << '\t' << "2 - Main Menu" << "\n";
  cout << '\t' << "3 - Centre" << "\n";
  cout << '\t' << "4 - Custom co-ordinate" << "\n";

  int option = -1;

char input[100];
int tempOption;

// Consume leftover newline from previous scanf
// {
//     int ch;
//     while ((ch = getchar()) != '\n' && ch != EOF);
// }

while (1) {

    printf("\n Pick a Relevant Depth value position:\t");

    // if (fgets(input, sizeof(input), stdin) == NULL) {
    //     continue;
    // }

    // --- ADDED FIX: Smart flush to clean the buffer safely ---
    while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
        std::cin.get();
    }
    // ---------------------------------------------------------

    if (fgets(input, sizeof(input), stdin) == NULL) {
      continue;
    }

    if (strcmp(input, "\n") == 0) {
        continue;
    }

    // Remove newline
    input[strcspn(input, "\n")] = '\0';

    char extra;

    // Strict integer validation
    if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
    }

    // Range validation
    if (tempOption < 0 || tempOption >= 5) {
        printf("\n Please enter a valid option between 0 and 4.\n");
        continue;
    }

    option = tempOption;
    break;
}

  switch (option) {
    case EXIT:
      bSwitch = true;

      for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
        bPreviewSet(1, false, deviceInd);
        previewThreadCtrlFlag[deviceInd] = false;
        streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
        if (bDetach) {
          if (streamThread[deviceInd].joinable()) {
            streamThread[deviceInd].detach();
          }
          if (previewThread[deviceInd].joinable()) {
            previewThread[deviceInd].detach();
          }
        }
#endif
        if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
          //cout << "Device Close success\n";
        }
      }
      if (DeInitialize() > 0) {
        destroyAllWindows();
      }

      std::quick_exit(0);
    case 1:
    case 2:
      return true;

    case 3:
      for (int deviceCnt = 0; deviceCnt < static_cast<int>(deviceHandleList.size()); deviceCnt++) {
        std::string input = "0";
        char leftover = 0;
        while (true) {
          
          printf("\n Enter width within range(4 to 200) of average kernal for camera %d :\t", deviceCnt);
          std::getline(std::cin, input);
          std::stringstream stringStream(input);
          if (stringStream >> avgX && !(stringStream >> leftover) && avgX >= MIN_AVG_X && avgX <= MAX_AVG_X) {
            // std::cout << "Value of avgX : " << avgX << '\n';
            break;  // valid integer and in range
          } else {
            std::cout << "Invalid input! Please enter a valid integer within range.\n";
          }
        }
        while (true) {
          //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          printf("\n Enter height within range(4 to 200) of average kernal for camera %d :\t", deviceCnt);
          std::getline(std::cin, input);
          std::stringstream stringStream(input);
          if (stringStream >> avgY && !(stringStream >> leftover) && avgY >= MIN_AVG_X && avgY <= MAX_AVG_X) {
            // std::cout << "Value of avgY : " << avgY << '\n';
            break;  // valid integer and in range
          } else {
            std::cout << "Invalid input! Please enter a valid integer within range.\n";
          }
        }
        if (UpdateAvgXandY(deviceHandleList[deviceCnt], avgX, avgY) < 0) {
          cout << "\nUpdateAvgXandY failed\n" << "\n";
        }
        avgX = -1;
        avgY = -1;
        if (SetAvgRegion(deviceHandleList[deviceCnt], AvgRegion::Center) < 0) {
          cout << "SetAvgRegion failed\n" << "\n";
        }
        if (GetDepthIRValues(deviceHandleList[deviceCnt], &avgDepth, &stdDepth, &avgIR, &stdIR) < 1) {
          std::cout << "Failed to Get Depth values for device " << deviceCnt << "\n";
        } else {
          cout << "Average Depth Value of camera  " << deviceCnt << " : " << avgDepth << "\n";
          cout << "Standard Depth Value of camera " << deviceCnt << " : " << stdDepth << "\n";
          cout << "Average IR Value of camera     " << deviceCnt << " : " << avgIR << "\n";
          cout << "Standard IR Value of camera    " << deviceCnt << " : " << stdIR << "\n";
        }

        if (exploreOpt == exploreOptions::singleCam) {
          break;
        }
      }
      break;

    case 4:
    {
        char input[100];
        int tempValue;

        for (int deviceCnt = 0;
            deviceCnt < static_cast<int>(deviceHandleList.size());
            deviceCnt++) {

            // ---------------- avgX ----------------
            while (1) {

                printf("\n Enter width range (4 to 200) of average kernal for camera %d :\t",
                      deviceCnt);

                if (fgets(input, sizeof(input), stdin) == NULL) {
                    continue;
                }

                input[strcspn(input, "\n")] = '\0';

                if (strlen(input) == 0) {
                    continue;
                }

                char extra;

                if (sscanf(input, "%d %c", &tempValue, &extra) != 1) {
                    printf("\n Invalid input! Numbers only.\n");
                    continue;
                }

                if (tempValue < MIN_AVG_X || tempValue > MAX_AVG_X) {
                    printf("\n Please enter a value between %d and %d.\n",
                          MIN_AVG_X,
                          MAX_AVG_X);
                    continue;
                }

                avgX = tempValue;
                break;
            }

            // ---------------- avgY ----------------
            while (1) {

                printf("\n Enter height range (4 to 200) of average kernal for camera %d :\t",
                      deviceCnt);

                if (fgets(input, sizeof(input), stdin) == NULL) {
                    continue;
                }

                input[strcspn(input, "\n")] = '\0';

                if (strlen(input) == 0) {
                    continue;
                }

                char extra;

                if (sscanf(input, "%d %c", &tempValue, &extra) != 1) {
                    printf("\n Invalid input! Numbers only.\n");
                    continue;
                }

                if (tempValue < MIN_AVG_Y || tempValue > MAX_AVG_Y) {
                    printf("\n Please enter a value between %d and %d.\n",
                          MIN_AVG_Y,
                          MAX_AVG_Y);
                    continue;
                }

                avgY = tempValue;
                break;
            }

            if (UpdateAvgXandY(deviceHandleList[deviceCnt],
                              avgX,
                              avgY) < 0) {

                cout << "UpdateAvgXandY failed\n"
                    << "\n";
            }

            // ---------------- X Coordinate ----------------
            while (1) {

                printf("\n Enter X co-ordinate between the range %d to %d for camera %d: \t",
                      (avgX / 2) + 1,
                      Depthcolormap[deviceCnt].cols - (avgX / 2) - 1,
                      deviceCnt);

                if (fgets(input, sizeof(input), stdin) == NULL) {
                    continue;
                }

                input[strcspn(input, "\n")] = '\0';

                if (strlen(input) == 0) {
                    continue;
                }

                char extra;

                if (sscanf(input, "%d %c", &tempValue, &extra) != 1) {
                    printf("\n Invalid input! Numbers only.\n");
                    continue;
                }

                if (tempValue <= avgX / 2 ||
                    tempValue > Depthcolormap[deviceCnt].cols - (avgX / 2)) {

                    printf("\n Please enter a valid X coordinate.\n");
                    continue;
                }

                cor.X = tempValue;
                break;
            }

            // ---------------- Y Coordinate ----------------
            while (1) {

                printf("\n Enter Y co-ordinate between the range %d to %d for camera %d: \t",
                      (avgY / 2) + 1,
                      Depthcolormap[deviceCnt].rows - (avgY / 2) - 1,
                      deviceCnt);

                if (fgets(input, sizeof(input), stdin) == NULL) {
                    continue;
                }

                input[strcspn(input, "\n")] = '\0';

                if (strlen(input) == 0) {
                    continue;
                }

                char extra;

                if (sscanf(input, "%d %c", &tempValue, &extra) != 1) {
                    printf("\n Invalid input! Numbers only.\n");
                    continue;
                }

                if (tempValue <= avgY / 2 ||
                    tempValue > Depthcolormap[deviceCnt].rows - (avgY / 2)) {

                    printf("\n Please enter a valid Y coordinate.\n");
                    continue;
                }

                cor.Y = tempValue;
                break;
            }

            if (SetAvgRegion(deviceHandleList[deviceCnt],
                            AvgRegion::CustomPtr) < 0) {

                cout << "SetAvgRegion failed\n"
                    << "\n";
            }

            if (SetDepthPos(deviceHandleList[deviceCnt], cor) < 1) {

                cout << "\nFailed to set depth position"
                    << "\n";

            } else {

                if (GetDepthIRValues(deviceHandleList[deviceCnt],
                                    &avgDepth,
                                    &stdDepth,
                                    &avgIR,
                                    &stdIR) < 1) {

                    std::cout << "Failed to Get Depth values for device "
                              << deviceCnt
                              << "\n";

                } else {
                  cout << "Average Depth Value of camera  " << deviceCnt << " : " << avgDepth << "\n";
                  cout << "Standard Depth Value of camera " << deviceCnt << " : " << stdDepth << "\n";
                  cout << "Average IR Value of camera     " << deviceCnt << " : " << avgIR << "\n";
                  cout << "Standard IR Value of camera    " << deviceCnt << " : " << stdIR << "\n";
                }
            }

            avgX = -1;
            avgY = -1;
        }

        break;
    }

    default:
      break;
  }
  return true;
}

/**
 * @brief 		Setting UVC Controls to the Device
 * @return		bool    return true on successfully setting the UVC Controls of the device, else retuns fail.
 */
bool uvcControlMenu() {
  int maxVal = 0;
  int camId = -1;
  int option = -1;
  int value = 0;
  // const uint32_t expoComp = 0;
  UVCProp gProp;
  // const uint16_t xCor = 0;
  // const uint16_t yCor = 0;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    while (true) {
      printf("\n Pick a Camera Device to change UVC controls : \t");
      const int ret = scanf("%d", &camId);
      if (ret != 1) {
        //printf("Invalid input.\n");
        int c;
        while ((c = getchar()) != '\n' && c != EOF)
          ;
        continue;
      }
      if (camId >= 0 && camId <= static_cast<int>(devices)) {
        break;
      }
      printf("Value out of range. Please try again.\n");
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = 1;
  }

  switch (camId) {
    case 0:
      return true;

    default:
      cout << '\t' << "0  - Exit" << '\n';
      cout << '\t' << "1  - Back" << '\n';
      cout << '\t' << "2  - Main Menu" << '\n';
      cout << '\t' << "3  - BRIGHTNESS" << '\n';
      cout << '\t' << "4  - CONTRAST" << '\n';
      cout << '\t' << "5  - SATURATION" << '\n';
      cout << '\t' << "6  - WHITE BALANCE MODE" << '\n';
      cout << '\t' << "7  - WHITE BALANCE CONTROL" << '\n';
      cout << '\t' << "8  - GAMMA" << '\n';
      cout << '\t' << "9  - GAIN" << '\n';
      cout << '\t' << "10 - POWER LINE FREQ" << '\n';
      cout << '\t' << "11 - SHARPNESS" << '\n';
      cout << '\t' << "12 - EXPOSURE MODE" << '\n';
      cout << '\t' << "13 - EXPOSURE CONTROL" << '\n';
      maxVal = 14;

      while (true) {
        printf("\n Pick a Relevant UVC Property: \t");
        const int ret = scanf("%d", &option);
        if (ret != 1) {
          //printf("Invalid input.\n");
          int c;
          while ((c = getchar()) != '\n' && c != EOF)
            ;
          continue;
        }
        if (option >= 0 && option < maxVal) {
          break;
        }
        printf("Value out of range. Please try again.\n");
      }

      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      DataMode mode = {};
      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          break;
        case 3:
          cout << "Brightness control for the device " << camId << "\n";
          if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_BRIGHTNESS, &gProp) > 0)) {
            cout << "Query UVC property failed" << "\n";
            return false;
          }
          while (true) {
            cout << "Current Brightness value is : " << gProp.cur << "\n";
            cout << "Enter relavent Brightness value between " << gProp.min << " to " << gProp.max << " : " << "\n";
            const int ret = scanf("%d", &value);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
            if ((value > gProp.min) || (value < gProp.max)) {
              break;
            }
          }
          if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_BRIGHTNESS, value) < 0) {
            cout << "\nFailed to set BRIGHTNESS value" << "\n";
            return false;
          }
          break;
        case 4:
          cout << "Contrast control for the device " << camId << "\n";
          if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_CONTRAST, &gProp) > 0)) {
            cout << "Query UVC property failed" << "\n";
            return false;
          }
          while (true) {
            cout << "Current CONTRAST value is : " << gProp.cur << "\n";
            cout << "Enter relavent CONTRAST value between " << gProp.min << " to " << gProp.max << " : " << "\n";
            const int ret = scanf("%d", &value);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
            if ((value > gProp.min) || (value < gProp.max)) {
              break;
            }
          }
          if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_CONTRAST, value) < 0) {
            cout << "\nFailed to set CONTRAST value" << "\n";
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
          break;
        case 5:
          cout << "Saturation control for the device " << camId << "\n";
          if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_SATURATION, &gProp) > 0)) {
            cout << "Query UVC property failed" << "\n";
            return false;
          }
          while (true) {
            cout << "Current SATURATION value is : " << gProp.cur << "\n";
            cout << "Enter relavent SATURATION value between " << gProp.min << " to " << gProp.max << " : " << "\n";
            const int ret = scanf("%d", &value);
            // Clear input buffer in case of invalid input
            while (getchar() != '\n' && getchar() != EOF) {
            }
            if (ret == 1 && value >= 0 && value <= gProp.max) {
              break;  // valid input
            }
            cout << "Invalid input! Please enter an integer between 0 and " << gProp.max << ".\n";
          }
          if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_SATURATION, value) < 0) {
            cout << "\nFailed to set SATURATION value" << "\n";
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
          break;
        case 6:
          cout << "White Balance control for the device " << camId << "\n";
          cout << '\t' << "1 - WHITE BALANCE AUTO OFF" << "\n";
          cout << '\t' << "2 - WHITE BALANCE AUTO ON" << "\n";
          value = 0;
          while ((value < 1) || (value >= 3)) {
            printf("\n Pick a Relevant option: \t");
            const int ret = scanf("%d", &value);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
          }
          value = value - 1;
          if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_WB_AUTO, value) < 0) {
            cout << "\nFailed to set WHITE BALANCE AUTO value" << "\n";
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
          break;
        case 7:
          if (GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_WB_AUTO, &gProp) < 0) {
            cout << "Query UVC property failed" << '\n';
            return false;
          } else {
            if (gProp.cur == 1) {
              cout << "White balance mode set to AUTO. Set White balance mode to MANUAL to change White Balance control"
                   << '\n';
              break;
            } else {
              cout << "White Balance control for the device " << camId << '\n';
              if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_WB_TEMP, &gProp) > 0)) {
                cout << "Query UVC property failed" << '\n';
                return false;
              }
              while (true) {
                cout << "Current WHITE BALANCE value is : " << gProp.cur << '\n';
                cout << "Enter relavent WHITE BALANCE value between " << gProp.min << " to " << gProp.max << " : "
                     << '\n';
                const int ret = scanf("%d", &value);
                while (getchar() != '\n' && getchar() != EOF && ret != 1) {
                }
                if ((value > gProp.min) || (value < gProp.max)) {
                  break;
                }
              }
              if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_WB_TEMP, value) < 0) {
                cout << "\nFailed to set WHITE BALANCE value" << '\n';
                return false;
              }
              if (exploreOpt == exploreOptions::singleCam) {
                break;
              }
            }
          }
          break;
        case 8:
          cout << "Gamma control for the device " << camId << "\n";
          if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_GAMMA, &gProp) > 0)) {
            cout << "Query UVC property failed" << "\n";
            return false;
          }
          while (true) {
            cout << "Current GAMMA value is : " << gProp.cur << "\n";
            cout << "Enter relavent GAMMA value between " << gProp.min << " to " << gProp.max << " : " << "\n";
            const int ret = scanf("%d", &value);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
            if ((value > gProp.min) || (value < gProp.max)) {
              break;
            }
          }
          if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_GAMMA, value) < 0) {
            cout << "\nFailed to set GAMMA value" << "\n";
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
          break;
        case 9:
          cout << "Gain control for the device " << camId << "\n";
          if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_GAIN, &gProp) > 0)) {
            cout << "Query UVC property failed" << "\n";
            return false;
          }
          while (true) {
            cout << "Current GAIN value is : " << gProp.cur << "\n";
            cout << "Enter relavent GAIN value between " << gProp.min << " to " << gProp.max << " : " << "\n";
            const int ret = scanf("%d", &value);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
            if ((value > gProp.min) || (value < gProp.max)) {
              break;
            }
          }
          if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_GAIN, value) < 0) {
            cout << "\nFailed to set GAIN value" << "\n";
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
          break;
        case 10:
          cout << "Power line frequency control for the device " << camId << "\n";
          if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_PWR_LINE_FREQ, &gProp) > 0)) {
            cout << "Query UVC property failed" << "\n";
            return false;
          }
          while (true) {
            cout << "Current POWER LINE FREQ value is : " << gProp.cur << "\n";
            cout << "Enter relavent POWER LINE FREQ value between " << gProp.min << " to " << gProp.max << " : "
                 << "\n";
            const int ret = scanf("%d", &value);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
            if ((value > gProp.min) || (value < gProp.max)) {
              break;
            }
          }
          if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_PWR_LINE_FREQ, value) < 0) {
            cout << "\nFailed to set POWER LINE FREQ value" << "\n";
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
          break;
        case 11:
          cout << "Sharpness control for the device " << camId << "\n";
          if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_SHARPNESS, &gProp) > 0)) {
            cout << "Query UVC property failed" << "\n";
            return false;
          }
          while (true) {
            cout << "Current SHARPNESS value is : " << gProp.cur << "\n";
            cout << "Enter relavent SHARPNESS value between " << gProp.min << " to " << gProp.max << " : " << "\n";
            const int ret = scanf("%d", &value);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
            if ((value > gProp.min) || (value < gProp.max)) {
              break;
            }
          }
          if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_SHARPNESS, value) < 0) {
            cout << "\nFailed to set SHARPNESS value" << "\n";
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
          break;
        case 12:

          GetDataMode(deviceHandleList[camId - 1], &mode);

          if((mode != DataMode::IR_Mode&&mode <= DataMode::Depth_IR_RGB_HD_Mode) || mode == DataMode::Depth_IR_Conf_HD_Mode){
            Depthstreamstarted = true;
          }
          else{
            Depthstreamstarted = false;
          }

          if (mode > DataMode::Depth_IR_RGB_HD_Mode) {
            cout << "Exposure mode control for the device " << camId << '\n';
            cout << '\t' << "1 - EXPOSURE AUTO ON" << '\n';
            cout << '\t' << "2 - EXPOSURE AUTO OFF" << '\n';
            value = 0;
            while ((value < 1) || (value >= 3)) {
              printf("\n Pick a Relevant option: \t");
              scanf("%d", &value);
              while (getchar() != '\n' && getchar() != EOF) {
              }
            }
            value = value - 1;
            if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_EXPOSURE_AUTO, value) < 0) {
              cout << "\nFailed to set EXPOSURE AUTO value" << '\n';
              return false;
            }
            if (exploreOpt == exploreOptions::singleCam) {
              break;
            }
          } else {
            cout << "Exposure mode cannot be changed in current DataMode. Control supported only in RGB modes" << '\n';
          }
          break;
        case 13:
          if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_EXPOSURE_AUTO, &gProp) > 0)) {
            cout << "Query UVC property failed" << '\n';
            return false;
          } else {
            if (gProp.cur == 0) {
              cout << "Exposure mode set to AUTO. Set Exposure mode to MANUAL to change Exposure" << '\n';
              break;
            } else {
              cout << "Exposure control for the device " << camId << '\n';
              if (!(GetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_EXPSOURE_ABS, &gProp) > 0)) {
                cout << "Query UVC property failed" << '\n';
                return false;
              }
              while (true) {
                cout << "Current EXPOSURE value is : " << gProp.cur << '\n';
                cout << "Enter relavent EXPOSURE value between " << gProp.min << " to " << gProp.max << " : " << '\n';
                scanf("%d", &value);
                while (getchar() != '\n' && getchar() != EOF) {
                }
                if ((value > gProp.min) || (value < gProp.max)) {
                  break;
                }
              }
              if (SetUVCControl(deviceHandleList[camId - 1], TOF_UVC_CID_EXPSOURE_ABS, value) < 0) {
                cout << "\nFailed to set EXPOSURE value" << '\n';
                return false;
              }
              if (exploreOpt == exploreOptions::singleCam) {
                break;
              }
            }
          }
          break;
        default:
          printf("Invalid option. Please try again.\n");
          break;
      }
      break;
  }
  return true;
}

/**
 * @brief 		Setting Depth Range of the Device
 * @return		bool    return true on successfully setting the depthRange of the device, else retuns fail.
 */
int depthRangeMenu() {
  int option = -1;
  int maxVal = 0;
  int camId = -1;
  int camAccessId = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    camId = 1;
    camAccessId = camId;
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    camAccessId = 1;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n" << "Total Number of Depth Range Supported by the Camera:  " << '\t' << "2" << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Near Mode" << "\n";
      cout << '\t' << "4 - Far Mode" << "\n";
      maxVal = 5;
      break;
  }

  while ((option < 0) || (option >= maxVal)) {
    printf("\n Pick a Relevant Depth Mode: \t");
    const int ret = scanf("%d", &option);
    while (getchar() != '\n' && getchar() != EOF && ret != 1) {
    }
  }

  switch (option) {
    case EXIT:
      bSwitch = true;

      for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
        bPreviewSet(1, false, deviceInd);
        previewThreadCtrlFlag[deviceInd] = false;
        streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
        if (bDetach) {
          if (streamThread[deviceInd].joinable()) {
            streamThread[deviceInd].detach();
          }
          if (previewThread[deviceInd].joinable()) {
            previewThread[deviceInd].detach();
          }
        }
#endif
        if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
          cout << "Device Close success\n";
        }
      }
      if (DeInitialize() > 0) {
        destroyAllWindows();
      }

      std::quick_exit(0);

    case 1:
    case 2:
      return option;

    default:
      depthRangeVec.clear();
      for (size_t itr = 1; itr <= deviceHandleList.size(); itr++) {
        if (exploreOpt == exploreOptions::multipleCam) {
          camAccessId = camId = static_cast<int>(itr);
        }
        if (dataModeVec[camAccessId - 1] <= Depth_IR_RGB_HD_Mode) {
          if (SetDepthRange(deviceHandleList[camAccessId - 1], option - 3) < 0) {
            pError("SetDepthRange");
            continue;
          }
          depthRangeVec[camAccessId - 1] = option - 3;
        } else {
          cout << "Current DataMode doesn't support Depth Range Control\n";
        }
        if (GetDepthRange(deviceHandleList[camAccessId - 1], &depthRangeVec[camAccessId - 1]) < 0) {
          pError("GetDepthRange");
        }
        if (depthRangeVec[camAccessId - 1] == DepthRange::NearRange) {
          depth_min_val = Depth_min = NEAR_MODE_MIN;
          depth_max_val = Depth_max = NEAR_MODE_MAX;
        } else if (depthRangeVec[camAccessId - 1] == DepthRange::FarRange) {
          depth_min_val = Depth_min = FAR_MODE_MIN;
          depth_max_val = Depth_max = FAR_MODE_MAX;
        }
        
        depth_offset = static_cast<int>((Depth_max - Depth_min) * 0.20);
        UpdateColorMap(deviceHandleList[camAccessId - 1], Depth_min, Depth_max + depth_offset, 4);
        if (exploreOpt == exploreOptions::singleCam) {
          break;
        }
      }
      break;
  }
  return 0;
}

/**
 * @brief 		Setting IMU Embedded data
 * @return		bool    return true on successfully setting the IMU Embedded data, else retuns fail.
 */
int imuEmbedDataMenu() {
  int option = -1;
  int camId = -1;
  int camAccessId = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to change IMU Embedded Data Control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
    camAccessId = camId;
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    camAccessId = 1;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Disable IMU Embedded data" << "\n";
      cout << '\t' << "4 - Enable IMU Embedded data" << "\n";

      while ((option < 0) || (option >= 5)) {
        printf("\n Pick a Relevant Option: \t");
        const int ret = scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF && ret != 1) {
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;

        case 3:
        case 4:
          
          if (SetIMUEmbeddedData(deviceHandleList[camAccessId - 1], static_cast<uint8_t>(option) - 3) < 0) {
            pError("SetIMUEmbeddedData");
            return -1;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
          break;
        default:
          printf("Invalid option. Please try again.\n");
          break;
      }
      break;
  }
  return 0;
}

/**
 * @brief 		Setting Depth Range of the Device
 * @return		bool    return true on successfully setting the depthRange of the device, else retuns fail.
 */
int irGainMenu() {
  int option = -1;
  int camId = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to Set or Get the IR Gain value :\t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);
    camId = 1;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Set IR Gain" << "\n";
      cout << '\t' << "4 - Get IR Gain" << "\n";

      uint16_t irGain = 65535;
    char input[100];
int tempOption;

// Clear leftover newline from previous scanf
//int c;
//while ((c = getchar()) != '\n' && c != EOF);

while (1) {
    printf("\n Pick a Relevant Option:\t");

    // if (fgets(input, sizeof(input), stdin) == NULL) {
    //     continue;
    // }

    // --- ADDED FIX: Smart flush to clean the buffer safely ---
    while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
        std::cin.get();
    }
    // ---------------------------------------------------------

    if (fgets(input, sizeof(input), stdin) == NULL) {
      continue;
    }

    // Remove newline
    input[strcspn(input, "\n")] = '\0';

    // Ignore empty input caused by leftover newline
    if (strlen(input) == 0) {
        continue;
    }

    char extra;

    // Strict integer validation
    if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
    }

    // Range validation
    if (tempOption < 0 || tempOption >= 5) {
        printf("\n Please enter a valid option between 0 and 4.\n");
        continue;
    }

    option = tempOption;
    break;
}
Result res = Result::Failed;
      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;

        case 3:
        {
            char input[100];
            int tempGain;

            while (1) {
                printf("\n Pick a Relevant IR Gain between 1 to 100:\t");

                if (fgets(input, sizeof(input), stdin) == NULL) {
                    continue;
                }

                // Remove newline
                input[strcspn(input, "\n")] = '\0';

                // Check empty input
                if (strlen(input) == 0) {
                    printf("\n Invalid input!\n");
                    continue;
                }

                char extra;

                // Strict validation
                if (sscanf(input, "%d %c", &tempGain, &extra) != 1) {
                    printf("\n Invalid input! Numbers only.\n");
                    continue;
                }

                // Range validation
                if (tempGain < 1 || tempGain > 100) {
                    printf("\n IR Gain must be between 1 and 100.\n");
                    continue;
                }

                irGain = static_cast<short>(tempGain);
                break;
            }

            res = SetTOFIRGain(deviceHandleList[camId - 1], irGain);

            if (res > 0) {
                cout << "TOF IR Gain is successfully set to : "
                    << irGain << "\n";
            } else {
                cout << "Setting TOF IR Gain value failed Result res : "
                    << res << "\n";
                return -1;
            }

            if (exploreOpt == exploreOptions::singleCam) {
                break;
            }

            break;
        }
        case 4:
          if (GetTOFIRGain(deviceHandleList[camId - 1], &irGain) > 0) {
            cout << "Current IR Gain value is : " << irGain << "\n";
          } else {
            cout << "Getting IR Gain value failed" << "\n";
            return -1;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
          break;
        default:
          printf("Invalid option. Please try again.\n");
          break;
      }
      break;
  }
  return 0;
}

/**
 * @brief 		Setting Depth Range of the Device
 * @return		bool    return true on successfully setting the depthRange of the device, else retuns fail.
 */
int tofCoringMenu() {
  int option = -1;
  int camId = -1;
  int camAccessId = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Exit" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to change TOF Coring Control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
    camAccessId = camId;
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    camAccessId = 1;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Get TOF Coring" << "\n";
      cout << '\t' << "4 - Set TOF Coring" << "\n";
      uint16_t tofCoring = 0;
      while ((option < 0) || (option >= 5)) {
        printf("\n Pick a Relevant Option: \t");
        const int ret = scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF && ret != 1) {
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;

        case 3:
          PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:maxDeviceIndex : " + to_string(maxDeviceIndex));
          if (GetTOFCoring(deviceHandleList[camAccessId - 1], &tofCoring) > 0) {
            cout << "Current TOF Mask value of camera " << camAccessId << " is : " << tofCoring << "\n";
          } else {
            cout << "Getting TOF Mask value failed" << "\n";
            return -1;
          }

          if (exploreOpt == exploreOptions::singleCam) {
            break;
          } else {
            PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:exploreOpt != exploreOptions::singleCam		" +
                                         to_string(camAccessId - 1));
          }
          break;
        case 4:
          while ((tofCoring <= 1) || (tofCoring > 16383)) {
            printf("\n Pick a Relevant Mask value between 2 to 16383: \t");
            const int ret = scanf("%hd", &tofCoring);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
          }
          if (SetTOFCoring(deviceHandleList[camAccessId - 1], tofCoring) > 0) {
            cout << "TOF Mask value is successfully set to : " << tofCoring << " in camera " << camAccessId - 1 << "\n";
          } else {
            return -1;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            break;
          }
        default:
          break;
      }
      break;
  }
  return 0;
}

/**
 * @brief     Setting and getting exposure of the Device
 * @return    bool    return true on successfully setting or getting the exposure of the device, else retuns fail.
 */
int exposureControlMenu() {
  int option = -1;
  int camId = -1;
  int camAccessId = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Exit" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to change Exposure Control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
    camAccessId = camId;
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    camAccessId = 1;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      if (gDevicesList[camId - 1].devType == DeviceType::CMOS_USB_IRD ||
          gDevicesList[camId - 1].devType == DeviceType::CMOS_MIPI) {
        cout << '\t' << "0 - Exit" << "\n";
        cout << '\t' << "1 - Back" << "\n";
        cout << '\t' << "2 - Main Menu" << "\n";
        cout << '\t' << "3 - Get Exposure" << "\n";
        cout << '\t' << "4 - Set Exposure" << "\n";
        uint16_t exposure = -1;
        while ((option < 0) || (option >= 5)) {
          printf("\n Pick a Relevant Option: \t");
          const int ret = scanf("%d", &option);
          while (getchar() != '\n' && getchar() != EOF && ret != 1) {
          }
        }

        switch (option) {
          case EXIT:
            bSwitch = true;

            for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
              bPreviewSet(1, false, deviceInd);
              previewThreadCtrlFlag[deviceInd] = false;
              streamThreadCtrlFlag[deviceInd] = false;
              if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              }
            }
            if (DeInitialize() > 0) {
              destroyAllWindows();
            }
            std::quick_exit(0);

          case 1:
          case 2:
            return option;

          case 3:
            PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:maxDeviceIndex : " + to_string(maxDeviceIndex));
            if (GetTOFExposure(deviceHandleList[camAccessId - 1], &exposure) > 0) {
              cout << "Current TOF Exposure value of camera " << camAccessId << " is : " << exposure << "\n";
            } else {
              cout << "Getting TOF Exposure value failed" << "\n";
              return -1;
            }

            if (exploreOpt == exploreOptions::singleCam) {
              break;
            } else {
              PrintLog(LOG_HIGH_DEBUG,
                       "\nDepthVistaConsole:exploreOpt != exploreOptions::singleCam   " + to_string(camAccessId - 1));
            }
            break;
          case 4:

            if (gDevicesList[camId - 1].devType == DeviceType::CMOS_USB_IRD) {
              while ((exposure <= 0) || (exposure > 100)) {
                printf("\n Pick a Relevant TOF Exposure value between 1 to 100: \t");
                const int ret = scanf("%hd", &exposure);
                while (getchar() != '\n' && getchar() != EOF && ret != 1) {
                }
              }
            } else if (gDevicesList[camId - 1].devType == DeviceType::CMOS_MIPI) {
              uint16_t maximum = 3900;
              while ((exposure < 0) || (exposure > maximum)) {
                printf("\n Pick a Relevant TOF Exposure value between 0 to %d: \t", maximum);
                const int ret = scanf("%hd", &exposure);
                while (getchar() != '\n' && getchar() != EOF && ret != 1) {
                }
              }
            }
            if (SetTOFExposure(deviceHandleList[camAccessId - 1], exposure) > 0) {
              cout << "TOF Exposure value is successfully set to : " << exposure << " in camera " << camAccessId - 1
                   << "\n";
            } else {
              return -1;
            }
            if (exploreOpt == exploreOptions::singleCam) {
              break;
            }
          default:
            printf("5.Invalid option. Please try again.\n");
            break;
        }
      } else if (gDevicesList[camId - 1].devType == DeviceType::RGBD_IRD) {
        cout << "Camera does not support TOF Exposure control" << "\n";
      }
      break;
  }
  return 0;
}

/**
 * @brief 		Setting TOF Controls to the Device
 * @return		bool    return true on successfully setting the TOF Controls of the device, else retuns fail.
 */
int tofControlSettings() {
  while (true) {
    if (gDevicesList[commonCamId - 1].devType == DeviceType::CMOS_MIPI ||
        gDevicesList[commonCamId - 1].devType == DeviceType::CMOS_USB_IRD) {
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Depth Range" << "\n";
      cout << '\t' << "4 - IR Gain" << "\n";
      cout << '\t' << "5 - Exposure Control" << "\n";
    } else {
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Depth Range" << "\n";
      cout << '\t' << "4 - TOF Coring" << "\n";
      cout << '\t' << "5 - IR Gain" << "\n";
      cout << '\t' << "6 - IMU Embedded data" << "\n";
    }

    int option = -1;
    while ((option < 0) || (option >= 7)) {
      printf("\n Pick a Relevant TOF Control: \t");
      const int ret = scanf("%d", &option);
      while (getchar() != '\n' && getchar() != EOF && ret != 1) {
      }
    }
    int res = 0;
    if (gDevicesList[commonCamId - 1].devType == DeviceType::CMOS_MIPI ||
        gDevicesList[commonCamId - 1].devType == DeviceType::CMOS_USB_IRD) {
      if (option == 4) {
        option = 5;
      } else if (option == 5) {
        option = 7;
      }
    }
    switch (option) {
      case EXIT:
        bSwitch = true;

        for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
          bPreviewSet(1, false, deviceInd);
          previewThreadCtrlFlag[deviceInd] = false;
          streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
          if (bDetach) {
            if (streamThread[deviceInd].joinable()) {
              streamThread[deviceInd].detach();
            }
            if (previewThread[deviceInd].joinable()) {
              previewThread[deviceInd].detach();
            }
          }
#endif
          if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
            cout << "Device Close success\n";
          }
        }
        if (DeInitialize() > 0) {
          destroyAllWindows();
        }

        std::quick_exit(0);

      case 1:
      case 2:
        return 1;
      case 3:
        if (gDevicesList[commonCamId - 1].devType == DeviceType::ITOF_USB ||
            gDevicesList[commonCamId - 1].devType == DeviceType::ITOF_GMSL) {
          cout << "Control not supported \n";
          break;
        } else {
          res = depthRangeMenu();
          if (res == -1) {
            cout << "\n" << "Depth Range Failed" << "\n";
            return 0;
          } else if (res == 1 || res == 0) {
            break;
          }
        }
        return 1;
      case 4:
        if (gDevicesList[commonCamId - 1].devType == DeviceType::ITOF_USB ||
            gDevicesList[commonCamId - 1].devType == DeviceType::ITOF_GMSL) {
          cout << "Control not supported \n";
          break;
        } else {
          res = tofCoringMenu();
          if (res == -1) {
            cout << "\n" << "Depth Coring Failed" << "\n";
            return 0;
          } else if (res == 1 || res == 0) {
            break;
          }
        }
        return 1;
      case 5:
        res = irGainMenu();
        if (res == -1) {
          cout << "TOF IR Gain control failed" << "\n";
          return 0;
        } else if (res == 1 || res == 0) {
          break;
        }
        return 1;
      case 6:
        if (gDevicesList[commonCamId - 1].devType == DeviceType::ITOF_USB ||
            gDevicesList[commonCamId - 1].devType == DeviceType::ITOF_GMSL) {
          cout << "Control not supported \n";
          break;
        } else {
          res = imuEmbedDataMenu();
          if (res == -1) {
            cout << "IMU embed control failed" << "\n";
            return 0;
          } else if (res == 1 || res == 0) {
            break;
          }
        }
        return 1;
      case 7:
        res = exposureControlMenu();
        if (res == -1) {
          cout << "Exposure control failed" << "\n";
          return 0;
        } else if (res == 1 || res == 0) {
          break;
        }
        return 1;
      default:
        printf("Invalid option. Please try again.\n");
        break;
    }
  }
  return 1;
}

/**
 * @brief 		Spatial and temporal filter menu
 * @return		bool    return true on successfully exploration, else retuns fail.
 */
int filtersMenu(uint8_t filterID) {
    int option = -1;
    int camId = -1;

    if (exploreOpt == exploreOptions::multipleCam) {
        std::cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
        std::cout << '\t' << "0 - Back" << "\n";

        for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {
          streaming_index = selecteddevindex[eachDevice] - 1;
          std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
          if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
            deviceName = "STURDeCAM13_TOF";
          }
          cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
               << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
               << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
        }
        char input[100];
        int tempCamId;

      
        while (1) {
          std::cout << "\n Pick a Camera Device to change" << post_processing[filterID] << " filter control : \t";

          // if (fgets(input, sizeof(input), stdin) == NULL) {
          //   continue;
          // }

          // --- ADDED FIX: Smart flush to clean the buffer safely ---
          while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
              std::cin.get();
          }
          // ---------------------------------------------------------

          if (fgets(input, sizeof(input), stdin) == NULL) {
            continue;
          }

          input[strcspn(input, "\n")] = '\0';

          if (strlen(input) == 0) {
            continue;
          }

          char extra;

          if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
            printf("\n Invalid input! Numbers only.\n");
            continue;
          }

          if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
            printf("\n Please enter a valid camera option.\n");
            continue;
          }

          camId = tempCamId;
          break;
        }
    }
    else if (exploreOpt == exploreOptions::singleCam) {
        camId = commonCamId;
     /* int c;
      while ((c = getchar()) != '\n' && c != EOF);*/

    }

    switch (camId) {
    case 0:
        return camId;

    default:

        std::cout << "\n";
        std::cout << '\t' << "0 - Exit" << "\n";
        std::cout << '\t' << "1 - Back" << "\n";
        std::cout << '\t' << "2 - Main Menu" << "\n";
        std::cout << '\t' << "3 - " << post_processing[filterID] << " OFF" << "\n";
        std::cout << '\t' << "4 - " << post_processing[filterID] << " ON" << "\n";

        {
          char inputBuf[100];
          int tempOpt;
          char extra;
          while (1) {
            printf("\n Pick a Relevant Option: \t");
            // if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) continue;
            // --- ADDED FIX: Smart flush to clean the buffer safely ---
            while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
                std::cin.get();
            }
            // ---------------------------------------------------------

            if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) {
              continue;
            }
            if (strcmp(inputBuf, "\n") == 0) continue;
            inputBuf[strcspn(inputBuf, "\n")] = '\0';
            if (sscanf(inputBuf, "%d %c", &tempOpt, &extra) != 1) {
              printf("\n Invalid input! Numbers only.\n");
              continue;
            }
            if (tempOpt < 0 || tempOpt >= 5) {
              printf("\n Please enter a valid option between 0 and 4.\n");
              continue;
            }
            option = tempOpt;
            break;
          }
        }
        if (exploreOpt == exploreOptions::singleCam) {
            camId = 1;
        }
        switch (option) {
        case EXIT:
            bSwitch = true;

            for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
                bPreviewSet(1, false, deviceInd);
                previewThreadCtrlFlag[deviceInd] = false;
                streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
                if (bDetach) {
                    if (streamThread[deviceInd].joinable()) {
                        streamThread[deviceInd].detach();
                    }
                    if (previewThread[deviceInd].joinable()) {
                        previewThread[deviceInd].detach();
                    }
                }
#endif
                if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
                    std::cout << "Device Close success\n";
                }
            }
            if (DeInitialize() > 0) {
                destroyAllWindows();
            }

            std::quick_exit(0);

        case 1:
        case 2:
            return option;

        default:
            if (SetTemporalFilter(deviceHandleList[camId - 1], (option - 3) != 0) < 0) {
                pError("SetFilterType");
                return -1;
            }
            else {
                std::cout << "SetFilterType success " << "\n";
            }
            if (exploreOpt == exploreOptions::singleCam) {
                break;
            }
            break;
        }
        break;
    }
    return 0;
}

int threeDNoiceFilter() {
  int camId = -1;
  int camAccessId = -1;
  if (exploreOpt == exploreOptions::multipleCam) {
    std::cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    std::cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {
      streaming_index = selecteddevindex[eachDevice] - 1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to change 3D Noise filter control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
    camAccessId = camId;
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    camAccessId = 1;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      if (gDevicesList[camId - 1].devType == DeviceType::RGBD_IRD ||
          // gDevicesList[camId - 1].devType == DeviceType::PCIE_IRD ||
          gDevicesList[camId - 1].devType == DeviceType::GMSL_USB) {
        std::cout << "\n";
        std::cout << '\t' << "0 - Exit" << "\n";
        std::cout << '\t' << "1 - Back" << "\n";
        std::cout << '\t' << "2 - Main Menu" << "\n";
        std::cout << '\t' << "3 - 3D Noice Reduction filter OFF" << "\n";
        std::cout << '\t' << "4 - 3D Noice Reduction filter ON" << "\n";
        int option = -1;
        while ((option < 0) || (option >= 5)) {
          printf("\n Pick a Relevant Option: \t");
          const int ret = scanf("%d", &option);
          while (getchar() != '\n' && getchar() != EOF && ret != 1) {
          }
        }

        switch (option) {
          case EXIT:
            bSwitch = true;

            for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
              bPreviewSet(1, false, deviceInd);
              previewThreadCtrlFlag[deviceInd] = false;
              streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
              if (bDetach) {
                if (streamThread[deviceInd].joinable()) {
                  streamThread[deviceInd].detach();
                }
                if (previewThread[deviceInd].joinable()) {
                  previewThread[deviceInd].detach();
                }
              }
#endif
              if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
                std::cout << "Device Close success\n";
              }
            }
            if (DeInitialize() > 0) {
              destroyAllWindows();
            }

            std::quick_exit(0);

          case 1:
          case 2:
            return option;

          default:
            PrintLog(LOG_HIGH_DEBUG,
                     "\nDepthVistaConsole:ctrlDeviceIndex for Set3DNoiseReduction : " + to_string(camAccessId - 1));
            if (Set3DNoiseReduction(deviceHandleList[camAccessId - 1], option - 3) < 0) {
              pError("Set3DNoiseReduction");
              return -1;
            }
            break;
        }
      } else if (gDevicesList[camId - 1].devType == DeviceType::USB_IRD) {
        std::cout << "Camera does not support 3D Noice Reduction Filter" << "\n";
      }
      break;
  }
  return 0;
}

int depthPlanarization() {
  int camId = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    std::cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    std::cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {
      streaming_index = selecteddevindex[eachDevice] - 1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    while (1) {
      printf("\n Pick a Camera Device to change Planarization filter Control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } 
  else if (exploreOpt == exploreOptions::singleCam) {
    camId = 1;
    /*int c;
    while ((c = getchar()) != '\n' && c != EOF);*/
  }

  switch (camId) {
    case 0:
      return camId;

    default:

      std::cout << "\n";
      std::cout << '\t' << "0 - Exit" << "\n";
      std::cout << '\t' << "1 - Back" << "\n";
      std::cout << '\t' << "2 - Main Menu" << "\n";
      std::cout << '\t' << "3 - Planarization OFF" << "\n";
      std::cout << '\t' << "4 - Planarization ON" << "\n";

      int option = -1;
      {
        char inputBuf[100];
        int tempOpt;
        char extra;
        while (1) {
          printf("\n Pick a Relevant Option: \t");
          // if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) continue;
          // --- ADDED FIX: Smart flush to clean the buffer safely ---
          while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
              std::cin.get();
          }
          // ---------------------------------------------------------

          if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) {
            continue;
          }
          if (strcmp(inputBuf, "\n") == 0) continue;
          inputBuf[strcspn(inputBuf, "\n")] = '\0';
          if (sscanf(inputBuf, "%d %c", &tempOpt, &extra) != 1) {
            printf("\n Invalid input! Numbers only.\n");
            continue;
          }
          if (tempOpt < 0 || tempOpt >= 5) {
            printf("\n Please enter a valid option between 0 and 4.\n");
            continue;
          }
          option = tempOpt;
          break;
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();

              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              std::cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;

        default:
          if (SetPlanarization(deviceHandleList[camId - 1], option - 3) < 0) {
            pError("SetPlanarization");
            return -1;
          }
          std::cout << "SetPlanarization success " << "\n";
          break;
      }
      break;
  }
  return 0;
}
/**
 * @brief 		Undistorting Depth
 * @return		bool    return true on successfully Undistorting Depth, else retuns fail.
 */
int depthUndistortion() {
  int option = -1;
  int camId = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    std::cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    std::cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {
      streaming_index = selecteddevindex[eachDevice] - 1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

   
    while (1) {
      printf("\n Pick a Camera Device to change Undistortion filter control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = 1;
   /* int c;
    while ((c = getchar()) != '\n' && c != EOF);*/

  }

  switch (camId) {
    case 0:
      return camId;

    default:
      std::cout << "\n";
      std::cout << '\t' << "0 - Exit" << "\n";
      std::cout << '\t' << "1 - Back" << "\n";
      std::cout << '\t' << "2 - Main Menu" << "\n";
      std::cout << '\t' << "3 - Undistortion OFF" << "\n";
      std::cout << '\t' << "4 - Undistortion ON" << "\n";

      {
        char inputBuf[100];
        int tempOpt;
        char extra;
        while (1) {
          printf("\n Pick a Relevant Option: \t");
          // if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) continue;
          // --- ADDED FIX: Smart flush to clean the buffer safely ---
          while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
              std::cin.get();
          }
          // ---------------------------------------------------------

          if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) {
            continue;
          }
          if (strcmp(inputBuf, "\n") == 0) continue;
          inputBuf[strcspn(inputBuf, "\n")] = '\0';
          if (sscanf(inputBuf, "%d %c", &tempOpt, &extra) != 1) {
            printf("\n Invalid input! Numbers only.\n");
            continue;
          }
          if (tempOpt < 0 || tempOpt >= 5) {
            printf("\n Please enter a valid option between 0 and 4.\n");
            continue;
          }
          option = tempOpt;
          break;
        }
      }
      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              std::cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;

        default:
          PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:ctrlDeviceIndex for SetUnDistortion : " + to_string(camId));
          if (SetUnDistortion(deviceHandleList[camId - 1], option - 3) < 0) {
            pError("SetUndistortion");
            return -1;
          }
          break;
      }
      break;
  }
  return 0;
}

int fillDepthHoleFilter() {
  int option = -1;
  int camId = -1;
  int camAccessId = -1;
  if (exploreOpt == exploreOptions::multipleCam) {
    std::cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    std::cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {
      streaming_index = selecteddevindex[eachDevice] - 1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    while ((camId < 0) || (camId > devices)) {
      printf("\n Pick a Camera Device to change Streaming Mode : \t");
      const int ret = scanf("%d", &camId);
      while (getchar() != '\n' && getchar() != EOF && ret != 1) {
      }
    }
    camAccessId = camId;
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    camAccessId = 1;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      if (gDevicesList[camId - 1].devType == DeviceType::RGBD_IRD ||
          // gDevicesList[camId - 1].devType == DeviceType::PCIE_IRD ||
          gDevicesList[camId - 1].devType == DeviceType::GMSL_USB) {
        if (rgbdMappingflag[camId - 1]) {
          std::cout << "\n";
          std::cout << '\t' << "0 - Exit" << "\n";
          std::cout << '\t' << "1 - Back" << "\n";
          std::cout << '\t' << "2 - Main Menu" << "\n";
          std::cout << '\t' << "3 - Depth Extrapolation filter OFF" << "\n";
          std::cout << '\t' << "4 - Depth Extrapolation filter ON" << "\n";

          while ((option < 0) || (option >= 5)) {
            printf("\n Pick a Relevant Option: \t");
            const int ret = scanf("%d", &option);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
          }
          switch (option) {
            case EXIT:
              bSwitch = true;

              for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
                bPreviewSet(1, false, deviceInd);
                previewThreadCtrlFlag[deviceInd] = false;
                streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
                if (bDetach) {
                  if (streamThread[deviceInd].joinable()) {
                    streamThread[deviceInd].detach();
                  }
                  if (previewThread[deviceInd].joinable()) {
                    previewThread[deviceInd].detach();
                  }
                }
#endif
                if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
                  std::cout << "Device Close success\n";
                }
              }
              if (DeInitialize() > 0) {
                destroyAllWindows();
              }

              std::quick_exit(0);

            case 1:
            case 2:
              return option;

            default:
              PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:ctrlDeviceIndex for SetFillDepthHolesFilter : " +
                                           to_string(camAccessId - 1));
              if (SetDepthExtrapolation(deviceHandleList[camAccessId - 1], option - 3) < 0) {
                pError("SetFillDepthHolesFilter");
                return -1;
              }
              break;
          }
        } else {
          std::cout << "Enable RGB-D mapping to use Depth Extrapolation filter" << "\n";
        }
      } else if (gDevicesList[camId - 1].devType == DeviceType::USB_IRD) {
        std::cout << "Camera does not support Depth Extrapolation filter" << "\n";
      }
      break;
  }
  return 0;
}

int flyingPixelFilter() {
  int option = -1;
  int camId = -1;
  int val = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);
  
    while (1) {
      printf("\n Pick a Camera Device to change Flying Pixel Filter control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    // Clear leftover newline from previous scanf
   /* int c;
    while ((c = getchar()) != '\n' && c != EOF);*/
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Flying Pixel filter OFF" << "\n";
      cout << '\t' << "4 - Flying Pixel filter ON" << "\n";
      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
     char input[100];
     int tempOption;

    //  int c;
    //  while ((c = getchar()) != '\n' && c != EOF);

while (1) {
    printf("\n Pick a Relevant Option:\t");

    // if (fgets(input, sizeof(input), stdin) == NULL) {
    //     continue;
    // }

    // --- ADDED FIX: Smart flush to clean the buffer safely ---
    while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
        std::cin.get();
    }
    // ---------------------------------------------------------

    if (fgets(input, sizeof(input), stdin) == NULL) {
      continue;
    }

    // Remove newline
    input[strcspn(input, "\n")] = '\0';

    // Ignore empty input caused by leftover newline
    if (strlen(input) == 0) {
        continue;
    }

    char extra;

    // Strict integer validation
    if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
    }
       
    // Range validation
    if (tempOption < 0 || tempOption >= 5) {
        printf("\n Please enter a valid option between 0 and 4.\n");
        continue;
    }
     
    option = tempOption;    
    break;
}
      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;

        default:
          if (option == 3) {
              val = 2000;
          }
          else if(option == 4) {
              val = 5;
          }
          PrintLog(LOG_HIGH_DEBUG,
                   "\nDepthVistaConsole:ctrlDeviceIndex for SetFlyingPixelFilter : " + to_string(camId - 1));
          if (SetFlyingPixelFilter(deviceHandleList[camId - 1], val) < 0) {
            pError("SetFlyingPixelFilter");
            return -1;
          }
          break;
      }
      break;
  }
  return 0;
}

int staticDefect() {
  int option = -1;
  int camId = -1;
  uint16_t getStatus = 0;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    while ((camId < 0) || (camId > static_cast<int>(devices))) {
      printf("\n Pick a Camera Device to change Static Defect control : \t");
      const int ret = scanf("%d", &camId);
      while (getchar() != '\n' && getchar() != EOF && ret != 1) {
      }
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Static Defect OFF" << "\n";
      cout << '\t' << "4 - Static Defect ON" << "\n";
      cout << '\t' << "5 - get current defect status:" << '\n';
      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      while ((option < 0) || (option >= 6)) {
        printf("\n Pick a Relevant Option: \t");
        const int ret = scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF && ret != 1) {
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;

        case 5:
          if (GetStaticDefect(deviceHandleList[camId - 1], &getStatus) < 0) {
            pError("GetStaticDefect");
            return -1;
          }
          cout << "Current Static defect status : " << getStatus << "\n";
          break;

        default:
          PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:ctrlDeviceIndex for SetStaticDefect : " + to_string(camId - 1));
          if (SetStaticDefect(deviceHandleList[camId - 1], option - 3) < 0) {
            pError("SetStaticDefect");
            return -1;
          }
          std::cout << "Static Defect set Successfully" << '\n';

          break;
      }
      break;
  }
  return 0;
}

int embeddedData() {
  int option = -1;
  int camId = -1;
  uint16_t getStatus = 0;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    while ((camId < 0) || (camId > static_cast<int>(devices))) {
      printf("\n Pick a Camera Device to change Embedded data control : \t");
      const int ret = scanf("%d", &camId);
      while (getchar() != '\n' && getchar() != EOF && ret != 1) {
      }
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Embedded Data OFF" << "\n";
      cout << '\t' << "4 - Embedded Data ON" << "\n";
      cout << '\t' << "5 - get current status value" << "\n";
      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      while ((option < 0) || (option >= 6)) {
        printf("\n Pick a Relevant Option: \t");
        const int ret = scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF && ret != 1) {
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
        case 5:
          if (GetEmbeddedData(deviceHandleList[camId - 1], &getStatus) < 0) {
            pError("GetEmbeddedData");
            return -1;
          }
          cout << "Current Embedded data status : " << getStatus << "\n";
          break;

        default:
          PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:ctrlDeviceIndex for SetEmbeddedData : " + to_string(camId - 1));
          if (SetEmbeddedData(deviceHandleList[camId - 1], option - 3) < 0) {
            pError("SetEmbeddedData");
            return -1;
          }
          break;
      }
      break;
  }
  return 0;
}

int coldDefect() {
  int option = -1;
  int camId = -1;
  uint16_t getStatus = 0;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    while ((camId < 0) || (camId > static_cast<int>(devices))) {
      printf("\n Pick a Camera Device to change Cold Defect control : \t");
      const int ret = scanf("%d", &camId);
      while (getchar() != '\n' && getchar() != EOF && ret != 1) {
      }
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Cold Defect OFF" << "\n";
      cout << '\t' << "4 - Cold Defect ON" << "\n";
      cout << '\t' << "5 - Get current status" << "\n";
      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      while ((option < 0) || (option >= 6)) {
        printf("\n Pick a Relevant Option: \t");
        const int ret = scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF && ret != 1) {
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;

        case 5:
          if (GetColdDefect(deviceHandleList[camId - 1], &getStatus) < 0) {
            pError("GetColdDefect");
            return -1;
          }
          cout << "Current Cold defect status : " << getStatus << "\n";
          break;

        default:
          PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:ctrlDeviceIndex for SetColdDefect : " + to_string(camId - 1));
          if (SetColdDefect(deviceHandleList[camId - 1], option - 3) < 0) {
            pError("SetColdDefect");
            return -1;
          }

          std::cout << "Cold Defect set Successfully" << '\n';

          break;
      }
      break;
  }
  return 0;
}

int hotDefect() {
  int option = -1;
  int camId = -1;
  uint16_t getStatus = 0;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to change Hot Defect control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Hot Defect OFF" << "\n";
      cout << '\t' << "4 - Hot Defect ON" << "\n";
      cout << '\t' << "5 - Get Current Status" << "\n";

      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      while ((option < 0) || (option >= 6)) {
        printf("\n Pick a Relevant Option: \t");
        const int ret = scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF && ret != 1) {
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
        case 5:
          if (GetHotDefect(deviceHandleList[camId - 1], &getStatus) < 0) {
            pError("GetHotDefect");
            return -1;
          }
          cout << "Current Hot defect status : " << getStatus << "\n";
          break;

        default:
          PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:ctrlDeviceIndex for SetHotDefect : " + to_string(camId - 1));
          if (SetHotDefect(deviceHandleList[camId - 1], (option - 3)) < 0) {
            pError("SetHotDefect");
            return -1;
          }
          std::cout << "Hot Defect set Successfully" << '\n';
          break;
      }
      break;
  }
  return 0;
}

int SensorDriverRegisterRead() {
  int option = -1;
  int camId = -1;
  uint16_t getStatus = 0;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to change Sensor Register Read control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
  }

  uint8_t mode = 0;      // 0 - I2C , 1 - SPI
  uint16_t address = 0;  // Register address
  uint16_t value = 0;    // Register value

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Set Value" << "\n";
      cout << '\t' << "4 - Get Value" << "\n";
      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      while ((option < 0) || (option >= 5)) {
        printf("\n Pick a Relevant Option: \t");
        const int ret = scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF && ret != 1) {
        }
      }

      // while ((option < 0) || (option >= 4)) {
      printf("\n Enter the mode : \t");
      uint8_t modeI = 0;
      const int ret = scanf("%hhx", &modeI);
      mode = static_cast<uint8_t>(modeI);
      // while (getchar() != '\n' && getchar() != EOF && ret != 1) {
      // }
      // }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;

        case 4: {
          printf("\n Enter Register address : \t");
          address = 0;
          int ret = scanf("%hx", &address);
          while (getchar() != '\n' && getchar() != EOF && ret != 1) {
          }

          std::cout << "Address : " << address << "\n";
          if (GetSensorDriverRegisterRead(deviceHandleList[camId - 1], mode, static_cast<uint16_t>(address), &value) <
              0) {
            pError("GetSensorDriverRegisterRead");
            return -1;
          }
          cout << "Sensor Register Read value : 0x" << std::hex << value << "\n";
          break;
        }

        default: {
          PrintLog(LOG_HIGH_DEBUG,
                   "\nDepthVistaConsole:ctrlDeviceIndex for SetSensorRegisterRead : " + to_string(camId - 1));
          printf("\n Enter Register address : \t");
          address = 0;
          int ret_ = scanf("%hx", &address);
          while (getchar() != '\n' && getchar() != EOF && ret_ != 1) {
          }

          printf("\n Enter value : \t");
          int valueI = 0;
          ret_ = scanf("%hx", &valueI);
          while (getchar() != '\n' && getchar() != EOF && ret_ != 1) {
          }
          if (SetSensorDriverRegisterWrite(deviceHandleList[camId - 1], mode, static_cast<uint16_t>(address),
                                           static_cast<uint16_t>(valueI)) < 0) {
            pError("SetSensorRegisterRead");
            return -1;
          }

          std::cout << "Sensor Register Write Successfully" << '\n';

          break;
        }
      }
      break;
  }
  return 0;
}

int getTemperature() {
  int option = -1;
  int camId = -1;
  float getStatus = 0.0f;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    while ((camId < 0) || (camId > static_cast<int>(devices))) {
      printf("\n Pick a Camera Device to change Cold Defect control : \t");
      const int ret = scanf("%d", &camId);
      while (getchar() != '\n' && getchar() != EOF && ret != 1) {
      }
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    // Clear leftover newline from previous scanf
    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - get Laserboard Temperature" << "\n";
      cout << '\t' << "4 - get Moduleboard Temperature" << "\n";
      cout << '\t' << "5 - get Sensor die Temperature" << "\n";
      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      int option = -1;

    char input[100];
    int tempOption;

   

    while (1) {

        printf("\n Pick a Relevant Option:\t");

        // if (fgets(input, sizeof(input), stdin) == NULL) {
        //     continue;
        // }

        // --- ADDED FIX: Smart flush to clean the buffer safely ---
        while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
            std::cin.get();
        }
        // ---------------------------------------------------------

        if (fgets(input, sizeof(input), stdin) == NULL) {
          continue;
        }

        // Remove newline
        input[strcspn(input, "\n")] = '\0';

        // Ignore empty input caused by leftover newline
        if (strlen(input) == 0) {
            continue;
        }

        char extra;

        // Strict integer validation
        if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
            printf("\n Invalid input! Numbers only.\n");
            continue;
        }

        // Range validation
        if (tempOption < 0 || tempOption >= 6) {
            printf("\n Please enter a valid option between 0 and 5.\n");
            continue;
        }

        option = tempOption;
        break;
    }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
        case 3:
          if (GetLaserBoardTemperatureData(deviceHandleList[camId - 1], &getStatus) < 0) {
            pError("GetLaserBoard");
            return -1;
          }
          cout << "Laser Board Temperature data: " << getStatus << "\n";
          break;

        case 4:
          if (GetBaseBoardTemperatureData(deviceHandleList[camId - 1], &getStatus) < 0) {
            pError("Moduleboard");
            return -1;
          }
          cout << "Module Board Temperature data: " << getStatus << "\n";
          break;

        case 5:
          if (GetSensorDieTemperatureData(deviceHandleList[camId - 1], &getStatus) < 0) {
            pError("GetSensorDie");
            return -1;
          }
          cout << "Sensor Die Temperature data: " << getStatus << "\n";
          break;

        default:
          break;
      }
      break;
  }
  return 0;
}

int spatialmenu() {
  int option = -1;
  int camId = -1;
  int value = -1;
  int res = -1;
  float getStatus = 0.0f;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

  
    while (1) {
      printf("\n Pick a Camera Device to Enable or Disable the Spatial filter : \t");

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Spatial OFF" << "\n";
      cout << '\t' << "4 - Spatial ON" << "\n";

      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      {
        char inputBuf[100];
        int tempOpt;
        char extra;
        while (1) {
          printf("\n Pick a Relevant Option: \t");
          if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) continue;
          if (strcmp(inputBuf, "\n") == 0) continue;
          inputBuf[strcspn(inputBuf, "\n")] = '\0';
          if (sscanf(inputBuf, "%d %c", &tempOpt, &extra) != 1) {
            printf("\n Invalid input! Numbers only.\n");
            continue;
          }
          if (tempOpt < 0 || tempOpt >= 5) {
            printf("\n Please enter a valid option between 0 and 4.\n");
            continue;
          }
          option = tempOpt;
          break;
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
        case 3:
        case 4:
          res = SetDepthSpatialFilter(deviceHandleList[camId - 1], option - 3);
          if (res < 0) {
            cout << "Set Spatial Filter failed" << "\n";
            // return - 1;
          } else {
            cout << "Set Spatial Filter success" << "\n";
          }
          break;

        default:
          break;
      }
      break;
  }
  return 0;
}

int depthnoisemenu() {
  int option = -1;
  int camId = -1;
  int value = -1;
  int res = -1;
  float getStatus = 0.0f;
  uint16_t getvalue = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to Enable or Disable the Denoise : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);
    camId = commonCamId;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Set Depth Denoise" << "\n";
      cout << '\t' << "4 - Get Depth Denoise" << "\n";

      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      char input[100];
int tempOption;

// Clear leftover newline from previous scanf


while (1) {
    printf("\n Pick a Relevant Option:\t");

    // if (fgets(input, sizeof(input), stdin) == NULL) {
    //     continue;
    // }

    // --- ADDED FIX: Smart flush to clean the buffer safely ---
    while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
        std::cin.get();
    }
    // ---------------------------------------------------------

    if (fgets(input, sizeof(input), stdin) == NULL) {
      continue;
    }

    // Remove newline
    input[strcspn(input, "\n")] = '\0';

    // Empty input check
    if (strlen(input) == 0) {
        printf("\n Invalid input!\n");
        continue;
    }

    char extra;

    // Strict integer validation
    if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
    }

    // Range validation
    if (tempOption < 0 || tempOption >= 5) {
        printf("\n Please enter a valid option between 0 and 4.\n");
        continue;
    }

    option = tempOption;
    break;
}
      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
        case 3:
        {
            char input[100];
            int tempValue;

            while (1) {
                printf("\n Enter Depth Denoise value between 0 to 15:\t");

                if (fgets(input, sizeof(input), stdin) == NULL) {
                    continue;
                }

                // Remove newline
                input[strcspn(input, "\n")] = '\0';

                // Empty input check
                if (strlen(input) == 0) {
                    printf("\n Invalid input!\n");
                    continue;
                }

                char extra;

                // Strict integer validation
                if (sscanf(input, "%d %c", &tempValue, &extra) != 1) {
                    printf("\n Invalid input! Numbers only.\n");
                    continue;
                }

                // Range validation
                if (tempValue < 0 || tempValue > 15) {
                    printf("\n Depth Denoise value must be between 0 and 15.\n");
                    continue;
                }

                value = tempValue;
                break;
            }

            if (SetDepthNoise(deviceHandleList[camId - 1],
                              static_cast<uint16_t>(value)) > 0) {

                cout << "Depth Denoise value is successfully set to : "
                    << value
                    << " in camera "
                    << commonCamId - 1
                    << "\n";

            } else {
                value = -1;
                return false;
            }

            if (exploreOpt == exploreOptions::singleCam) {
                value = -1;
                break;
            }

            break;
        }
        case 4:
          if (GetDepthNoise(deviceHandleList[camId - 1], &getvalue) > 0) {
            cout << "Depth Denoise value of the camera " << camId- 1 << " : " << getvalue << "\n ";
          } else {
            cout << "Get Depth Denoise value failed" << "\n";
            getvalue = -1;
            return false;
          }
          break;

        default:
          break;
      }
      break;
  }
  return 0;
}

int confidencemenu() {
  int option = -1;
  int camId = -1;
  int value = -1;
  int res = -1;
  float getStatus = 0.0f;
  uint16_t getvalue = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to Set or Get the Confidence value : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    // Clear leftover newline from previous scanf
    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Set Confidence Threshold" << "\n";
      cout << '\t' << "4 - Get Confidence Threshold" << "\n";

      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      char input[100];
      int tempOption;



while (1) {
    printf("\n Pick a Relevant Option:\t");

    // if (fgets(input, sizeof(input), stdin) == NULL) {
    //     continue;
    // }

    // --- ADDED FIX: Smart flush to clean the buffer safely ---
    while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
        std::cin.get();
    }
    // ---------------------------------------------------------

    if (fgets(input, sizeof(input), stdin) == NULL) {
      continue;
    }

    // Remove newline
    input[strcspn(input, "\n")] = '\0';

    // Ignore pure newline from leftover buffer
    if (strlen(input) == 0) {
        continue;
    }

    char extra;

    // Strict integer validation
    if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
    }

    // Range validation
    if (tempOption < 0 || tempOption >= 5) {
        printf("\n Please enter a valid option between 0 and 4.\n");
        continue;
    }

    option = tempOption;
    break;
}
      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
case 3:
{
    char input[100];
    int tempValue;

    while (1) {
        printf("\nEnter Confidence Threshold value between 0 to %d: ", CONF_THRESHOLD_MAX);

        if (fgets(input, sizeof(input), stdin) == NULL) {
            continue;
        }

        // Remove newline
        input[strcspn(input, "\n")] = '\0';

        // Empty input check
        if (strlen(input) == 0) {
            printf("\nInvalid input!\n");
            continue;
        }

        char extra;

        // Strict integer validation
        if (sscanf(input, "%d %c", &tempValue, &extra) != 1) {
            printf("\nInvalid input! Numbers only.\n");
            continue;
        }

        // Range validation
        if (tempValue < 0 || tempValue > CONF_THRESHOLD_MAX) {
            printf("\nConfidence Threshold must be between 0 and %d.\n", CONF_THRESHOLD_MAX);
            continue;
        }

        value = tempValue;
        break;
    }

    if (SetConfidenceThreshold(deviceHandleList[camId - 1],
                               static_cast<uint16_t>(value)) > 0) {

        cout << "Confidence Threshold value is successfully set to : "
             << value
             << " in camera "
             << camId - 1
             << "\n";

    } else {
        value = -1;
        return false;
    }

    if (exploreOpt == exploreOptions::singleCam) {
        value = -1;
        break;
    }

    value = -1;
    break;
}
        case 4:
          if (GetConfidenceThreshold(deviceHandleList[camId - 1], &getvalue) > 0) {
            cout << "Confidence Threshold value of the camera " << camId - 1 << " : " << getvalue << "\n ";
          } else {
            cout << "Get Confidence Threshold value failed" << "\n";
            getvalue = -1;
            return false;
          }
          break;

        default:
          break;
      }
      break;
  }
  return 0;
}

int frequencymenu() {
  int option = -1;
  int camId = -1;
  int value = -1;
  int res = -1;
  float getStatus = 0.0f;
  uint16_t getvalue = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to Get or Set the Frequency Merge : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Set Frequency Merge" << "\n";
      cout << '\t' << "4 - Get Frequency Merge" << "\n";

      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
      while ((option < 0) || (option >= 5)) {
        printf("\n Pick a Relevant Option: \t");
        const int ret = scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF && ret != 1) {
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
        case 3:
          while ((value < 0) || (value > 2)) {
            printf("\n Enter Frequency Merge value between 0 to 2: \t");
            int ret = scanf("%d", &value);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
          }
          if (SetFrequencyMerge(deviceHandleList[camId - 1], static_cast<uint16_t>(value)) > 0) {
            cout << "Frequency Merge Data is successfully set to : " << value << " in camera " << commonCamId - 1
                 << "\n";
          } else {
            value = -1;
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            value = -1;
            break;
          }
          value = -1;
          break;
        case 4:
          if (GetFrequencyMerge(deviceHandleList[camId - 1], &getvalue) > 0) {
            cout << "Frequency Merge value of the camera " << camId- 1 << " : " << getvalue << "\n ";
          } else {
            cout << "Get Frequency Merge value failed" << "\n";
            getvalue = -1;
            return false;
          }
          break;

        default:
          break;
      }
      break;
  }
  return 0;
}

int intgtimemenu() {
  int option = -1;
  int camId = -1;
  int value = -1;
  int res = -1;
  float getStatus = 0.0f;
  uint16_t getvalue = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {

      streaming_index = selecteddevindex[eachDevice]-1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to Get or Set the Integration Time : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    // Clear leftover newline from previous scanf
    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Set Integration Time" << "\n";
      cout << '\t' << "4 - Get Integration Time" << "\n";

      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
     char input[100];
int tempOption;



while (1) {
    printf("\n Pick a Relevant Option:\t");

    // if (fgets(input, sizeof(input), stdin) == NULL) {
    //     continue;
    // }

    // --- ADDED FIX: Smart flush to clean the buffer safely ---
    while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
        std::cin.get();
    }
    // ---------------------------------------------------------

    if (fgets(input, sizeof(input), stdin) == NULL) {
      continue;
    }

    // Remove newline
    input[strcspn(input, "\n")] = '\0';

    // Ignore empty input caused by leftover newline
    if (strlen(input) == 0) {
        continue;
    }

    char extra;

    // Strict integer validation
    if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
    }

    // Range validation
    if (tempOption < 0 || tempOption >= 5) {
        printf("\n Please enter a valid option between 0 and 4.\n");
        continue;
    }

    option = tempOption;
    break;
}
      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
        case 3:
{
    char input[100];
    int tempValue;

    while (1) {
	uint32_t maxIntegrationTime = INTEGRATION_TIME_MAX;
          FpsMode curMode;
          if (GetFPSMode(deviceHandleList[camId - 1], &curMode) < 0) {
            cout << "Getting FPS Mode Failed while setting Integration Time" << "\n";
          }
          DataMode curDataMode;
          if (GetDataMode(deviceHandleList[camId - 1], &curDataMode) < 0) {
            cout << "Getting Data Mode Failed while setting Integration Time" << "\n";
          }
          if (curMode == FpsMode::TOF_50Fps && curDataMode == DataMode::Depth_IR_Conf_Mode) {
            maxIntegrationTime = INTEGRATION_TIME_MAX_VGA_50_FPS;
          }
        printf("\n Enter Integration time value between %d to %d:\t", INTEGRATION_TIME_MIN, maxIntegrationTime);

        if (fgets(input, sizeof(input), stdin) == NULL) {
            continue;
        }

        // Remove newline
        input[strcspn(input, "\n")] = '\0';

        // Empty input check
        if (strlen(input) == 0) {
            printf("\n Invalid input!\n");
            continue;
        }

        char extra;

        // Strict integer validation
        if (sscanf(input, "%d %c", &tempValue, &extra) != 1) {
            printf("\n Invalid input! Numbers only.\n");
            continue;
        }

        // Range validation
        if (tempValue < INTEGRATION_TIME_MIN || tempValue > maxIntegrationTime) {
            printf("\n Integration time must be between %d and %d.\n", INTEGRATION_TIME_MIN, maxIntegrationTime);
            continue;
        }

        value = tempValue;
        break;
    }

    if (SetIntegrationTime(deviceHandleList[camId - 1],
                           static_cast<uint16_t>(value)) > 0) {

        cout << "Integration Data is successfully set to : "
             << value
             << " in camera "
             << commonCamId - 1
             << "\n";

    } else {
        value = -1;
        return false;
    }

    if (exploreOpt == exploreOptions::singleCam) {
        value = -1;
        break;
    }

    value = -1;
    break;
}
        case 4:
          if (GetIntegrationTime(deviceHandleList[camId - 1], &getvalue) > 0) {
            cout << "Integration time value of the camera " << camId- 1 << " : " << getvalue << "\n ";
          } else {
            cout << "Get Integration time value failed" << "\n";
            getvalue = -1;
            return false;
          }
          break;

        default:
          printf("Invalid option. Please try again.\n");
          break;
      }
      break;
  }
  return 0;
}

int confmodemenu() {
  int option = -1;
  int camId = -1;
  int value = -1;
  int res = -1;
  float getStatus = 0.0f;
  uint16_t getvalue = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {
      streaming_index = selecteddevindex[eachDevice] - 1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to Set the Confidence Mode : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    // Clear leftover newline from previous scanf
    //  int c;
    //  while ((c = getchar()) != '\n' && c != EOF);
  }

  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Set Confidence Mode" << "\n";
      cout << '\t' << "4 - Get Confidence Mode" << "\n";

      if (exploreOpt == exploreOptions::singleCam) {
        camId = 1;
      }
    int option = -1;

    char input[100];
    int tempOption;

    

    while (1) {

        printf("\n Pick a Relevant Option:\t");

        // if (fgets(input, sizeof(input), stdin) == NULL) {
        //     continue;
        // }

        // --- ADDED FIX: Smart flush to clean the buffer safely ---
        while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
            std::cin.get();
        }
        // ---------------------------------------------------------

        if (fgets(input, sizeof(input), stdin) == NULL) {
          continue;
        }

        // Remove newline
        input[strcspn(input, "\n")] = '\0';

        // Ignore empty input caused by leftover newline
        if (strlen(input) == 0) {
            continue;
        }

        char extra;

        // Strict integer validation
        if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
            printf("\n Invalid input! Numbers only.\n");
            continue;
        }

        // Range validation
        if (tempOption < 0 || tempOption >= 5) {
            printf("\n Please enter a valid option between 0 and 4.\n");
            continue;
        }

        option = tempOption;
        break;
    }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
        case 3: {
          option = -1;
          cout << "\n";
          cout << '\t' << "0 - Exit" << "\n";
          cout << '\t' << "1 - Back" << "\n";
          cout << '\t' << "2 - Main Menu" << "\n";
          cout << '\t' << "3 - Amplitude Frame" << "\n";
          cout << '\t' << "4 - SNR Frame" << "\n";

          {
            char inputBuf[100];
            int tempOpt;
            char extra;
            while (1) {
              printf("\n Pick a Relevant Option : \t");
              if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) continue;
              if (strcmp(inputBuf, "\n") == 0) continue;
              inputBuf[strcspn(inputBuf, "\n")] = '\0';
              if (sscanf(inputBuf, "%d %c", &tempOpt, &extra) != 1) {
                printf("\n Invalid input! Numbers only.\n");
                continue;
              }
              if (tempOpt < 0 || tempOpt > 4) {
                printf("\n Please enter a valid option between 0 and 4.\n");
                continue;
              }
              option = tempOpt;
              break;
            }
          }
          if (option == 0) {
            bSwitch = true;

            for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
              bPreviewSet(1, false, deviceInd);
              previewThreadCtrlFlag[deviceInd] = false;
              streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
              if (bDetach) {
                if (streamThread[deviceInd].joinable()) {
                  streamThread[deviceInd].detach();
                }
                if (previewThread[deviceInd].joinable()) {
                  previewThread[deviceInd].detach();
                }
              }
#endif
              if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
                cout << "Device Close success\n";
              }
            }
            if (DeInitialize() > 0) {
              destroyAllWindows();
            }

            std::quick_exit(0);
          }
          else if (option == 1 || option == 2) {
            return option;
          }
          else if (option == 3) {
            value = 0;
          } else {
            value = 4;
          }
          if (SetConfidenceMode(deviceHandleList[camId - 1], static_cast<uint16_t>(value)) > 0) {
            cout << "Confidence Mode is successfully set in camera " << commonCamId - 1 << "\n";
          } else {
            value = -1;
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            value = -1;
            break;
          }
          value = -1;
          break;
        }
        case 4:
          if (GetConfidenceMode(deviceHandleList[camId - 1], &getvalue) > 0) {
            if (getvalue == 0) {
              cout << "Confidence Mode of the camera " << commonCamId - 1 << " : Amplitude Frame" << "\n ";
            } else if (getvalue == 4) {
              cout << "Confidence Mode of the camera " << commonCamId - 1 << " : SNR Frame" << "\n ";
            }
            else{
              cout << "Confidence Mode of the camera " << commonCamId - 1 << " : Unknown(" << getvalue << ")\n ";
            }
          } else {
              cout << "Confidence Mode Get failed for camera " << commonCamId - 1 << "\n ";

            value = -1;
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            value = -1;
            break;
          }
          value = -1;
          break;
        default:
          printf("Invalid option. Please try again.\n");
          break;
      }
      break;
  }
  return 0;
}

int laserSafetyMenu() {
  int option = -1;
  int camId = -1;
  int value = -1;
  int res = -1;
  float getStatus = 0.0f;
  uint16_t getvalue = -1;

  if (exploreOpt == exploreOptions::multipleCam) {

    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {
      streaming_index = selecteddevindex[eachDevice] - 1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to Set or Get the Laser Safety :\t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }


  } 
  else if (exploreOpt == exploreOptions::singleCam) {
    camId = 1;
    // int c;
    //while ((c = getchar()) != '\n' && c != EOF);    
  }


  switch (camId) {
    case 0:
      return camId;

    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Set Laser Safety Mode" << "\n";
      cout << '\t' << "4 - Get Laser Safety Mode" << "\n";
      cout << '\t' << "5 - Laser Reset" << "\n";

      
      int option = -1;
      char input[100];
      int tempOption;

      // Clear leftover newline from previous scanf
      /*int c;
      while ((c = getchar()) != '\n' && c != EOF);*/

      while (1) {
        printf("\n Pick a Relevant Option:\t");
        // if (fgets(input, sizeof(input), stdin) == NULL) {
        //   continue;
        // }

        // --- ADDED FIX: Smart flush to clean the buffer safely ---
        while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
            std::cin.get();
        }
        // ---------------------------------------------------------

        if (fgets(input, sizeof(input), stdin) == NULL) {
          continue;
        }
        // Remove newline
        input[strcspn(input, "\n")] = '\0';
        // Ignore empty input caused by leftover newline
        if (strlen(input) == 0) {
          continue;
        }
        char extra;
        // Strict integer validation
        if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
          printf("\n Invalid input! Numbers only.\n");
          continue;
        }
        // Range validation
        if (tempOption < 0 || tempOption >= 6) {
          printf("\n Please enter a valid option between 0 and 5.\n");
          continue;
        }
        option = tempOption;
        break;
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          return option;
        case 3: {
          option = -1;
          cout << "\n";
          cout << '\t' << "0 - Exit" << "\n";
          cout << '\t' << "1 - Back" << "\n";
          cout << '\t' << "2 - Main Menu" << "\n";
          cout << '\t' << "3 - Disable Laser Safety" << "\n";
          cout << '\t' << "4 - Enable Laser Safety" << "\n";

          int option = -1;
          char input[100];
          int tempOption;

          // Clear leftover newline from previous scanf
          //int c;
          //while ((c = getchar()) != '\n' && c != EOF);

          while (1) {
            printf("\n Pick a Relevant Option:\t");
            // if (fgets(input, sizeof(input), stdin) == NULL) {
            //   continue;
            // }

            // --- ADDED FIX: Smart flush to clean the buffer safely ---
            while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
                std::cin.get();
            }
            // ---------------------------------------------------------

            if (fgets(input, sizeof(input), stdin) == NULL) {
              continue;
            }
            // Remove newline
            input[strcspn(input, "\n")] = '\0';
            // Ignore empty input caused by leftover newline
            if (strlen(input) == 0) {
              continue;
            }
            char extra;
            // Strict integer validation
            if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
              printf("\n Invalid input! Numbers only.\n");
              continue;
            }
            // Range validation
            if (tempOption < 0 || tempOption >= 5) {
              printf("\n Please enter a valid option between 0 and 4.\n");
              continue;
            }
            option = tempOption;
            break;
          }
          if (option == 0) {
            bSwitch = true;

            for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
              bPreviewSet(1, false, deviceInd);
              previewThreadCtrlFlag[deviceInd] = false;
              streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
              if (bDetach) {
                if (streamThread[deviceInd].joinable()) {
                  streamThread[deviceInd].detach();
                }
                if (previewThread[deviceInd].joinable()) {
                  previewThread[deviceInd].detach();
                }
              }
#endif
              if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
                cout << "Device Close success\n";
              }
            }
            if (DeInitialize() > 0) {
              destroyAllWindows();
            }

            std::quick_exit(0);
          }
          else if (option == 1 || option == 2) {
            return option;
          }

          if (SetLaserSafety(deviceHandleList[camId - 1], static_cast<uint16_t>(option - 3)) > 0) {
            cout << "Laser Safety Mode is successfully set in camera " << camId - 1 << "\n";
          } else {
            value = -1;
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            value = -1;
            break;
          }
          value = -1;
          break;
        }
        case 4:
          if (GetLaserSafety(deviceHandleList[camId - 1], &getvalue) > 0) {
            if (getvalue == 0) {
              cout << "Laser Safety of the camera " << camId - 1 << " :  OFF" << "\n ";
            } else if (getvalue == 1) {
              cout << "Laser Safety of the camera " << camId - 1 << " :  ON" << "\n ";
            }
          } else {
            value = -1;
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            value = -1;
            break;
          }
          value = -1;
          break;
        case 5: {
          Result lasResult = LaserReset(deviceHandleList[camId - 1]);
          if (lasResult > 0) {
            cout << "Laser Reset successful for camera " << camId - 1 << "\n";
          } else {
            cout << "Laser Reset failed for camera " << camId - 1 << "\n";
            value = -1;
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            value = -1;
            break;
          }
          value = -1;
          break;
        }
        default:
          printf("Invalid option. Please try again.\n");
          break;
      }
      break;
  }
  return 0;
}
/**
* @brief 		Exploring the post processing features
* @return		int		1 on successfull exploration of post processing filters.
                                                                                                0 on fail
*/

int postProcessing() {
  int maxval = 0;
  DeviceInfo devInfo;
  if (exploreOpt == exploreOptions::singleCam) {

    devInfo = gDevicesList[commonCamId - 1];
  } else {
    if (successCam.size() > 0) {
      devInfo = successCam[0];
    } else {
      return false;
    }
  }
  // Consume leftover newline from previous scanf in camera properties menu

  // int ch;
  // while ((ch = getchar()) != '\n' && ch != EOF);

  while (true) {
    if (devInfo.devType == DeviceType::ITOF_GMSL || devInfo.devType == DeviceType::ITOF_USB) {
      std::cout << "\n";
      std::cout << '\t' << "0 - Exit" << "\n";
      std::cout << '\t' << "1 - Back" << "\n";
      std::cout << '\t' << "2 - Main " << "\n";
      std::cout << '\t' << "3 - Planarization" << "\n";
      std::cout << '\t' << "4 - Temporal Filter" << "\n";
      std::cout << '\t' << "5 - Undistort depth" << "\n";
      std::cout << '\t' << "6 - Depth Spatial Filter" << "\n";
      maxval = 7;
    } else {
      std::cout << "\n";
      std::cout << '\t' << "0 - Exit" << "\n";
      std::cout << '\t' << "1 - Back" << "\n";
      std::cout << '\t' << "2 - Main " << "\n";
      std::cout << '\t' << "3 - Planarization" << "\n";
      std::cout << '\t' << "4 - Temporal Filter" << "\n";
      std::cout << '\t' << "5 - Undistort depth" << "\n";
      std::cout << '\t' << "6 - Flying Pixel filter" << "\n";
      std::cout << '\t' << "7 - 3D Noice Reduction filter" << "\n";
      std::cout << '\t' << "8 - Depth Extrapolation filter (Enable RGB-D mapping before use)" << "\n";
      std::cout << '\t' << "9 - Depth Spatial Filter" << "\n";
      maxval = 9;
    }

    int option = -1;
    char inputBuf[100];
    int tempOpt;
    char extra;

    /*int ch;
    while ((ch = getchar()) != '\n' && ch != EOF);*/
      
    while (1) {
        printf("\n Pick a Relevant Option: \t");
        //fflush(stdout);
        //if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) continue;

        //  if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) {
        //   continue;
        // }

        // --- ADDED FIX: Smart flush to clean the buffer safely ---
        while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
            std::cin.get();
        }
        // ---------------------------------------------------------

        if (fgets(inputBuf, sizeof(inputBuf), stdin) == NULL) {
          continue;
        }

        inputBuf[strcspn(inputBuf, "\n")] = '\0';


        if (strlen(inputBuf) == 0) continue;


        if (sscanf(inputBuf, "%d %c", &tempOpt, &extra) != 1) {
            printf("\n Invalid input! Numbers only.\n");
            continue;
        }

         

        if (tempOpt < 0 || tempOpt > maxval) {
            printf("\n Please enter a valid option between 0 and %d.\n", maxval);
            continue;
        }

        

        option = tempOpt;
       
        break;
      }
    
    int res = 0;

    if (devInfo.devType == DeviceType::ITOF_GMSL || devInfo.devType == DeviceType::ITOF_USB) {
      if (option == 6) {
        option = 9;
      }
    }

    switch (option) {

      case EXIT:
        bSwitch = true;

        for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
          bPreviewSet(1, false, deviceInd);
          previewThreadCtrlFlag[deviceInd] = false;
          streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
          if (bDetach) {
            if (streamThread[deviceInd].joinable()) {
              streamThread[deviceInd].detach();
            }
            if (previewThread[deviceInd].joinable()) {
              previewThread[deviceInd].detach();
            }
          }
#endif
          if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
            std::cout << "Device Close success\n";
          }
        }
        if (DeInitialize() > 0) {
          destroyAllWindows();
        }

        std::quick_exit(0);

      case 1:
      case 2:
        return 1;

      case 3:
        res = depthPlanarization();
        if (res == -1) {
          std::cout << "\n" << "Depth Planarization Failed" << "\n";
          return 0;
        } else if (res == 1 || res == 0) {
          break;
        }
        return 1;
      case 4:
        res = filtersMenu(1);  // value for temporal filter in post_processing structure is 1
        if (res == -1) {
          std::cout << "filtersMenu failed" << "\n";
          return 0;
        } else if (res == 1 || res == 0) {
          break;
        }
        return 1;
      case 5:
        res = depthUndistortion();
        if (res == -1) {
          std::cout << "\n" << "Depth Undistortion Failed" << "\n";
          return 0;
        } else if (res == 1 || res == 0) {
          break;
        }
        return 1;
      case 6:
        res = flyingPixelFilter();
        if (res == -1) {
          std::cout << "\n" << "Flying Pixel Filter Failed" << "\n";
          return 0;
        } else if (res == 1 || res == 0) {
          break;
        }
        return 1;
      case 7:
        res = threeDNoiceFilter();
        if (res == -1) {
          std::cout << "\n" << "3D noie reduction Filter Failed" << "\n";
          return 0;
        } else if (res == 1 || res == 0) {
          break;
        }
        return 1;
      case 8:
        res = fillDepthHoleFilter();
        if (res == -1) {
          std::cout << "\n" << "Depth extrapolation filter Failed" << "\n";
          return 0;
        } else if (res == 1 || res == 0) {
          break;
        }
        return 1;
        
      case 9:
        res = spatialmenu();
        if (res == -1) {
          std::cout << "\n" << "Depth Spatial filter Failed" << "\n";
          return 0;
        } else if (res == 1 || res == 0) {
          break;
        }
        return 1;
      default:
        printf("Invalid option. Please try again.\n");
        break;
    }
  }
  return 1;
}

/**
 * @brief 		Mapping Depth to RGB Depth
 * @return		bool    return true on successfully RGB-D mapping, else retuns fail.
 */
bool rgbdMapping() {
  int option = -1;
  int camId = -1;
  int camAccessId = -1;
  if (exploreOpt == exploreOptions::multipleCam) {
    camId = 1;
    camAccessId = camId;
  } else if (exploreOpt == exploreOptions::singleCam) {
    camId = commonCamId;
    camAccessId = 1;
  }

  switch (camId) {
    case 0:
      break;

    default:
      std::cout << "\n";
      std::cout << '\t' << "0 - Exit" << "\n";
      std::cout << '\t' << "1 - Back" << "\n";
      std::cout << '\t' << "2 - Main Menu" << "\n";
      std::cout << '\t' << "3 - RGB-D Mapping OFF" << "\n";
      std::cout << '\t' << "4 - RGB-D Mapping ON" << "\n";

      int returnVal = 0;
      while ((option < 0) || (option >= 5)) {
        printf("\n Pick a Relevant Option: \t");
        const int ret = scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF && ret != 1) {
        }
      }

      switch (option) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }

          std::quick_exit(0);

        case 1:
        case 2:
          break;

        default:
          bSwitch = true;
          if (exploreOpt == exploreOptions::multipleCam) {
            for (int itr = 0; itr < static_cast<int>(deviceHandleList.size()); itr++) {
              bPreviewSet(1, false, itr);
              streamStarted = false;
              returnVal = SetRGBDMapping(deviceHandleList[itr], option - 3);
              if (returnVal < 0 && returnVal != Result::RGB_DCalibNotFound) {
                pError("SetRGBDMapping");
                continue;
              } else if (returnVal == Result::RGB_DCalibNotFound) {
                std::cout << "\n" << "RGBD Calibration data not found. RGB to Depth Mapping failed" << "\n";
                continue;
              }
              if ((option - 3) != 0) {
                rgbdMappingflag[itr] = true;
              } else {
                rgbdMappingflag[itr] = false;
              }
            }
            startStream();
          } else if (exploreOpt == exploreOptions::singleCam) {
            bPreviewSet(1, false, camAccessId - 1);
            streamStarted = false;

            PrintLog(LOG_HIGH_DEBUG,
                     "\nDepthVistaConsole:ctrlDeviceIndex for SetUnDistortion : " + to_string(camAccessId - 1));
            returnVal = SetRGBDMapping(deviceHandleList[camAccessId - 1], option - 3);
            if (returnVal < 0 && returnVal != Result::RGB_DCalibNotFound) {
              pError("SetRGBDMapping");
              startStream();
              return false;
            } else if (returnVal == Result::RGB_DCalibNotFound) {
              std::cout << "\n" << "RGBD Calibration data not found. RGB to Depth Mapping failed" << "\n";
              startStream();
              return false;
            }
            if ((option - 3) != 0) {
              rgbdMappingflag[camId - 1] = true;
            } else {
              rgbdMappingflag[camId - 1] = false;
            }
            startStream();
          }
          break;
      }
      break;
  }
  return true;
}

int serialNo() {
    int option = -1;
    const int camId = 1;
    int serialNoLength = SERIAL_NUM_LENGTH;
    std::vector<unsigned char> serialNoBuffer(SERIAL_NUM_LENGTH, 0);
    std::string serialNoInput;
    Result res = Result::Failed;
    
    // only get serial number is supported, so remap option 3 to 4.
    if (exploreOpt == exploreOptions::singleCam) {
        std::cout << "\n";
        std::cout << '\t' << "0 - Exit" << "\n";
        std::cout << '\t' << "1 - Back" << "\n";
        std::cout << '\t' << "2 - Main Menu" << "\n";
        std::cout << '\t' << "3 - Get Serial Number" << "\n";

        while ((option < 0) || (option >= 4)) {
            printf("\n Pick a Relevant Option: \t");
            const int ret = scanf("%d", &option);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
        }

        if (option == 3) {
          option = 4;  
        }
    }
    else if (exploreOpt == exploreOptions::multipleCam) {
        std::cout << "\n";
        std::cout << '\t' << "0 - Exit" << "\n";
        std::cout << '\t' << "1 - Back" << "\n";
        std::cout << '\t' << "2 - Main Menu" << "\n";
        std::cout << '\t' << "3 - Get Serial Number" << "\n";

        while ((option < 0) || (option >= 4)) {
            printf("\n Pick a Relevant Option: \t");
            const int ret = scanf("%d", &option);
            while (getchar() != '\n' && getchar() != EOF && ret != 1) {
            }
        }

        if (option == 3) {
            option = 4;
          } 
    }

    switch (option) {
    case EXIT:
        bSwitch = true;

        for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
            }
        }
        if (DeInitialize() > 0) {
            destroyAllWindows();
        }
        std::quick_exit(0);

    case 1:
    case 2:
        return option;
  
    case 3:
        if (gDevicesList[camId - 1].devType == DeviceType::ITOF_USB || gDevicesList[camId - 1].devType == DeviceType::ITOF_GMSL) {
            std::string serialNoInput;
            std::vector<unsigned char> serialNoBuffer(SERIAL_NUM_LENGTH, 0);
            int serialNoLength = SERIAL_NUM_LENGTH;
            // Loop to get the serial number until the length matches
            while (true) {
                // to enter the serial number
                printf("\n Enter the serial number, the size should be less than %d : \t", SERIAL_NUM_LENGTH);
                std::getline(std::cin, serialNoInput);

                // Check if the length matches the expected length
                if (serialNoInput.length() <= static_cast<size_t>(serialNoLength)) {
                    break;  // Exit the loop if the length matches
                }

                std::cout << " The entered serial number does not match the specified length (" << serialNoLength << ") \n";
            }

            // Copy the serial number into the vector
            serialNoBuffer.assign(serialNoInput.begin(), serialNoInput.end());

            Result res = WriteSerialNo(deviceHandleList[camId - 1], serialNoLength, serialNoBuffer.data());
            if (res > 0) {
                std::cout << "Serial Number is successfully set for the camera index " << (camId - 1) << " to : ";
                for (const unsigned char character : serialNoBuffer) {
                    std::cout << character;
                }
                std::cout << "\n";
            }
            else {
                std::cout << "Setting Serial Number failed with the result of : " << res << "\n";
                return -1;
            }
        }
        else {
            std::cout << "Control not supported for this device\n";
        }
        break;

    case 4:
        for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < static_cast<int>(deviceHandleList.size()); ctrlDeviceIndex++) {
            res = ReadSerialNo(deviceHandleList[ctrlDeviceIndex], &serialNoLength, serialNoBuffer.data());
            if (res > 0) {
                if (serialNoLength == 0) {
                  std::cout << "Serial number is not set for the camera index " << ctrlDeviceIndex << "\n";
                  continue;
                }
                // Resize the vector according to the length after the function returns
                serialNoBuffer.resize(serialNoLength);
                std::cout << "Serial Number for the camera index " << ctrlDeviceIndex << " is : ";
                for (const unsigned char character : serialNoBuffer) {
                    std::cout << character;
                }
                std::cout << "\n";
            }
            else {
                std::cout << "Getting serial number failed for the camera index " << ctrlDeviceIndex << " with result " << res << "\n";
            }
        }
        break;
    default:
        printf("Invalid option. Please try again.\n");
        break;
    }

    return 0;
}

int fpsControlMenu() {
  int camId = -1;
  int option = -1;
  int value = -1;

  if (exploreOpt == exploreOptions::multipleCam) {
    cout << "\n" << "Camera Devices Connected to the PC Port : " << "\n" << "\n";
    cout << '\t' << "0 - Back" << "\n";

    for (uint32_t eachDevice = 0; eachDevice < static_cast<int>(deviceHandleList.size()); eachDevice++) {
      streaming_index = selecteddevindex[eachDevice] - 1;
      std::string deviceName = static_cast<char*>(gDevicesList[streaming_index].deviceName);
      if (gDevicesList[streaming_index].devType == DeviceType::ITOF_GMSL) {
        deviceName = "STURDeCAM13_TOF";
      }
      cout << '\t' << eachDevice + 1 << " - " << deviceName << " ("
           << static_cast<char*>(gDevicesList[streaming_index].serialNo) << "  "
           << static_cast<char*>(gDevicesList[streaming_index].devicePath) << ")\n";
    }
    char input[100];
    int tempCamId;

    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);

    while (1) {
      printf("\n Pick a Camera Device to Explore FPS Control : \t");

      // if (fgets(input, sizeof(input), stdin) == NULL) {
      //   continue;
      // }

      // --- ADDED FIX: Smart flush to clean the buffer safely ---
      while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
          std::cin.get();
      }
      // ---------------------------------------------------------

      if (fgets(input, sizeof(input), stdin) == NULL) {
        continue;
      }

      input[strcspn(input, "\n")] = '\0';

      if (strlen(input) == 0) {
        continue;
      }

      char extra;

      if (sscanf(input, "%d %c", &tempCamId, &extra) != 1) {
        printf("\n Invalid input! Numbers only.\n");
        continue;
      }

      if (tempCamId < 0 || tempCamId > static_cast<int>(devices)) {
        printf("\n Please enter a valid camera option.\n");
        continue;
      }

      camId = tempCamId;
      break;
    }
  } else if (exploreOpt == exploreOptions::singleCam) {
    // int c;
    // while ((c = getchar()) != '\n' && c != EOF);
    camId = 1;
  }

  switch (camId) {
    case 0:
      return camId;
    default:
      cout << "\n";
      cout << '\t' << "0 - Exit" << "\n";
      cout << '\t' << "1 - Back" << "\n";
      cout << '\t' << "2 - Main Menu" << "\n";
      cout << '\t' << "3 - Set FPS Mode" << "\n";
      cout << '\t' << "4 - Get FPS Mode" << "\n";

      char input[100];
      int tempOption;

      while (1) {
        printf("\n Pick a Relevant Option:\t");

        // if (fgets(input, sizeof(input), stdin) == NULL) {
        //   continue;
        // }

        // --- ADDED FIX: Smart flush to clean the buffer safely ---
        while (std::cin.peek() == '\n' || std::cin.peek() == '\r' || std::cin.peek() == ' ') {
            std::cin.get();
        }
        // ---------------------------------------------------------

        if (fgets(input, sizeof(input), stdin) == NULL) {
          continue;
        }

        // Remove newline
        input[strcspn(input, "\n")] = '\0';

        // Ignore empty input caused by leftover newline
        if (strlen(input) == 0) {
          continue;
        }

        char extra;

        // Strict integer validation
        if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
          printf("\n Invalid input! Numbers only.\n");
          continue;
        }

        // Range validation
        if (tempOption < 0 || tempOption >= 5) {
          printf("\n Please enter a valid option between 0 and 4.\n");
          continue;
        }

        option = tempOption;
        break;
      }

      switch (option) {
        case EXIT:
          bSwitch = true;
          for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
            if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
              cout << "Device Close success\n";
            }
          }
          if (DeInitialize() > 0) {
            destroyAllWindows();
          }
          std::quick_exit(0);

        case 1:
        case 2:
        return option;

        case 3: {
          // Set FPS Mode flow
          option = -1;
          cout << "\n";
          cout << '\t' << "0 - Exit" << "\n";
          cout << '\t' << "1 - Back" << "\n";
          cout << '\t' << "2 - Main Menu" << "\n";
          cout << '\t' << "3 - 30 FPS" << "\n";
          cout << '\t' << "4 - 50 FPS" << "\n";

          char input[100];
          int tempOption;

          while (1) {
            printf("\n Pick a Relevant Option:\t");

            if (fgets(input, sizeof(input), stdin) == NULL) {
              continue;
            }

            // Remove newline
            input[strcspn(input, "\n")] = '\0';

            // Ignore empty input caused by leftover newline
            if (strlen(input) == 0) {
              continue;
            }

            char extra;

            // Strict integer validation
            if (sscanf(input, "%d %c", &tempOption, &extra) != 1) {
              printf("\n Invalid input! Numbers only.\n");
              continue;
            }

            // Range validation
            if (tempOption < 0 || tempOption >= 5) {
              printf("\n Please enter a valid option between 0 and 4.\n");
              continue;
            }

            option = tempOption;
            break;
          }
          if (option == 0) {
            bSwitch = true;
            for (int deviceInd = 0; deviceInd < static_cast<int>(deviceHandleList.size()); deviceInd++) {
              bPreviewSet(1, false, deviceInd);
              previewThreadCtrlFlag[deviceInd] = false;
              streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
              if (bDetach) {
                if (streamThread[deviceInd].joinable()) {
                  streamThread[deviceInd].detach();
                }
                if (previewThread[deviceInd].joinable()) {
                  previewThread[deviceInd].detach();
                }
              }
#endif
              if (CloseDevice(deviceHandleList[deviceInd]) > 0) {
                cout << "Device Close success\n";
              }
            }
            if (DeInitialize() > 0) {
              destroyAllWindows();
            }
            std::quick_exit(0);
          } else if (option == 1 || option == 2) {
            return option;
          }
          // option == 1 or 2 were earlier returns; actual fps selection stored in 'option'
          // map selection: 3 -> 30 fps, 4 -> 50 fps
          FpsMode setMode = (option == 4) ? TOF_50Fps : TOF_30Fps;

          // Show current FPS before selection
          FpsMode curMode;
          const DeviceHandle curHandle = deviceHandleList[camId - 1];
          if (GetFPSMode(curHandle, &curMode) > 0) {
            //cout << "Current FPS Mode : " << (curMode == TOF_50Fps ? "50 FPS" : "30 FPS") << "\n";
          } else {
            cout << "Unable to read current FPS mode\n";
          }

          // Console-level validation before calling SetFPSMode
          if (gDevicesList[camId - 1].devType == DeviceType::ITOF_USB && setMode == TOF_50Fps) {
            DataMode curDataMode;
            if (GetDataMode(deviceHandleList[camId - 1], &curDataMode) > 0) {
              if (curDataMode == DataMode::Depth_IR_Conf_HD_Mode) {
                cout << "50 FPS is not supported for HD mode in ITOF_USB variant\n";
                break;
              }
            } else {
              cout << "Unable to determine current DataMode\n";
              break;
            }
          }

          if (SetFPSMode(deviceHandleList[camId - 1], setMode) > 0) {
            cout << "FPS mode is successfully set in camera " << commonCamId - 1 << " with FPS " << (setMode == TOF_50Fps ? "50" : "30") << "\n";
          } else {
            value = -1;
            return false;
          }

          if (exploreOpt == exploreOptions::singleCam) {
            value = -1;
            break;
          }
          value = -1;
          break;
        }
        case 4: {
          // Get FPS Mode
          FpsMode getMode;
          if (GetFPSMode(deviceHandleList[camId - 1], &getMode) > 0) {
            cout << "Current FPS Mode of the camera " << commonCamId - 1 << " : "
                 << (getMode == TOF_50Fps ? "50 FPS" : "30 FPS") << "\n";
          } else {
            value = -1;
            return false;
          }
          if (exploreOpt == exploreOptions::singleCam) {
            value = -1;
            break;
          }
          value = -1;
          break;
        }
        default:
          printf("Invalid option. Please try again.\n");
          break;
      }
      break;
  }
  return 0;
}

/**
 * @brief 		Explore the controls of Device
 * @return		bool    return true on success, else retuns fail.
 */
bool exploreCam() {
  const int uniqueIDLength = 32;
  std::vector<char> uniqueID(uniqueIDLength);
  FWVersion fwVer = {0};
  const int length_depth = 0;
  bool closeDeviceRes = false;
  int maxVal = 0;
  int tofCalibFileSize = 0;
  Result result = Result::Others;
  std::string FileName;
  std::string line;
  std::string CalibData;
  // FILE* fptr = nullptr;
  int read_length_depth = 0;
  int16_t value = -1;
  int res = -1;
  uint16_t getvalue = 0;
  int gValue = 0;
  while (true) {
    if (g_needRelist.exchange(false)) {
#ifdef _WIN32
      FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
#elif __linux__
      std::cout << "\nDevice list changed, refreshing...\n";
      tcflush(STDIN_FILENO, TCIFLUSH);
#endif
      if (!listDevices()) {
        cout << "\n" << "List Devices Information Failed" << "\n";
        return false;
      }
      continue;
    }

    int choice = -1;
    DeviceInfo devInfo;
    if (exploreOpt == exploreOptions::singleCam) {
      devInfo = gDevicesList[commonCamId - 1];
    } else {
      if (successCam.size() > 0) {
        devInfo = successCam[0];
      } else {
        return false;
      }
    }

    if (devInfo.devType == DeviceType::ITOF_USB || devInfo.devType == DeviceType::ITOF_GMSL) {
      cout << "\n" << '\t' << "0    - Exit" << "\n";
      cout << '\t' << "1    - Back" << "\n";
      cout << '\t' << "2    - Streaming Mode" << "\n";
      cout << '\t' << "3    - Capture Frames" << "\n";
      cout << '\t' << "4    - Depth Denoise" << "\n";
      cout << '\t' << "5    - Confidence Threshold" << "\n";
      cout << '\t' << "6    - Integration Time" << "\n";
      cout << '\t' << "7    - TOF IR Gain" << "\n";
      cout << '\t' << "8    - Flying Pixel Filter" << "\n";
      cout << '\t' << "9    - Get Temperature data" << "\n";
      cout << '\t' << "10   - Post Processing" << "\n";
      cout << '\t' << "11   - Confidence Mode" << "\n";   
      cout << '\t' << "12   - AVG Depth" << "\n";
      cout << '\t' << "13   - Unique ID" << "\n";
      cout << '\t' << "14   - Read Firmware Version" << "\n";
      cout << '\t' << "15   - Serial Number" << "\n";
      cout << '\t' << "16   - Laser Safety" << "\n";
      cout << '\t' << "17   - FPS Control" << "\n";
      maxVal = 18;
    } else if (devInfo.devType == DeviceType::CMOS_MIPI || devInfo.devType == DeviceType::CMOS_USB_IRD) {
      cout << "\n" << '\t' << "0   - Exit" << "\n";
      cout << '\t' << "1   - Back" << "\n";
      cout << '\t' << "2   - Streaming Mode" << "\n";
      cout << '\t' << "3   - TOF Controls" << "\n";
      cout << '\t' << "4   - Capture Frames" << "\n";
      cout << '\t' << "5   - Unique ID" << "\n";
      cout << '\t' << "6   - Read Firmware Version" << "\n";
      cout << '\t' << "7   - Get Depth Values" << "\n";
      maxVal = 8;
    } else {
      cout << "\n" << '\t' << "0   - Exit" << "\n";
      cout << '\t' << "1   - Back" << "\n";
      cout << '\t' << "2   - Streaming Mode" << "\n";
      cout << '\t' << "3   - TOF Controls" << "\n";
      cout << '\t' << "4   - UVC Controls" << "\n";
      cout << '\t' << "5   - RGB-D Mapping" << "\n";
      cout << '\t' << "6   - Post Processing" << "\n";
      cout << '\t' << "7   - Capture Frames" << "\n";
      cout << '\t' << "8   - Unique ID" << "\n";
      cout << '\t' << "9   - Read Firmware Version" << "\n";
      cout << '\t' << "10   - Get Depth value" << "\n";
      maxVal = 11;
    }

    {
      bool doRelist = false;
      while (!doRelist) {

        printf("\n Pick a Relevant Choice of Camera Properties : \t");
        fflush(stdout);
        choice = -1;

#ifdef _WIN32
        {
          char inputBuf[32] = {};
          int inputLen = 0;
          bool lineReady = false;
          while (!lineReady && !doRelist) {
            if (g_needRelist) { doRelist = true; break; }
            if (!_kbhit()) {
              std::this_thread::sleep_for(std::chrono::milliseconds(20));
              continue;
            }
            const int c = _getch();
            if (c == '\r') {
              putchar('\n'); fflush(stdout);
              if (inputLen > 0) choice = atoi(inputBuf);
              lineReady = true;
            } else if ((c == 8 || c == 127) && inputLen > 0) {
              inputLen--;
              printf("\b \b"); fflush(stdout);
            } else if (c >= '0' && c <= '9' && inputLen < 30) {
              inputBuf[inputLen++] = static_cast<char>(c);
              putchar(c); fflush(stdout);
            }
          }
        }
#elif __linux__
        {
          struct termios oldt, newt;
          tcgetattr(STDIN_FILENO, &oldt);
          newt = oldt;
          newt.c_lflag &= ~(static_cast<tcflag_t>(ICANON) | static_cast<tcflag_t>(ECHO));
          tcsetattr(STDIN_FILENO, TCSANOW, &newt);

          char inputBuf[32] = {};
          int inputLen = 0;
          bool lineReady = false;
          while (!lineReady && !doRelist) {
            if (g_needRelist) { doRelist = true; break; }
            struct timeval tv = {0, 20000};
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) <= 0) continue;
            if (c == '\n' || c == '\r') {
              putchar('\n'); fflush(stdout);
              if (inputLen > 0) choice = atoi(inputBuf);
              lineReady = true;
            } else if ((c == 8 || c == 127) && inputLen > 0) {
              inputLen--;
              printf("\b \b"); fflush(stdout);
            } else if (c >= '0' && c <= '9' && inputLen < 30) {
              inputBuf[inputLen++] = c;
              putchar(c); fflush(stdout);
            }
          }
          tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
          tcflush(STDIN_FILENO, TCIFLUSH);
        }
#endif

        if (!doRelist) {
          if ((choice >= 0) && (choice < maxVal)) break;
          printf("\n Please enter a valid choice between 0 and %d.\n", maxVal - 1);
        }
      }

      if (doRelist || g_needRelist.exchange(false)) {
        g_needRelist = false;
#ifdef _WIN32
        FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
#elif __linux__
        tcflush(STDIN_FILENO, TCIFLUSH);
#endif
        if (!listDevices()) {
          cout << "\n" << "List Devices Information Failed" << "\n";
          return false;
        }
        continue;
      }
    }

    if (devInfo.devType == DeviceType::ITOF_USB || devInfo.devType == DeviceType::ITOF_GMSL) {
      if (choice == 3) {
        choice = 7;
      } else if (choice == 4) {
        choice = 12;
      } else if (choice == 5) {
        choice = 13;
      } else if (choice == 6) {
        choice = 11;
      } else if (choice == 7) {
        choice = 14;
      } else if (choice == 8) {
        choice = 17;
      } else if (choice == 9) {
        choice = 15;
      } else if (choice == 10) {
        choice = 6;
      } else if (choice == 11) {
        choice = 18;
      } else if (choice == 12) {
        choice = 10;
      } else if (choice == 13) {
        choice = 8;
      } else if (choice == 14) {
        choice = 9;
      } else if (choice == 15) {
        choice = 19;
      } else if (choice == 16) {
        choice = 20;
      } else if (choice == 17) {
        choice = 21;
      }

      if (devInfo.devType == DeviceType::CMOS_MIPI || devInfo.devType == DeviceType::CMOS_USB_IRD) {
        if (choice == 4) {
          choice = 7;
        } else if (choice == 5) {
          choice = 8;
        } else if (choice == 6) {
          choice = 9;
        } else if (choice == 7) {
          choice = 10;
        }
      }
      // else if (gDevicesList[camId - 1].devType == DeviceType::ITOF_GMSL) {
      //   if (choice == 2) {
      //     choice = 7;
      //   }
     }

      switch (choice) {
        case EXIT:
          bSwitch = true;

          for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++) {
            bPreviewSet(1, false, deviceInd);
            previewThreadCtrlFlag[deviceInd] = false;
            streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
            if (bDetach) {
              if (streamThread[deviceInd].joinable()) {
                streamThread[deviceInd].detach();
              }
              if (previewThread[deviceInd].joinable()) {
                previewThread[deviceInd].detach();
              }
            }
#endif
          }
          for (const auto& deviceHandle : deviceHandleList) {
            const Result closeDevRes = CloseDevice(deviceHandle);
            if (closeDevRes > 0) {
              closeDeviceRes = true;
            } else {
              std::cout << "Close Device failed with ret : " << closeDevRes << "\n";
            }
          }
          if (closeDeviceRes) {

            if (DeInitialize() > 0) {
              destroyAllWindows();
            }
          }
          std::quick_exit(0);

        case 1:
          if (!listDevices()) {
            cout << "\n" << "List Devices Information failed" << "\n";
          }
          cout << "\n" << "Connected Devices were Listed" << "\n";
          break;

        case 2:
          if (!selectStreamingMode()) {
            cout << "\n" << "Data Mode Selection Failed" << "\n";
          }
          cout << "\n" << "Data Mode Selected" << "\n";
          break;

        case 3:
          if (tofControlSettings() == 0) {
            cout << "\n" << "TOF Controls selection Failed" << "\n";
          }
          cout << "\n" << "TOF Controls Selected" << "\n";
          break;

        case 4:
          if (!uvcControlMenu()) {
            cout << "\n" << "UVC controls Selection Failed" << "\n";
          }
          cout << "\n" << "UVC controls Selected" << "\n";
          break;

        case 5:
          if (!rgbdMapping()) {
            cout << "\n" << "RGB-D Mapping Failed" << "\n";
          }
          break;

        case 6:
          if (postProcessing() == 0) {
            cout << "\n" << "PostProcessing Failed" << "\n";
          }
          break;

        case 7:
          if (bPreviewSet(3, true, 0)) {
            memset(static_cast<void*>(depthFrameFileNameBuf), 0, MAX_FILENAME);
            memset(static_cast<void*>(RGBFrameFileNameBuf), 0, MAX_FILENAME);
            memset(static_cast<void*>(IRFrameFileNameBuf), 0, MAX_FILENAME);
            memset(static_cast<void*>(depthRawFrameFileNameBuf), 0, MAX_FILENAME);
            memset(static_cast<void*>(IRRawFrameFileNameBuf), 0, MAX_FILENAME);
            memset(static_cast<void*>(PLY3DFileNameBuf), 0, MAX_FILENAME);
            memset(static_cast<void*>(depthConfFrameFileNameBuf), 0, MAX_FILENAME);

            saveFrames[0] = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (exploreOpt != 0) {
              for (int itr = 1; itr < static_cast<int>(deviceHandleList.size()); itr++) {
                saveFrames[itr] = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
              }
            }
          } else {
            cout << "\n" << "Image Capture Failed, Please start preview before capturing frames" << "\n";
          }

          cout << "\n" << "Frame Capture ends" << "\n";

          break;

        case 8:
          for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < static_cast<int>(deviceHandleList.size());
               ctrlDeviceIndex++) {
            if (GetUniqueID(deviceHandleList[ctrlDeviceIndex], uniqueID.data()) == Result::Ok) {
              cout << "\n" << "Unique ID of the Camera " << ctrlDeviceIndex << " is " << uniqueID.data() << "\n";
            } else {
              cout << "Unique ID read failed in camera " << ctrlDeviceIndex << "\n";
            }
          }
          break;

        case 9:
          for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < static_cast<int>(deviceHandleList.size());
               ctrlDeviceIndex++) {
            if (ReadFirmwareVersion(deviceHandleList[ctrlDeviceIndex], &fwVer) == Result::Ok) {
              if (gDevicesList[ctrlDeviceIndex].devType == DeviceType::ITOF_GMSL) {
                cout << "\n"
                     << "Firmware Version in camera " << ctrlDeviceIndex << "  : " << fwVer.mcuFirmwareVersion[4]
                     << fwVer.mcuFirmwareVersion[5] << fwVer.mcuFirmwareVersion[6] << "\n";
              } else {
                cout << "\n"
                     << "Firmware Version in camera " << ctrlDeviceIndex << "  : " << fwVer.cx3FirmwareVerison << "\n";
              }
            } else {
              cout << "Firmware Version read failed in camera " << ctrlDeviceIndex << "\n";
            }
          }
          break;

        case 10:
          if (Depthstreamstarted) {
            if (!selectGetDepthValue()) {
              cout << "\n" << "Get Depth value Failed" << "\n";
            }
          } else {
            cout << "\n" << " Start Depth stream to get depth value" << "\n";
          }
          break;
        case 11:
          intgtimemenu();
          break;

        case 12:
          depthnoisemenu();
          break;

        case 13:
          confidencemenu();
          break;

        case 14:
          res = irGainMenu();
          if (res == -1) {
            cout << "\n" << "SetTOFIRGain Failed" << "\n";
          }
          break;
        case 15:
          res = getTemperature();
          if (res == -1) {
            cout << "\n" << "getting Temperature Data Failed" << "\n";
          }
          break;

        case 16:
          spatialmenu();
          break;

        case 17:
          res = flyingPixelFilter();
          if (res == -1) {
            cout << "\n" << "Flying Pixel Filter Failed" << "\n";
          }
          break;
        case 18:
          res = confmodemenu();
          if (res == -1) {
            cout << "\n" << "Confidence Mode Filter Failed" << "\n";
          }
          break;
        case 19:
          res = serialNo();
          if (res == -1) {
            cout << "\n" << "Serial Number Failed" << "\n";
          }
          break;
        case 20:
          res = laserSafetyMenu();
          if (res == -1) {
            cout << "\n" << "Laser Safety Failed" << "\n";
          }
          break;
          case 21:
            res = fpsControlMenu();
            if (res == -1) {
              cout << "\n" << "FPS Control Failed" << "\n";
            }
            break;
        default:
          printf("Invalid option. Please try again.\n");
          break;
      }
    }
    return true;
  
}

void registerNotificationCb(int notificationId, DeviceHandle handle) {
  bSwitch = true;
  streamStarted = false;
  for (int itr = 0; itr < static_cast<int>(deviceHandleList.size()); itr++) {
    bPreviewSet(1, false, itr);
    streamThreadCtrlFlag[itr] = false;
    previewThreadCtrlFlag[itr] = false;
  }

    deviceHandleList.erase(
    std::remove_if(deviceHandleList.begin(), deviceHandleList.end(),
                    [&handle](const DeviceHandle& entry) { return strcmp(entry.serialNo, handle.serialNo) == 0; }),
    deviceHandleList.end());

  if (notificationId == 1 || notificationId == 0) {
    CloseDevice(handle);
  }

  auto itvar = deviceHandleList.end();
  for (auto it = deviceHandleList.begin(); it != deviceHandleList.end(); ++it) {
    if (std::memcmp(it->serialNo, handle.serialNo, sizeof(handle.serialNo)) == 0) {
      itvar = it;
      break;
    }
  }

  if (itvar != deviceHandleList.end()) {
    const size_t index = std::distance(deviceHandleList.begin(), itvar);
    deviceHandleList.erase(itvar);
    if (index < rgbdMappingflag.size()) {
      rgbdMappingflag.erase(rgbdMappingflag.begin() + static_cast<std::ptrdiff_t>(index));
    }
  }
  destroyAllWindows();

  for (int itr = 0; itr < static_cast<int>(deviceHandleList.size()); itr++) {
    tempDeviceIndex[itr] = itr;
    streamThreadCtrlFlag[itr] = true;
    previewThreadCtrlFlag[itr] = true;

#ifdef _WIN32
    if (streamThread[itr].joinable()) {
      streamThread[itr].detach();
    }
    if (previewThread[itr].joinable()) {
      previewThread[itr].detach();
    }
    PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:Before creating thread\n");

    previewThread[itr] = thread(&preview, itr);
    streamThread[itr] = thread(&stream, itr);

    PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:After creating thread\n");
#elif __linux__
    pthread_create(&streamThread[itr], nullptr, stream, &tempDeviceIndex[itr]);

    pthread_create(&previewThread[itr], nullptr, preview, &tempDeviceIndex[itr]);
#endif
  }

  maxDeviceIndex--;
 //deviceHandleList.clear();

  if (!deviceHandleList.empty()) {
    startStream();
  } else {
    g_needRelist = true;
  }
}

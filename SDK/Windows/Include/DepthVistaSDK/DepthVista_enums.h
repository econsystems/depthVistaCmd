/*
Copyright © 2003 - 2024, e-con Systems India Private Limited. All rights reserved.
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

#ifndef DEPTHVISTA_ENUMS_H
#define DEPTHVISTA_ENUMS_H

/**
 * @brief The data modes that determine the frame output from the device.
 */
typedef enum {
  // Invalid_Mode= -1,
  Depth_IR_Mode = 0,
  Depth_IR_Conf_Mode = 1,
  Raw_Mode = 1,
  Depth_Mode = 2,
  IR_Mode = 3,
  Depth_IR_RGB_VGA_Mode = 4,
  Depth_IR_RGB_HD_Mode = 5,
  RGB_VGA_Mode = 6,
  RGB_HD_Mode = 7,
  RGB_Full_HD_Mode = 8,
  RGB_1200p_Mode = 9,
  Depth_IR_C1_C2_Mode = 10,
  Depth_RGB_HD_Mode = 11,
  Depth_IR_C1_RGB_HD_Mode = 12,
  Depth_IR_C1_C2_RGB_HD_Mode = 13,
  RGB_960x540_Mode_100fps = 14,
  RGB_Full_HD_Mode_60fps = 15,
  RGB_4K_Mode_20fps = 16,
  RGB_4K_Mode_30fps = 17,
  IR_RGB_HD_Mode = 18,
  RGB_4K_Mode_15fps = 19,
  Depth_IR_Conf_HD_Mode = 20,
  Depth_Conf_175Mhz = 21,
  Depth_Conf_200Mhz = 22,
} DataMode;

/**
 * @brief LogLevel determine the level of the log that is to be stored.
 */
typedef enum { Low = 0, Critical = 1, High = 2, Disable = 3 } LogLevel;

/**
 * @brief Logging contains the options for logging debug prints.
 */
typedef enum {
  Console = 1,
  LogFile = 2,
  BothConsoleAndLogFile = 3,
} Logging;

/**
 * @brief Camera modes supported by DepthVista.
 */
typedef enum {
  DualCameraMode = 0,
  RGBCameraMode = 1,
  TOFCameraMode = 2,
} CameraMode;

/**
 * @brief Depth range setting.\n
 *        These set estimated ranges. Detection distances may be greater than what is listed for the given setting. \n
 *        Precision and minimum distance for depth detection varies with longer ranges.
 */
typedef enum {
  NearRange = 0,  //!< Range0.2- 1.2m
  FarRange = 1,   //!< Range1 - 6.5m
  WDRRange = 2,   //!< Range0.2 - 4m
} DepthRange;

/**
 * @brief Specifies the type of image frame.
 */
typedef enum {
  IRPreviewFrame,  //!< IR frame with 12 bits per pixel multiplied with IR Gain.
  DepthColorMap,   //!< Colormap applied for raw depth frame with post processing
  RGBFrame,        //!< RGB Frame with 16bit per pixel.
  DepthRawFrame,   //!< Depth raw frame with 16 bits per pixel.
  DepthRawOriginal,
  ConfidenceFrame,
  IRA0RawFrame,       //!< Raw A0 IR frame with 16 bits per pixel.
  IRA1RawFrame,       //!< Raw A1 IR frame with 16 bits per pixel.
  IRA2RawFrame,       //!< Raw A2 IR frame with 16 bits per pixel.
  IRA0RawFrame_save,  //!< Raw A0 IR frame with 16 bits per pixel.
  IRA1RawFrame_save,  //!< Raw A1 IR frame with 16 bits per pixel.
  IRA2RawFrame_save,  //!< Raw A2 IR frame with 16 bits per pixel.
} FrameType;

/**
 * @brief FPS modes for TOF cameras
 */
typedef enum {
  TOF_30Fps = 0,
  TOF_50Fps = 1,
} FpsMode;

/**
 * @brief Return status codes for all APIs.\n
 * 		  Ok = 1 means the API successfully completed its operation.\n
 * 		  All other codes indicate a device, parameter, or API usage error.
 */
typedef enum {
  Ok = 1,                    //!< The function completed successfully.
  NotInitialized = -1,       //!< The APIs are not initialized
  NotDeInitialized = -2,     //!< The APIs are not deinitialized
  InvalidFrameType = -3,     //!< The input frame type is invalid.
  NoStreaming = -4,          //!< The camera is not streaming.
  AlreadyStreaming = -5,     //!< The camera is already streaming.
  InvalidNode = -6,          //!< The device Node is invalid.
  CameraNotOpened = -7,      //!< The camera has not been opened.
  InvalidDeviceIndex = -8,   //!< The input device index is invalid.
  NoDeviceConnected = -9,    //!< There is no depth camera connected or the camera has not been connected correctly.
  NoPropertyValueGet = -10,  //!< Cannot get the value for the specified property.
  NoPropertyValueSet = -11,  //!< Cannot set the value for the specified property.
  SysCallFail = -12,         //!< System call function failed.
  InvalidValue = -13,        //!< One or more of the parameter values provided are invalid.
  HidWriteFail = -14,        //!< Cannot write the buffer to the hid handle.
  HidReadFail = -15,         //!< Cannot read the buffer from the hid handle.
  UVCOpenFail = -16,
  TimeoutError = -17,            //!< Read on file descriptor timedout.
  InvalidBuffer = -18,           //!< Frame from the camera is invalid.
  RGB_DCalibNotFound = -19,      //!< RGB-D Calibration Data not found in the device
  FrameSeparationFailed = -20,   //!< Frame separation logic failed
  RegisterCallBackFailed = -21,  //!< Registering Call back to SDK failed
  CameraAlreadyOpen = -22,       //!< Device is already open>
  InvalidDataMode = -23,         //<! Current Datamode doesn't support the request
  Failed = -24,
  Others = -255,  //!< An unknown error occurred.
  ProcessPtrOpenFailed = -25,
  ProcessPtrCloseFailed = -26,
  NoGraphicCardFound = -27,
  InvalidDepthMode = -28,
  CameraNotSupported = -29,
  SizeQueryFailed = -30,
  FrameQueryFailed = -31,
  FilterNotApplied = -32,
  TOFCalibrationNotFound = -33,
  DeviceNotCapable = -34,
} Result;

/**
 * @brief Value to specify if Average of Depth and IR is calculated at the center or along the mouse pointer.
 */
typedef enum {
  Center = 0,     //!< To get the average depth and ir value at center region of the image.
  CustomPtr = 1,  //!< To get the average depth and ir value at region around the mouse pointer.
  Exit = -1,
} AvgRegion;

/**
 * @brief List of all UVC control in DepthVista.
 */
typedef enum {
  TOF_UVC_CID_BRIGHTNESS,
  TOF_UVC_CID_CONTRAST,
  TOF_UVC_CID_SATURATION,
  TOF_UVC_CID_WB_AUTO,
  TOF_UVC_CID_GAMMA,
  TOF_UVC_CID_GAIN,
  TOF_UVC_CID_PWR_LINE_FREQ,
  TOF_UVC_CID_WB_TEMP,
  TOF_UVC_CID_SHARPNESS,
  TOF_UVC_CID_EXPOSURE_AUTO,
  TOF_UVC_CID_EXPSOURE_ABS,
} UVCPropID;

/**
 * @brief Flag values to specify board’s Calibration status.
 */
typedef enum { Not_Calibrated = -1, Only_VGA = 0, Only_HD = 1, Both_VGA_HD = 2 } CalibValidFlag;

/**
 * @brief List of all Pixel format supported by DepthVista.
 */
typedef enum {
  PIX_FMT_UYVY,
  PIX_FMT_Y16,
  PIX_FMT_RGB,
  PIX_FMT_Y12,
} PixFormat;

typedef enum { USB_IRD = 1, GMSL_USB = 2, CMOS_USB_IRD = 3, ITOF_USB = 4, ITOF_GMSL = 5, RGBD_IRD = 6 ,CMOS_MIPI = 7} DeviceType;

#endif /* DEPTHVISTA_ENUMS_H */

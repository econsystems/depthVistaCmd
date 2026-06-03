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

#ifndef DEPTHVISTA_TYPES_H
#define DEPTHVISTA_TYPES_H

#include <functional>
#include <string>

#if __linux__
	#include <linux/types.h>
#endif

#include "DepthVista_enums.h"

typedef struct {
  char depthNodePath[500];
  char confNodePath[500];
  char irNodePath[500];
  int busID;
  int nodeAdd;
  int deviceIndex;
  uint16_t nodeValidation;
} GMSLDeviceInfo;

/**
 * @brief Specifies the information about the device.
 */
typedef struct {
  char deviceName[50];
  char vid[5];
  char pid[5];
  char devicePath[500];
  char serialNo[50];
  DeviceType devType;
  GMSLDeviceInfo gmslDev;
} DeviceInfo;

/**
 * @brief Contains Intrinsic Calibration parameters.
 */
typedef struct {
  double fx, fy, cx, cy;
  double k1, k2, p1, p2, k3;
} IntrinsicCalibParams;

/**
 * @brief Contains Extrinsic Calibration parameters.
 */
typedef struct {
  double rotationalVec[3][3];
  double translationalVec[3];
} ExtrinsicCalibParams;

/**
 * @brief Contains Intrinsic and Extrinsic Calibration parameters.
 */
typedef struct {
  IntrinsicCalibParams depthCamVGAIntrinsic, rgbCamVGAIntrinsic, depthCamHDIntrinsic, rgbCamHDIntrinsic;
  ExtrinsicCalibParams VGAextrinsic, HDextrinsic;
} CalibrationParams;

/**
 * @brief Holds serialNo of the camera which is used to differentiate multiple device.
 */
typedef struct {
  char serialNo[50];
} DeviceHandle;

/**
 * @brief Contains the details of the UVC properties queried form the device.
 */
typedef struct {
  int id;
  int min;
  int max;
  int cur;
  int step;
  int default_val;
} UVCProp;

/**
 * @brief Specifies the X and Y coordinates of the DepthPtr.
 */
typedef struct {
  int X;
  int Y;
} DepthPtr;

/**
 * @brief Contains the details of the frame obtained from camera.
 */
typedef struct {
  unsigned char* frame_data;
  uint16_t width;
  uint16_t height;
  uint8_t pixel_format;
  uint32_t size;
  uint64_t time_stamp;
  uint64_t frame_id;
} ToFFrame;

/**
 * @brief Contains TofFrame of all frame types.
 */
typedef struct {
  ToFFrame rgb, ir, raw_ir, raw_depth, raw_depth_original, depth_colormap, confidence_frame, confidence_frame2,
      IRA0RawFrame, IRA1RawFrame, IRA2RawFrame, IRA0RawFrame_save, IRA1RawFrame_save, IRA2RawFrame_save;
} Frames;

/**
 * @brief Contains the configuration details for IMU.
 */
typedef struct {
  uint8_t IMU_MODE;
  uint8_t ACC_AXIS_CONFIG;
  uint8_t ACC_SENSITIVITY_CONFIG;
  uint8_t GYRO_AXIS_CONFIG;
  uint8_t GYRO_SENSITIVITY_CONFIG;
  uint8_t IMU_ODR_CONFIG;
} IMUCONFIG_TypeDef;

/**
 * @brief Contains the configuration details of IMU for Control IMU Capture.
 */
typedef struct {
  uint8_t imu_update_mode;
  uint16_t imu_num_of_values;
} IMUDATAINPUT_TypeDef;

/**
 * @brief Contains the output data from IMU.
 */
typedef struct {
  uint16_t imu_value_id;
  double accX;
  double accY;
  double accZ;
  double gyroX;
  double gyroY;
  double gyroZ;
} IMUDATAOUTPUT_TypeDef;

/**
*@brief Contains the outdata from IMU for iTof GMSL
*/

typedef struct _imu_data {
#if __linux__
			__u16 index;
			__u16 size;
#endif
			int16_t accX;
			int16_t accY;
			int16_t accZ;
			int16_t gyroX;
			int16_t gyroY;
			int16_t gyroZ;
			uint16_t imu_temp;
			int8_t acc_flag;
			int8_t gyro_flag;
			int8_t temp_flag;
			int8_t config_flag;
			int8_t imu_mode;
			int8_t odr_config;
			int8_t acc_sensitivity;
			int8_t gyro_sensitivity;
}IMU_DATA_GMSL;

typedef struct {
  char mcuFirmwareVersion[32];
  char fpgaFirmwareVersion[32];
  char driverFirmwareVersion[32];
  char cx3FirmwareVerison[32];
} FWVersion;

#endif /* DEPTHVISTA_TYPES_H */

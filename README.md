
# DepthVistaCmd : Command Line application with DepthVistaSDK (CPP & Python)

Sample command line application, opens the device, gets stream, and enables the user to control some basic TOF Camera Controls. 

## Prerequisite

* CMake (version 3.5 and above)
* OpenCV library (version 4.5.4 and above)


## Supported Platforms:

* Windows 10 , 11
* Ubuntu 20.04 (64 bit ), 22.04 (64bit)
* ARM - Ubuntu 22.04

## Supported Camera controls for See3CAM_TOF_25CUG:

* Streaming Mode
* TOF Controls
    * Depth Range
    * TOF Coring
    * IR Gain
    * IMU Embedded data
* Post Processing
    * Planarization
    * Temporal Filter
    * Undistort depth
    * Flying Pixel filter
    * 3D Noice Reduction filter
    * Depth Extrapolation filter
    * Depth Spatial Filter
* Capture Frames
* Unique ID
* Read Firmware Version
* Get Depth value

## Supported Camera controls for See3CAM_TOF_CU13 and STURDeCAM13_TOF:

* Streaming Mode
* Capture Frames
* Depth Denoise
* Confidence Threshold
* Integration Time
* TOF IR Gain
* Flying Pixel Filter
* Get Temperature data
* Post Processing
    * Planarization
    * Temporal Filter
    * Undistort depth
    * Depth Spatial Filter
* Confidence Mode
* AVG Depth
* Unique ID
* Read Firmware Version
* Serial Numbe
* Laser Safety
* FPS Control

## How to Use

- Build the package in Windows using [this Build Manual](https://github.com/econsystems/depthVistaCmd/tree/master/windows/Documents)

- Build the package in Ubuntu using [this Build Manual](https://github.com/econsystems/depthVistaCmd/tree/master/linux/Documents)

- Run the DepthVistaConsole application using [this User Manual](https://github.com/econsystems/depthVistaCmd/tree/master/windows/Documents)

- Know more about RGB-D mapping using [this User Manual](https://github.com/econsystems/depthVistaCmd/tree/master/windows/Documents)

## Releases

* Latest releases can be downloaded from [this link](https://github.com/econsystems/depthVistaCmd/releases)

## Release

* DepthVistaCmd v1.0.0		-	05-Sep-2022
* DepthVistaCmd v1.0.1		-	02-Nov-2022
* DepthVistaCmd v1.0.3		-	15-Dec-2023
* DepthVistaCmd v1.0.11		-	22-Jan-2025
* DepthVistaCmd v1.0.0.15		-	03-Jun-2026

## What's new

* SDK and console application bug fixes

## Refer other sample source

- Refer DepthVista GUI application sample source using [this page](https://github.com/econsystems/DepthVista)

## Support

If you need assistance, visit at https://www.e-consystems.com/create-ticket.asp or contact us at techsupport@e-consystems.com

#include"DepthVistaConsoleApp.h"


/**
* @brief 				Setting and Checking preview state
* @param[in]            tid	Id based on which the function acts as setting or checking preview status
* @param[in]            bPrev   preview status
* @return				bool	the state of preview
*/
bool bPreviewSet(int tid, bool bPrev, int deviceIndex)
{
    std::lock_guard<std::mutex> guard(guardMutex);

    if (tid == 1)
    {
        bPreview[deviceIndex] = bPrev;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return bPreview[deviceIndex];
}

/**
* @brief 		Starting preview with current dataMode and depthRange
* @return		void
*/
void startStream()
{
	for (int deviceCnt = 0; deviceCnt < deviceHandleList.size(); deviceCnt++)
	{
		if(Depthcolormap[deviceCnt].empty())
		{
			Depthcolormap[deviceCnt] = cv::Mat(PRE_VGA_HEIGHT, PRE_VGA_WIDTH, CV_8UC3);
		}
		if(IRFrame[deviceCnt].empty())
		{
			IRFrame[deviceCnt] = cv::Mat(PRE_VGA_HEIGHT, PRE_VGA_WIDTH, CV_16UC1);
		}

		if(DepthImg_Y16[deviceCnt].empty())
		{
			DepthImg_Y16[deviceCnt] = cv::Mat(PRE_VGA_HEIGHT, PRE_VGA_WIDTH, CV_16UC1);
		}

			if (dataMode == RGB_HD_Mode || dataMode == Depth_IR_RGB_HD_Mode)
			{
				UYVYFrame[deviceCnt].release();
				UYVYFrame[deviceCnt] = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_8UC2);
				if (dataMode == Depth_IR_RGB_HD_Mode)
				{
					if (rgbdMappingflag[deviceCnt])
					{
						Depthcolormap[deviceCnt].release();
						Depthcolormap[deviceCnt] = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_8UC3);
						DepthImg_Y16[deviceCnt].release();
						DepthImg_Y16[deviceCnt] = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_16UC1);
					}
					else
					{
						Depthcolormap[deviceCnt].release();
						Depthcolormap[deviceCnt] = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_8UC3);
						DepthImg_Y16[deviceCnt].release();
						DepthImg_Y16[deviceCnt] = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_16UC1);
					}
				}
			}
			else if (dataMode == RGB_Full_HD_Mode)
			{
				UYVYFrame[deviceCnt].release();
				UYVYFrame[deviceCnt] = cv::Mat(PRE_RGB_FULL_HD_HEIGHT, PRE_RGB_FULL_HD_WIDTH, CV_8UC2);
			}
			else if (dataMode == RGB_1200p_Mode)
			{
				UYVYFrame[deviceCnt].release();
				UYVYFrame[deviceCnt] = cv::Mat(PRE_RGB_1200p_HEIGHT, PRE_RGB_1200p_WIDTH, CV_8UC2);
			}
			else if (dataMode == Depth_IR_RGB_VGA_Mode || dataMode == RGB_VGA_Mode)
			{
				UYVYFrame[deviceCnt].release();
				UYVYFrame[deviceCnt] = cv::Mat(PRE_RGB_VGA_HEIGHT, 640, CV_8UC2);
				Depthcolormap[deviceCnt].release();
				Depthcolormap[deviceCnt] = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_8UC3);
				DepthImg_Y16[deviceCnt].release();
				DepthImg_Y16[deviceCnt] = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_16UC1);
			}
			if (depthRange == DepthRange::NearRange)
			{
				Depth_min = NEAR_MODE_MIN;
				Depth_max = NEAR_MODE_MAX;
			}
			else
			{
				Depth_min = FAR_MODE_MIN;
				Depth_max = FAR_MODE_MAX;
			}
			depth_offset = (Depth_max - Depth_min) * 0.20;
			UpdateColorMap(deviceHandleList[deviceCnt], Depth_min, Depth_max + depth_offset, 4);
			bSwitch = false;
			bPreviewSet(1, true, deviceCnt);
	}
}

void mouseCallBck(int event, int x, int y, int flages, void* userdata)
{
	int deviceId = *((int*)userdata);
	if (event == EVENT_LBUTTONDOWN)
	{
		DepthPtr liveDepthPtr;
		liveDepthPtr.X = x;
		liveDepthPtr.Y = y;
		SetDepthPos(deviceHandleList[deviceId],liveDepthPtr);
		mouseCallbckSync[deviceId] = false;
	}
}


#if 1

/**
* @brief 		Reads the RGBD Calibration data from the Device
* @Params		Mat unregisteredIntrinsic	:		Intrinsic data of the unregistered cam matrix
				Mat registeredIntrinsic		:		Intrinsic data of the registered cam matrix
				Mat registeredDistortion	:		Distortion data of the registered cam matrix
				Mat transform				:		Transform matrix from Unregistered to registered cam
				Mat unregisteredDepth		:		Unregistered depth data
				Mat rgbFrame				:		Registered rgb data
				char* fileName				:		PLY file name
* @return		int		 1 on success
*						-1 on no valid depth data found for creating ply file
						-2 on invalid input parameters.
*/
int savePLYfiles(IntrinsicCalibParams Intrinsic, Mat Depth, Mat rgbFrame, char* fileName)
{
	Mat distortedDepth;
	if (Depth.empty())
	{
		return -2;
	}
	double focalLengthx;
	double focalLengthy;
	double principlePointx;
	double principlePointy;


	focalLengthx = Intrinsic.fx;
	focalLengthy = Intrinsic.fy;
	principlePointx = Intrinsic.cx;
	principlePointy = Intrinsic.cy;

	cv::Mat DepthFloat;
	float zoom_factor = 1.0;
	float Z;
	uint16_t raw_depth;
	int u, v;
	float x, y, z;
	color_point_t pclPoint;
	points.clear();
	Depth.convertTo(DepthFloat, CV_32F);

	for (v = 0; v < DepthFloat.rows; v++) {
		for (u = 0; u < DepthFloat.cols; u++) {
			Z = DepthFloat.at<float>(v, u) / zoom_factor;
			raw_depth = Depth.at<uint16_t>(v, u);


			if (raw_depth > depth_max_val || raw_depth < depth_min_val)
			{
				continue;
			}
			z = Z;
			x = (u - principlePointx) * Z / focalLengthx;
			y = ((v - principlePointy) * Z / focalLengthy);

			pclPoint.xyz[0] = x / METER_TO_MILLIMETER;
			pclPoint.xyz[1] = -y / METER_TO_MILLIMETER;
			pclPoint.xyz[2] = -z / METER_TO_MILLIMETER;
			pclPoint.rgb[0] = rgbFrame.at<cv::Vec3b>(v, u)[0];
			pclPoint.rgb[1] = rgbFrame.at<cv::Vec3b>(v, u)[1];
			pclPoint.rgb[2] = rgbFrame.at<cv::Vec3b>(v, u)[2];

			points.push_back(pclPoint);


		}
	}
	if (points.size() <= 0)
	{
		return -1;
	}


	//Changed by Abishek on 15/03/2023
	//Removed Pcl dependencies for Saving Ply file and replaced with file write
#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

// save to the ply file

	std::ofstream ofs(fileName); // text mode first
	ofs << PLY_START_HEADER << std::endl;
	ofs << PLY_ASCII << std::endl;
	ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "property uchar red" << std::endl;
	ofs << "property uchar green" << std::endl;
	ofs << "property uchar blue" << std::endl;
	ofs << PLY_END_HEADER << std::endl;
	ofs.close();
	std::stringstream ss;
	for (size_t i = 0; i < points.size(); ++i)
	{
		// image data is BGR
		ss << (float)points[i].xyz[0] << " " << (float)points[i].xyz[1] << " " << (float)points[i].xyz[2];
		ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
		ss << std::endl;
	}

	std::ofstream ofs_text(fileName, std::ios::out | std::ios::app);
	ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());

	return 1;
}
#endif
#ifdef _WIN32
/**
* @brief 		Preview function for Windows
* @return		void
*/
void stream(int deviceIndex)
{
	while (streamThreadCtrlFlag[deviceIndex])
	{

		while (bPreviewSet(2, true, deviceIndex))
		{
#if 1
			if (GetNextFrame(deviceHandleList[deviceIndex]) == Result::Ok) {
#if 1
				// To get RGB frame
				previewMutex[deviceIndex].lock();
				if (dataMode >= Depth_IR_RGB_VGA_Mode /*&& dataMode != Raw_Mode*/) {

					// To get the 1st RGB Frame
					if (GetToFFrame(deviceHandleList[deviceIndex], FrameType::RGBFrame, &ToFRGBFrame) > 0) {
						UYVYFrame[deviceIndex].data = ToFRGBFrame.frame_data;
					}


				}

				// To get Depth Color Map frame
				if ((dataMode != IR_Mode && dataMode <= Depth_IR_RGB_HD_Mode) ) {
					if (GetToFFrame(deviceHandleList[deviceIndex], FrameType::DepthColorMap, &ToFDepthColorMapFrame) > 0) {
						Depthstreamstarted = true;
						Depthcolormap[deviceIndex].data = (uchar*)ToFDepthColorMapFrame.frame_data;
					}
				}
				// To get IR frame
				if ((dataMode != Depth_Mode && dataMode <= Depth_IR_RGB_HD_Mode)) {
					if (GetToFFrame(deviceHandleList[deviceIndex], FrameType::IRPreviewFrame, &ToFIrFrame) > 0) {
						IRFrame[deviceIndex].data = ToFIrFrame.frame_data;
					}
				}
#endif

				previewMutex[deviceIndex].unlock();
				if (streamStarted == false)
				{
					streamStarted = true;
				}
				SetEvent(m_hEvent[deviceIndex]);

			}
			else
			{
				PrintLog(LOG_ESSENTIAL_DEBUG, "\nDepthVistaConsole:Getnextframe failed\n");
			}
#endif
		}
		Depthstreamstarted = false;
	}
}

void preview(int deviceIndex)
{
	uint32_t failCount = 0;
	struct timeval tv, res;
	struct tm tm;

	while (previewThreadCtrlFlag[deviceIndex])
	{
		while (bPreviewSet(2, true, deviceIndex))
		{
			if (streamStarted) {
				if (saveFrames[deviceIndex])
				{
					startSavingFrames[deviceIndex] = true;
					time_t t = time(0);
					localtime_s(&tm, &t);
				}

				DWORD dWait = WaitForSingleObject(m_hEvent[deviceIndex], 200);
				if (dWait == WAIT_TIMEOUT)
				{
					PrintLog(LOG_ESSENTIAL_DEBUG, "\nDepthVistaConsole:TimeoutError in renderthread");
					continue;
				}
				else
				{
					// To get RGB frame
					if (previewMutex[deviceIndex].try_lock())
					{
#if 1
						PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:Data inside preview : " + std::to_string(dataMode));
						if (dataMode >= Depth_IR_RGB_VGA_Mode /*&& dataMode != Raw_Mode */) {

							cv::cvtColor(UYVYFrame[deviceIndex], rgbFrame[deviceIndex], cv::COLOR_YUV2BGR_UYVY);
								depthVistaRGBWindowName[deviceIndex].str("");
								depthVistaRGBWindowName[deviceIndex].clear();
								depthVistaRGBWindowName[deviceIndex] << "DepthVista RGB Frame " << deviceIndex;
								namedWindow(depthVistaRGBWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);
								imshow(depthVistaRGBWindowName[deviceIndex].str().c_str(), rgbFrame[deviceIndex]);

							if (startSavingFrames[deviceIndex])
							{
								sprintf(RGBFrameFileNameBuf, "DepthVistaCam%d_RGB_%d_%d_%d_%d_%d_%d.bmp", deviceIndex,  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
								RGBFrameSaveStatus[deviceIndex] = imwrite(RGBFrameFileNameBuf, rgbFrame[deviceIndex]);
								if (RGBFrameSaveStatus[deviceIndex])
									cout << endl << "RGB frame is successfully saved as " << RGBFrameFileNameBuf << endl;
								else
									cout << endl << "Saving RGB Frame Failed" << endl;
							}

						}

						// To get Depth Color Map frame
						if ((dataMode != IR_Mode && dataMode <= Depth_IR_RGB_HD_Mode) ) {

							if (!Depthcolormap[deviceIndex].empty())
							{
								depthVistaColorMapWindowName[deviceIndex].str("");
								depthVistaColorMapWindowName[deviceIndex].clear();
								depthVistaColorMapWindowName[deviceIndex] << "DepthVista Color Map Frame " << deviceIndex;
								namedWindow(depthVistaColorMapWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);
								imshow(depthVistaColorMapWindowName[deviceIndex].str().c_str(), Depthcolormap[deviceIndex]);
								setMouseCallback(depthVistaColorMapWindowName[deviceIndex].str().c_str(), mouseCallBck, (void*)&deviceIndex);

							}

							if (startSavingFrames[deviceIndex])
							{
								sprintf(depthFrameFileNameBuf, "DepthVistaCam%d_Depth_%d_%d_%d_%d_%d_%d.bmp", deviceIndex, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
								DepthFrameSaveStatus[deviceIndex] = imwrite(depthFrameFileNameBuf, Depthcolormap[deviceIndex]);
								if (DepthFrameSaveStatus[deviceIndex])
									cout << endl << "DepthColorMap frame is successfully saved as " << depthFrameFileNameBuf << endl;
								else
									cout << endl << "Saving DepthColorMap Frame Failed" << endl;
								if (GetToFFrame(deviceHandleList[deviceIndex], FrameType::DepthRawFrame, &ToFDepthRawFrame) > 0) {
									DepthImg_Y16[deviceIndex].data = ToFDepthRawFrame.frame_data;
								}
								sprintf(depthRawFrameFileNameBuf, "DepthVistaCam%d_raw_%d_%d_%d_%d_%d_%d.raw", deviceIndex, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
								FILE* fp;
								fp = fopen(depthRawFrameFileNameBuf, "wb+");
								fwrite(DepthImg_Y16[deviceIndex].data, DepthImg_Y16[deviceIndex].total() * DepthImg_Y16[deviceIndex].channels() * DepthImg_Y16[deviceIndex].elemSize1(), 1, fp);
								fclose(fp);
								cout << endl << "Raw Depth frame is successfully saved as " << depthRawFrameFileNameBuf << endl;
								sprintf(PLY3DFileNameBuf, "DepthVistaCam%d_PLY_%d_%d_%d_%d_%d_%d.ply", deviceIndex, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
								CalibrationParams devCalibrationData;
								Result readCalibRes = GetDeviceCalibrationParams(deviceHandleList[deviceIndex], &devCalibrationData);
								if (readCalibRes == Result::Ok)
								{
									int PLYSave;
									if (rgbdMappingflag[deviceIndex])
									{
										if (dataMode == DataMode::Depth_IR_RGB_HD_Mode)
											PLYSave = savePLYfiles(devCalibrationData.rgbCamHDIntrinsic, DepthImg_Y16[deviceIndex], rgbFrame[deviceIndex], PLY3DFileNameBuf);
										else
											PLYSave = savePLYfiles(devCalibrationData.rgbCamVGAIntrinsic, DepthImg_Y16[deviceIndex], rgbFrame[deviceIndex], PLY3DFileNameBuf);
									}
									else
									{
										PLYSave = savePLYfiles(devCalibrationData.depthCamVGAIntrinsic, DepthImg_Y16[deviceIndex], Depthcolormap[deviceIndex], PLY3DFileNameBuf);
									}
									if (PLYSave > 0)
									{
										cout << endl << "3D PLY is successfully saved as " << PLY3DFileNameBuf << endl;
									}
									else
									{
										cout << endl << "3D PLY saving is failed " << PLYSave << endl;
									}

								}
								else
								{
									cout << "3D files are not saved because Calibration Data is Not Found in the device\n";
								}
							}
						}
						// To get IR frame
						if ((dataMode != Depth_Mode && dataMode <= Depth_IR_RGB_HD_Mode)) {

							depthVistaIRWindowName[deviceIndex].str("");
							depthVistaIRWindowName[deviceIndex].clear();
							depthVistaIRWindowName[deviceIndex] << "DepthVista IR Frame " << deviceIndex;
							namedWindow(depthVistaIRWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);
							imshow(depthVistaIRWindowName[deviceIndex].str().c_str(), IRFrame[deviceIndex]);

							if (startSavingFrames[deviceIndex])
							{
								sprintf(IRFrameFileNameBuf, "DepthVistaCam%d_IR_%d_%d_%d_%d_%d_%d.png", deviceIndex, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
								IRFrameSaveStatus[deviceIndex] = imwrite(IRFrameFileNameBuf, IRFrame[deviceIndex]);
								if (IRFrameSaveStatus[deviceIndex])
									cout << endl << "IR frame is successfully saved as " << IRFrameFileNameBuf << endl;
								else
									cout << endl << "Saving IR Frame Failed" << endl;
							}
						}
#endif
						previewMutex[deviceIndex].unlock();
						if (startSavingFrames[deviceIndex])
						{
							startSavingFrames[deviceIndex] = saveFrames[deviceIndex] = false;
							PrintLog(LOG_CRITICAL_DEBUG, "\nDepthVistaConsole:Inside startSavingFrames saveFrames[" + std::to_string(deviceIndex) + "] : " + std::to_string(saveFrames[deviceIndex]) );
							SetEvent(saveEvent[deviceIndex]);
						}

					}
					else
					{
						PrintLog(LOG_ESSENTIAL_DEBUG, "\nDepthVistaConsole:Trylock failed ");
					}
				}
			}

			keyPressed = waitKey(5);
			while (bSwitch)
			{
				bSwitch = false;
				destroyAllWindows();
			}
		}
		Depthstreamstarted = false;
	}
}

#elif __linux__



/**
* @brief 		Preview function for Windows
* @return		void
*/
void *stream(void* arg)
{
    int deviceIndex = *((int*)(arg));

	while (streamThreadCtrlFlag[deviceIndex])
	{
        while (bPreviewSet(2, true, deviceIndex))
		{

			if (GetNextFrame(deviceHandleList[deviceIndex]) == Result::Ok) {

				previewMutex[deviceIndex].lock();
        if(GetFrames(deviceHandleList[deviceIndex], &depthVistaFrames) > 0)
        {
          if (dataMode >= Depth_IR_RGB_VGA_Mode) {
              UYVYFrame[deviceIndex].data = depthVistaFrames.rgb.frame_data;
          }
          if ((dataMode != IR_Mode && dataMode <= Depth_IR_RGB_HD_Mode) ) {
              Depthstreamstarted = true;
              Depthcolormap[deviceIndex].data = (uchar*)depthVistaFrames.depth_colormap.frame_data;
              DepthImg_Y16[deviceIndex].data = depthVistaFrames.raw_depth.frame_data;
          }
          if ((dataMode != Depth_Mode && dataMode <= Depth_IR_RGB_HD_Mode) ) {
              IRFrame[deviceIndex].data = depthVistaFrames.ir.frame_data;
          }
        }

      	previewMutex[deviceIndex].unlock();

				if (streamStarted == false)
				{
					streamStarted = true;
				}
				pthread_cond_signal( &retrieve_cond[deviceIndex]);
			}
			else
			{
				PrintLog(LOG_ESSENTIAL_DEBUG, "\nDepthVistaConsole:Getnextframe failed\n");
			}
		}
		Depthstreamstarted = false;
	}
}

void *preview(void* arg)
{
    uint32_t failCount = 0;
    struct timeval tv, res;
    struct tm* tm;
    char cwd[256];
    int deviceInd = *((int*)(arg));

	while (previewThreadCtrlFlag[deviceInd])
	{
        while (bPreviewSet(2, true, deviceInd))
		{
			for(int deviceIndex = 0; deviceIndex < deviceHandleList.size() ; deviceIndex++)
			{
				if (streamStarted) {
					if (saveFrames[deviceIndex])
					{
						startSavingFrames[deviceIndex] = true;
						time_t t = time(0);
						tm = localtime(&t);
						getcwd(cwd, sizeof(cwd));
					}
						// To get RGB frame
					const int gettime_rv = clock_gettime(CLOCK_REALTIME, &max_wait);
					max_wait.tv_nsec =max_wait.tv_nsec + (200 * 1000 * 1000);
					pthread_mutex_lock( &wait_mutex[deviceIndex] );
					const int timed_wait_rv = pthread_cond_timedwait(&retrieve_cond[deviceIndex], &wait_mutex[deviceIndex], &max_wait);
					if (timed_wait_rv == ETIMEDOUT)
					{
						pthread_mutex_unlock( &wait_mutex[deviceIndex] );
					}
					else
					{
						pthread_mutex_unlock( &wait_mutex[deviceIndex] );
						if (previewMutex[deviceIndex].try_lock())
						{

							if (dataMode >= Depth_IR_RGB_VGA_Mode && dataMode && !UYVYFrame[deviceIndex].empty()) {
								cv::cvtColor(UYVYFrame[deviceIndex], rgbFrame[deviceIndex], cv::COLOR_YUV2BGR_UYVY);
								depthVistaRGBWindowName[deviceIndex].str("");
									depthVistaRGBWindowName[deviceIndex].clear();
									depthVistaRGBWindowName[deviceIndex] << "DepthVista RGB Frame " << deviceIndex + 1;
									namedWindow(depthVistaRGBWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);
									imshow(depthVistaRGBWindowName[deviceIndex].str().c_str(), rgbFrame[deviceIndex]);

								if (startSavingFrames[deviceIndex])
								{
									sprintf(RGBFrameFileNameBuf, "%s/DepthVistaCam%d_RGB_%d_%d_%d_%d_%d_%d.bmp", cwd, deviceIndex, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
									RGBFrameSaveStatus[deviceIndex] = imwrite(RGBFrameFileNameBuf, rgbFrame[deviceIndex]);
									if (RGBFrameSaveStatus[deviceIndex])
										cout << endl << "RGB frame is successfully saved as " << RGBFrameFileNameBuf << endl;
									else
										cout << endl << "Saving RGB Frame Failed" << endl;
								}
							}

							// To get Depth Color Map frame
							if ((dataMode != IR_Mode && dataMode <= Depth_IR_RGB_HD_Mode)) {

								if (!Depthcolormap[deviceIndex].empty())
								{
									depthVistaColorMapWindowName[deviceIndex].str("");
									depthVistaColorMapWindowName[deviceIndex].clear();
									depthVistaColorMapWindowName[deviceIndex] << "DepthVista Color Map Frame " << deviceIndex + 1;
									namedWindow(depthVistaColorMapWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);
									imshow(depthVistaColorMapWindowName[deviceIndex].str().c_str(), Depthcolormap[deviceIndex]);
									if(!mouseCallbckSync[deviceIndex])
									{
										callBckDeviceId[deviceIndex] = deviceIndex;
										mouseCallbckSync[deviceIndex] = true;
									}
									setMouseCallback(depthVistaColorMapWindowName[deviceIndex].str().c_str(), mouseCallBck, (void*)&(callBckDeviceId[deviceIndex]));

								}

								if (startSavingFrames[deviceIndex])
								{
									sprintf(depthFrameFileNameBuf, "%s/DepthVistaCam%d_Depth_%d_%d_%d_%d_%d_%d.bmp",cwd, deviceIndex, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
									DepthFrameSaveStatus[deviceIndex] = imwrite(depthFrameFileNameBuf, Depthcolormap[deviceIndex]);
									if (DepthFrameSaveStatus[deviceIndex])
										cout << endl << "DepthColorMap frame is successfully saved as " << depthFrameFileNameBuf << endl;
									else
										cout << endl << "Saving DepthColorMap Frame Failed" << endl;
									sprintf(depthRawFrameFileNameBuf, "%s/DepthVistaCam%d_raw_%d_%d_%d_%d_%d_%d.raw", cwd, deviceIndex, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
									FILE* fp;
									fp = fopen(depthRawFrameFileNameBuf, "wb+");
									fwrite(DepthImg_Y16[deviceIndex].data, DepthImg_Y16[deviceIndex].total() * DepthImg_Y16[deviceIndex].channels() * DepthImg_Y16[deviceIndex].elemSize1(), 1, fp);
									fclose(fp);
									cout << endl << "Raw Depth frame is successfully saved as " << depthRawFrameFileNameBuf << endl;
                  sprintf(PLY3DFileNameBuf, "%s/DepthVistaCam%d_3D_%d_%d_%d_%d_%d_%d.ply", cwd, deviceIndex, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
                  CalibrationParams devCalibrationData;
                  Result readCalibRes = GetDeviceCalibrationParams(deviceHandleList[deviceIndex], &devCalibrationData);
  								if (readCalibRes == Result::Ok)
  								{
  									int PLYSave;
  									if (rgbdMappingflag[deviceIndex])
  									{
  										if (dataMode == DataMode::Depth_IR_RGB_HD_Mode)
  											PLYSave = savePLYfiles(devCalibrationData.rgbCamHDIntrinsic , DepthImg_Y16[deviceIndex], rgbFrame[deviceIndex], PLY3DFileNameBuf);
  										else
  											PLYSave = savePLYfiles(devCalibrationData.rgbCamVGAIntrinsic, DepthImg_Y16[deviceIndex], rgbFrame[deviceIndex], PLY3DFileNameBuf);
  									}
  									else
  									{
  										PLYSave = savePLYfiles(devCalibrationData.depthCamVGAIntrinsic, DepthImg_Y16[deviceIndex], Depthcolormap[deviceIndex], PLY3DFileNameBuf);
  									}
  									if (PLYSave > 0)
  									{
  										cout << endl << "3D PLY is successfully saved as " << PLY3DFileNameBuf << endl;
  									}
  									else
  									{
  										cout << endl << "3D PLY saving is failed " << PLYSave << endl;
  									}

  								}
  								else
  								{
  									cout << "3D files are not saved because Calibration Data is Not Found in the device\n";
  								}
								}
							}
							// To get IR frame
							if ((dataMode != Depth_Mode && dataMode <= Depth_IR_RGB_HD_Mode && !IRFrame[deviceIndex].empty())) {

								depthVistaIRWindowName[deviceIndex].str("");
								depthVistaIRWindowName[deviceIndex].clear();
								depthVistaIRWindowName[deviceIndex] << "DepthVista IR Frame " << deviceIndex +1;
								namedWindow(depthVistaIRWindowName[deviceIndex].str().c_str(), WINDOW_AUTOSIZE);
								imshow(depthVistaIRWindowName[deviceIndex].str().c_str(), IRFrame[deviceIndex]);

								if (startSavingFrames[deviceIndex])
								{
									sprintf(IRFrameFileNameBuf, "%s/DepthVistaCam%d_IR_%d_%d_%d_%d_%d_%d.png",cwd,  deviceIndex, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
									IRFrameSaveStatus[deviceIndex] = imwrite(IRFrameFileNameBuf, IRFrame[deviceIndex]);
									if (IRFrameSaveStatus[deviceIndex])
										cout << endl << "IR frame is successfully saved as " << IRFrameFileNameBuf << endl;
									else
										cout << endl << "Saving IR Frame Failed" << endl;
								}
							}
							previewMutex[deviceIndex].unlock();

							if (startSavingFrames[deviceIndex])
							{
								startSavingFrames[deviceIndex] = saveFrames[deviceIndex] = false;
							}
						}
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
		Depthstreamstarted = false;

	}

}

#endif



int main()
{
	//Basic Introduction about the Application
	cout << endl << "e-con's Sample Application for DepthVista " << endl;
	cout << endl << "Demonstrates the working of e-con's DepthVistaSDK" << endl;
	uint8_t gMajorVersion, gMinorVersion1;
	uint16_t gMinorVersion2;
	if (GetSDKVersion(&gMajorVersion, &gMinorVersion1, &gMinorVersion2) <= 0)
	{
		cout << endl << "\t" << "SDK version Failed" << endl;
	}
	cout << endl << "\t" << "DepthVista SDK-Version = " << (uint16_t)gMajorVersion << "." << (uint16_t)gMinorVersion1 << "." << gMinorVersion2 << "\n\n";

	//Initialize DepthVista
	if (Initialize() < 0) {
		pError();
	}
	SetLogLevel((LogLevel)2);
	//Open a Camera Device
	if (!(listDevices()))
	{
		cout << endl << "List Devices Information Failed main()\n" << endl;
		cout << endl << '\t' << "Press Any key to exit the Application: " << '\t';
#ifdef _WIN32
		_getch();
#endif
		return 0;
	}

    if (!(exploreCam()))
    {
        cout << endl << "Camera Exploration Failed" << endl;
        return 0;
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
    devices = 0;
	Result getDeviceRes;
	getDeviceRes = GetDeviceCount(&devices);
	if (getDeviceRes  != Result::Ok && getDeviceRes != Result::NoDeviceConnected) {
		pError("GetDeviceCount");
		cout << endl << "GetDeviceCount failed getDeviceRes :  " << getDeviceRes << endl;
		return false;
	}
	maxDeviceIndex = devices;
    if (devices < 0 || getDeviceRes == Result::NoDeviceConnected)
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

    cout << endl << "Number of Camera Devices Connected to the Port : " << devices <<   endl;
	if (exploreOpt == exploreOptions::notInitialize && devices > 1)
	{
		int exploreOption = -1;
		cout << endl << "How would you like to explore the devices" << endl;

		cout << '\t' << "0  - Single Device" << endl;
		cout << '\t' << "1  - Multiple Devices" << endl;

		while ((exploreOption < 0) || (exploreOption > devices))
		{
			printf("\n Pick a relavent Option: \t");
			scanf("%d", &exploreOption);
			while (getchar() != '\n' && getchar() != EOF)
			{

			}
		}
		exploreOpt = exploreOption;
	}
	if (exploreOpt == exploreOptions::singleCam || devices == 1)
	{
		//This is to initialize exploreOpt when devices == 1
		exploreOpt = 0;
		cout << endl << "Camera Devices Connected to the PC Port : " << endl << endl;
		cout << '\t' << "0 - Exit" << endl;

		//List the Camera Names
		for (int eachDevice = 0; eachDevice < devices; eachDevice++)
		{
			cout << '\t' << eachDevice + 1 << " - " << (gDevicesList + eachDevice)->deviceName << " (" <<  (gDevicesList + eachDevice)->serialNo << ")\n";
		}
		while ((camId < 0) || (camId > devices))
		{
			printf("\n Pick a Camera Device to Explore : \t");
			scanf("%d", &camId);
			while (getchar() != '\n' && getchar() != EOF)
			{

			}
		}
	}
    switch (camId)
    {
    case EXIT:
#ifdef _WIN32
		if (bDetach)
		{
			for (int deviceInd = 0; deviceInd < maxDeviceIndex; deviceInd)
			{
				bPreviewSet(1, false, deviceInd);
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
		}
#endif
        if (DeInitialize() > 0)
        {
            destroyAllWindows();
        }

        exit(0);
        break;

    default:
#ifdef _WIN32
		for(int itr = 0; itr < devices; itr++)
		{
      bSwitch = true;
  bPreviewSet(1, false, itr);
  streamStarted = false;
  previewThreadCtrlFlag[itr] = false;
	streamThreadCtrlFlag[itr] = false;
			if (bDetach)
			{
				if (streamThread[itr].joinable())
				{
					streamThread[itr].detach();
				}
				if (previewThread[itr].joinable())
				{
					previewThread[itr].detach();
				}
			}
			PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:after detach");
		}
#elif __linux__
		for(int itr = 0; itr < devices ; itr++)
		{
      bSwitch = true;
  bPreviewSet(1, false, itr);
  streamStarted = false;
  previewThreadCtrlFlag[itr] = false;
  streamThreadCtrlFlag[itr] = false;

			pthread_join(streamThread[itr], NULL);
	  pthread_join(previewThread[itr], NULL);
		}
#endif
		currentDeviceIndex = camId - 1;

		for (int deviceNO = 0 ; deviceNO < devices; deviceNO++)
		{
				DeviceHandle devHandle;
				cv::Mat depthIntrinsic;
				cv::Mat rgbIntrinsic;
				cv::Mat rgbHDDistortion;

				if (exploreOpt == exploreOptions::singleCam)
				{

					if (deviceNO != currentDeviceIndex && deviceHandleList.size() > 0)
					{
						PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:before Closedevice " );
						if (CloseDevice(deviceHandleList[deviceNO]) > 0)
						{
							deviceHandleList.erase(deviceHandleList.begin() + deviceNO);
						}
						else
						{
							PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:Closedevice for "+ to_string(deviceNO) + " failed" );
						}

					}
					if (deviceNO == currentDeviceIndex)
					{

            Result odRes = OpenDevice(gDevicesList[deviceNO], &devHandle);
						if (odRes < 0) {

              if(odRes == Result::CameraAlreadyOpen)
              {
                previewThreadCtrlFlag[0] = true;
                streamThreadCtrlFlag[0] = true;
                tempDeviceIndex[0] = 0;
#ifdef _WIN32
				PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:Before creating thread\n");

				previewThread[deviceNO] = thread(&preview, deviceNO);
				streamThread[deviceNO] = thread(&stream, deviceNO);

				PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:After creating thread\n");
#elif __linux__
                pthread_create(&streamThread[0], NULL, stream, &tempDeviceIndex[0]);
                pthread_create(&previewThread[0], NULL, preview, &tempDeviceIndex[0]);
#endif
                startStream();
                return true;
              }
              else
              {
                pError("OpenDevice:");
				std::cout << "Open device failed : " << odRes << "\n";
                return false;
              }
						}

						deviceHandleList.insert(deviceHandleList.begin(), devHandle);

						rgbdMappingflag.insert(rgbdMappingflag.begin(), false);

            RegisterNotificationCallback(devHandle, &registerNotificationCb);
						previewThreadCtrlFlag[0] = true;
						streamThreadCtrlFlag[0] = true;
#ifdef _WIN32
							previewThread[0] = thread(&preview, 0);
							streamThread[0] = thread(&stream, 0);

#elif __linux__
						tempDeviceIndex[0] = 0;
                        pthread_create(&streamThread[0], NULL, stream, &tempDeviceIndex[0]);
                        pthread_create(&previewThread[0], NULL, preview, &tempDeviceIndex[0]);
#endif
					}

				}
				else
				{

            Result OpenDeviceRes = OpenDevice(gDevicesList[deviceNO], &devHandle);
						if (OpenDeviceRes < 0) {
							pError("OpenDevice:");
							return false;
						}
						else
						{
							deviceHandleList.push_back(devHandle);
							rgbdMappingflag.push_back(false);
              RegisterNotificationCallback(deviceHandleList[deviceNO], &registerNotificationCb);

						}
				}
		}

		if(exploreOpt != exploreOptions::singleCam)
		{

			currentDeviceIndex = 0;
#if 1
#ifdef _WIN32
			for (int deviceNO = 0; deviceNO < deviceHandleList.size(); deviceNO++)
			{
        streamThreadCtrlFlag[deviceNO] = true;
		previewThreadCtrlFlag[deviceNO] = true;
				previewThread[deviceNO] = thread(&preview, deviceNO);
				streamThread[deviceNO] = thread(&stream, deviceNO);
			}
#elif __linux__
			for (int deviceNO = 0; deviceNO < maxDeviceIndex; deviceNO++)
			{
        		streamThreadCtrlFlag[deviceNO] = true;
				previewThreadCtrlFlag[deviceNO] = true;
				tempDeviceIndex[deviceNO] = deviceNO;
                pthread_create(&streamThread[deviceNO], NULL, stream, &tempDeviceIndex[deviceNO]);
                pthread_create(&previewThread[deviceNO], NULL, preview, &tempDeviceIndex[deviceNO]);
			}
#endif
#endif
		}
#if 1
		for (int deviceNO = 0; deviceNO < deviceHandleList.size(); deviceNO++)
		{
			if (GetDataMode(deviceHandleList[deviceNO], &dataMode) < 0) {
				pError("GetDataMode");
			}

			if (GetDepthRange(deviceHandleList[deviceNO], &depthRange) < 0) {
				pError("GetDepthRange");
			}

			if (SetAvgRegion(deviceHandleList[deviceNO], AvgRegion::CustomPtr) < 0)
			{
				cout << "SetAvgRegion failed\n" << endl;
			}

			if (depthRange == DepthRange::NearRange)
			{
				depth_min_val = NEAR_MODE_MIN;
				depth_max_val = NEAR_MODE_MAX;
			}
			else if (depthRange == DepthRange::FarRange)
			{
				depth_min_val = FAR_MODE_MIN;
				depth_max_val = FAR_MODE_MAX;
			}
		}


		startStream();

#endif
        break;
    }
#if 1
#ifdef _WIN32
    bDetach = true;
	saveEvent[0] = CreateEvent(NULL, FALSE, FALSE, L"frameCaptureEvent0");
	saveEvent[1] = CreateEvent(NULL, FALSE, FALSE, L"frameCaptureEvent1");
		m_hEvent[0] = CreateEvent(NULL, FALSE, FALSE, L"renderEvent0");
		m_hEvent[1] = CreateEvent(NULL, FALSE, FALSE, L"renderEvent1");
#endif
#endif
    return true;
}

/**
* @brief 		Setting Streaming Mode of the Device
* @return		bool    return true on successfully setting the stream mode of the device, else retuns fail.
*/
bool selectStreamingMode()
{
	int option = -1;

    cout << endl << "Total Number of Streaming Modes Supported by the Camera:  " << '\t' << "9" << endl;
    cout << '\t' << "0  - Exit" << endl;
    cout << '\t' << "1  - Back" << endl;
    cout << '\t' << "2  - Main Menu" << endl;
    cout << '\t' << "3  - Depth IR Mode" << endl;
    cout << '\t' << "4  - Depth Mode" << endl;
    cout << '\t' << "5  - IR Mode" << endl;
    cout << '\t' << "6  - Depth IR RGB VGA Mode" << endl;
    cout << '\t' << "7  - Depth IR RGB HD Mode" << endl;
    cout << '\t' << "8  - RGB VGA Mode" << endl;
    cout << '\t' << "9  - RGB HD Mode" << endl;
    cout << '\t' << "10 - RGB Full HD Mode" << endl;
    cout << '\t' << "11 - RGB 1920x1200 Mode" << endl;
    while (1)
    {
        printf("\n Pick a Relevant Streaming Mode: \t");
        scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF)
        {
        }
		if((option >= 0) && (option < 12))
		{
			break;
		}

    }

    switch (option)
    {
    case EXIT:
        bSwitch = true;

			for (int deviceInd = 0; deviceInd < deviceHandleList.size() ; deviceInd++)
			{
				bPreviewSet(1, false, deviceInd);
				previewThreadCtrlFlag[deviceInd] = false;
				streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
				if (bDetach)
				{
					if (streamThread[deviceInd].joinable())
					{
						streamThread[deviceInd].detach();
					}
					if (previewThread[deviceInd].joinable())
					{
						previewThread[deviceInd].detach();
					}
				}
#endif
				if (CloseDevice(deviceHandleList[deviceInd]) > 0)
				{
					std::cout << "Device Close success\n";
				}
			}
			if (DeInitialize() > 0)
			{
				destroyAllWindows();

			}

        exit(0);

    case 1:
    case 2:
        exploreCam();
        break;

    case 3:

        if (dataMode != option - 3)
        {
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size() ; ctrlDeviceIndex++)
			{

				bSwitch = true;
				bPreviewSet(1, false, ctrlDeviceIndex);
				streamStarted = false;
				if (SetDataMode(deviceHandleList[ctrlDeviceIndex], (DataMode)(option - 3)) < 0) {
					pError("SetDataMode");
					return false;
				}
				dataMode = (DataMode)(option - 3);
				if (exploreOpt == exploreOptions::singleCam)
					break;
			}
        }
        break;

    default:

        if (dataMode != option - 2)
        {
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{

				bSwitch = true;
				bPreviewSet(1, false, ctrlDeviceIndex);
				streamStarted = false;
				if (SetDataMode(deviceHandleList[ctrlDeviceIndex], (DataMode)(option - 2)) < 0) {
					pError("SetDataMode");
					return false;
				}
				dataMode = (DataMode)(option - 2);
				if (exploreOpt == exploreOptions::singleCam)
					break;
			}
        }
        break;
    }
    startStream();
    return true;
}

/**
* @brief 		Setting depth value Mode of the Device
* @return		bool    return true on successfully setting the stream mode of the device, else retuns fail.
*/
bool selectGetDepthValue()
{
    int avgDepth, stdDepth, avgIR, stdIR;
    DepthPtr cor;
	int avgX = -1, avgY = -1;

    cout << '\t' << "0 - Exit" << endl;
    cout << '\t' << "1 - Back" << endl;
    cout << '\t' << "2 - Main Menu" << endl;
    cout << '\t' << "3 - Centre" << endl;
    cout << '\t' << "4 - Custom co-ordinate" << endl;

    int option = -1;
    while ((option < 0) || (option >= 5))
    {
        printf("\n Pick a Relevant Depth value position: \t");
        scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF)
        {
        }
    }

    switch (option)
    {
    case EXIT:
        bSwitch = true;

			for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
			{
				bPreviewSet(1, false, deviceInd);
				previewThreadCtrlFlag[deviceInd] = false;
				streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
				if (bDetach)
				{
					if (streamThread[deviceInd].joinable())
					{
						streamThread[deviceInd].detach();
					}
					if (previewThread[deviceInd].joinable())
					{
						previewThread[deviceInd].detach();
					}
				}
#endif
				if (CloseDevice(deviceHandleList[deviceInd]) > 0)
				{
				}
			}
                if (DeInitialize() > 0)
                {
                    destroyAllWindows();

                }
            exit(0);
        case 1:
        case 2:
            exploreCam();
            break;

        case 3:
			while ((avgX < MIN_AVG_X) || (avgX > MAX_AVG_X))
			{
				printf("\n Enter width of average kernal :\t");
				scanf("%d", &avgX);
				while (getchar() != '\n' && getchar() != EOF)
				{
				}
			}
			while ((avgY < MIN_AVG_Y) || (avgY > MAX_AVG_Y))
			{
				printf("\n Enter height of average kernal :\t");
				scanf("%d", &avgY);
				while (getchar() != '\n' && getchar() != EOF)
				{
				}
			}
			for (int deviceCnt = 0; deviceCnt < deviceHandleList.size(); deviceCnt++)
			{

        if (SetAvgRegion(deviceHandleList[deviceCnt], AvgRegion::Center) < 0)
        {
        	cout << "SetAvgRegion failed\n" << endl;
        }

				if (UpdateAvgXandY(deviceHandleList[deviceCnt], avgX, avgY) < 0)
				{
					cout << "UpdateAvgXandY failed\n" << endl;
				}

				if (GetDepthIRValues(deviceHandleList[deviceCnt], &avgDepth, &stdDepth, &avgIR, &stdIR) < 1)
				{
					return false;
				}
				else
				{
					cout << "\nDepth Value of camera " << deviceCnt << " : " << avgDepth << endl;
				}
				if (SetAvgRegion(deviceHandleList[deviceCnt], AvgRegion::CustomPtr) < 0)
				{
					cout << "SetAvgRegion failed\n" << endl;
				}
				else
				{
					cor.X = PRE_RGB_VGA_WIDTH / 2;
					cor.Y = PRE_RGB_VGA_HEIGHT / 2;
					if (SetDepthPos(deviceHandleList[deviceCnt], cor) < 1)
					{
						cout << "\nFailed to set depth position" << endl;
						return false;
					}
				}
				if (exploreOpt == exploreOptions::singleCam)
					break;
			}
            break;

        case 4:
			while ((avgX < MIN_AVG_X) || (avgX > MAX_AVG_X))
			{
				printf("\n Enter width of average kernal :\t");
				scanf("%d", &avgX);
				while (getchar() != '\n' && getchar() != EOF)
				{
				}
			}
			while ((avgY < MIN_AVG_Y) || (avgY > MAX_AVG_Y))
			{
				printf("\n Enter height of average kernal :\t");
				scanf("%d", &avgY);
				while (getchar() != '\n' && getchar() != EOF)
				{
				}
			}
			for (int deviceCnt = 0; deviceCnt < deviceHandleList.size(); deviceCnt++)
			{

				if (UpdateAvgXandY(deviceHandleList[deviceCnt], avgX, avgY) < 0)
				{
					cout << "UpdateAvgXandY failed\n" << endl;
				}
				cor.X = -1;
				while ((cor.X <= avgX/2) || (cor.X > 640 - avgX/2))
				{
					printf("\n Enter X co-ordinate between the range %d to %d for camera %d: \t", avgX/2 +1 , 640 - avgX / 2,  deviceCnt);
					scanf("%d", &cor.X);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				}
				cor.Y = -1;
				while ((cor.Y <= avgY/2) || (cor.Y > 480 - avgY/2))
				{
					printf("\n Enter Y co-ordinate between the range %d to %d for camera %d: \t",avgY/2 +1,  480 - avgY / 2,  deviceCnt);
					scanf("%d", &cor.Y);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				}
				if(SetDepthPos(deviceHandleList[deviceCnt], cor) < 1)
				{
					cout<<"\nFailed to set depth position"<<endl;
					return false;
				}
				else
				{
					if(GetDepthIRValues(deviceHandleList[deviceCnt], &avgDepth, &stdDepth, &avgIR, &stdIR) < 1)
					{
						cout<<"\nFailed to get depth value"<<endl;
						return false;
					}
					else
					{
						cout<<"\nDepth Value of camera " << deviceCnt << " : "<<avgDepth<<endl;
					}
				}
			}
            break;
    }
    return true;

}

/**
* @brief 		Setting UVC Controls to the Device
* @return		bool    return true on successfully setting the UVC Controls of the device, else retuns fail.
*/
bool uvcControlMenu()
{
    cout << '\t' << "0  - Exit" << endl;
    cout << '\t' << "1  - Back" << endl;
    cout << '\t' << "2  - Main Menu" << endl;
    cout << '\t' << "3  - BRIGHTNESS" << endl;
    cout << '\t' << "4  - CONTRAST" << endl;
    cout << '\t' << "5  - SATURATION" << endl;
    cout << '\t' << "6  - WHITE BALANCE MODE" << endl;
	cout << '\t' << "7  - WHITE BALANCE CONTROL" << endl;
    cout << '\t' << "8  - GAMMA" << endl;
    cout << '\t' << "9  - GAIN" << endl;
    cout << '\t' << "10 - POWER LINE FREQ" << endl;
    cout << '\t' << "11 - SHARPNESS" << endl;
	cout << '\t' << "12 - EXPOSURE MODE" << endl;
	cout << '\t' << "13 - EXPOSURE CONTROL" << endl;

    int option = -1;
    int value = 0;
	UVCProp gProp;

    while ((option < 0) || (option >= 14))
    {
        printf("\n Pick a Relevant UVC Property: \t");
        scanf("%d", &option);
        while (getchar() != '\n' && getchar() != EOF)
        {
        }
    }

    switch (option)
    {
    case EXIT:
        bSwitch = true;

		for (int deviceInd = 0; deviceInd < deviceHandleList.size() ; deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif

			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}
                if (DeInitialize() > 0)
                {
                    destroyAllWindows();

                }
            exit(0);

        case 1:
        case 2:
            exploreCam();
            break;
        case 3: //brightness
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{
				cout << "Brightness control for the device " << ctrlDeviceIndex + 1 << endl;

				if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_BRIGHTNESS, &gProp) > 0))
				{
					cout << "Query UVC property failed" << endl;
					return false;
				}
				do
				{
					cout << "Current Brightness value is : " << gProp.cur << endl;
					cout << "Enter relavent Brightness value between " << gProp.min << " to " << gProp.max << " : " << endl;
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				} while ((value < gProp.min) || (value > gProp.max));
				if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_BRIGHTNESS, value) < 0) {
					cout << "\nFailed to set BRIGHTNESS value" << endl;
					return false;
				}
			}
            break;
        case 4: //contrast
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{

				cout << "Contrast control for the device " << ctrlDeviceIndex + 1 << endl;

				if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_CONTRAST, &gProp) > 0))
				{
					cout << "Query UVC property failed" << endl;
					return false;
				}
				do
				{
					cout << "Current CONTRAST value is : " << gProp.cur << endl;
					cout << "Enter relavent CONTRAST value between " << gProp.min << " to " << gProp.max << " : " << endl;
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				} while ((value < gProp.min) || (value > gProp.max));
				if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_CONTRAST, value) < 0) {
					cout << "\nFailed to set CONTRAST value" << endl;
					return false;
				}
				if (exploreOpt == exploreOptions::singleCam)
					break;
			}
            break;
        case 5: //saturation
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{

				cout << "Saturation control for the device " << ctrlDeviceIndex + 1 << endl;

				if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_SATURATION, &gProp) > 0))
				{
					cout << "Query UVC property failed" << endl;
					return false;
				}
				do
				{
					cout << "Current SATURATION value is : " << gProp.cur << endl;
					cout << "Enter relavent SATURATION value between " << gProp.min << " to " << gProp.max << " : " << endl;
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				} while ((value < gProp.min) || (value > gProp.max));
				if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_SATURATION, value) < 0) {
					cout << "\nFailed to set SATURATION value" << endl;
					return false;
				}
				if (exploreOpt == exploreOptions::singleCam)
					break;
			}
            break;
        case 6: //white balance mode (AUTO/MANUAL)
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{

				cout << "White Balance mode control for the device " << ctrlDeviceIndex + 1 << endl;

				cout << '\t' << "1 - WHITE BALANCE AUTO OFF" << endl;
				cout << '\t' << "2 - WHITE BALANCE AUTO ON" << endl;

				value = 0;
				while ((value < 1) || (value >= 3))
				{
					printf("\n Pick a Relevant option: \t");
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				}
				value = value - 1;
				if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_WB_AUTO, value) < 0) {
					cout << "\nFailed to set WHITE BALANCE AUTO value" << endl;
					return false;
				}
				if (exploreOpt == exploreOptions::singleCam)
					break;
			}
            break;
		case 7: //White balance control
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{
				if (GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_WB_AUTO, &gProp) < 0)
				{
					cout << "Query UVC property failed" << endl;
					return false;
				}
				else
				{
					if (gProp.cur == 1)
					{
						cout << "White balance mode set to AUTO. Set White balance mode to MANUAL to change White Balance control" << endl;
						break;
					}
					else
					{
						cout << "White Balance control for the device " << ctrlDeviceIndex + 1 << endl;

						if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_WB_TEMP, &gProp) > 0))
						{
							cout << "Query UVC property failed" << endl;
							return false;
						}
						do
						{
							cout << "Current WHITE BALANCE value is : " << gProp.cur << endl;
							cout << "Enter relavent WHITE BALANCE value between " << gProp.min << " to " << gProp.max << " : " << endl;
							scanf("%d", &value);
							while (getchar() != '\n' && getchar() != EOF)
							{
							}
						} while ((value < gProp.min) || (value > gProp.max));
						if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_WB_TEMP, value) < 0) {
							cout << "\nFailed to set WHITE BALANCE value" << endl;
							return false;
						}
						if (exploreOpt == exploreOptions::singleCam)
							break;
					}
				}
			}
			break;
        case 8: //Gamma control
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{

				cout << "Gamma control for the device " << ctrlDeviceIndex + 1 << endl;

				if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_GAMMA, &gProp) > 0))
				{
					cout << "Query UVC property failed" << endl;
					return false;
				}
				do
				{
					cout << "Current GAMMA value is : " << gProp.cur << endl;
					cout << "Enter relavent GAMMA value between " << gProp.min << " to " << gProp.max << " : " << endl;
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				} while ((value < gProp.min) || (value > gProp.max));
				if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_GAMMA, value) < 0) {
					cout << "\nFailed to set GAMMA value" << endl;
					return false;
				}
				if (exploreOpt == exploreOptions::singleCam)
					break;
			}
            break;
        case 9: //Gain control
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{
				if (GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_EXPOSURE_AUTO, &gProp) < 0)
				{
					cout << "Query UVC property failed" << endl;
					return false;
				}
				else 
				{
					cout << "exposure auto or manual : " << gProp.cur << endl;
					if (gProp.cur == 0)
					{
						cout << "Exposure mode set to AUTO. Set Exposure mode to MANUAL to change gain" << endl;
						break;
					}
					else 
					{
						cout << "Gain control for the device " << ctrlDeviceIndex + 1 << endl;

						if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_GAIN, &gProp) > 0))
						{
							cout << "Query UVC property failed" << endl;
							return false;
						}
						do
						{
							cout << "Current GAIN value is : " << gProp.cur << endl;
							cout << "Enter relavent GAIN value between " << gProp.min << " to " << gProp.max << " : " << endl;
							scanf("%d", &value);
							while (getchar() != '\n' && getchar() != EOF)
							{
							}
						} while ((value < gProp.min) || (value > gProp.max));
						if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_GAIN, value) < 0) {
							cout << "\nFailed to set GAIN value" << endl;
							return false;
						}
						if (exploreOpt == exploreOptions::singleCam)
							break;
					}
				}
			}
            break;
        case 10: //Powerline Freq
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{

				cout << "Power line frequency control for the device " << ctrlDeviceIndex + 1 << endl;

				if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_PWR_LINE_FREQ, &gProp) > 0))
				{
					cout << "Query UVC property failed" << endl;
					return false;
				}
				do
				{
					cout << "Current POWER LINE FREQ value is : " << gProp.cur << endl;
					cout << "Enter relavent POWER LINE FREQ value between " << gProp.min << " to " << gProp.max << " : " << endl;
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				} while ((value < gProp.min) || (value > gProp.max));
				if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_PWR_LINE_FREQ, value) < 0) {
					cout << "\nFailed to set POWER LINE FREQ value" << endl;
					return false;
				}
				if (exploreOpt == exploreOptions::singleCam)
					break;
			}

            break;
        case 11: //Sharpness
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{

				cout << "Sharpness control for the device " << ctrlDeviceIndex + 1 << endl;

				if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_SHARPNESS, &gProp) > 0))
				{
					cout << "Query UVC property failed" << endl;
					return false;
				}
				do
				{
					cout << "Current SHARPNESS value is : " << gProp.cur << endl;
					cout << "Enter relavent SHARPNESS value between " << gProp.min << " to " << gProp.max << " : " << endl;
					scanf("%d", &value);
					while (getchar() != '\n' && getchar() != EOF)
					{
					}
				} while ((value < gProp.min) || (value > gProp.max));
				if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_SHARPNESS, value) < 0) {
					cout << "\nFailed to set SHARPNESS value" << endl;
					return false;
				}
				if (exploreOpt == exploreOptions::singleCam)
					break;
			}
            break;
		case 12: //Exposure mode (AUTO/MANUAL) 
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{
				DataMode mode;
				GetDataMode(deviceHandleList[ctrlDeviceIndex], &mode);
				if (mode > DataMode::Depth_IR_RGB_HD_Mode)
				{
					cout << "Exposure mode control for the device " << ctrlDeviceIndex + 1 << endl;

					cout << '\t' << "1 - EXPOSURE AUTO ON" << endl;
					cout << '\t' << "2 - EXPOSURE AUTO OFF" << endl;

					value = 0;
					while ((value < 1) || (value >= 3))
					{
						printf("\n Pick a Relevant option: \t");
						scanf("%d", &value);
						while (getchar() != '\n' && getchar() != EOF)
						{
						}
					}
					value = value - 1;
					if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_EXPOSURE_AUTO, value) < 0) {
						cout << "\nFailed to set WHITE BALANCE AUTO value" << endl;
						return false;
					}
					if (exploreOpt == exploreOptions::singleCam)
						break;
				}
				else
				{
					cout << "Exposure mode cannot be changed in current DataMode. Control supported only in RGB modes" << endl;
				}
			}
			break;
		case 13: //Exposure control
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{
				if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_EXPOSURE_AUTO, &gProp) > 0))
				{
					cout << "Query UVC property failed" << endl;
					return false;
				}
				else
				{
					if (gProp.cur == 0)
					{
						cout << "Exposure mode set to AUTO. Set Exposure mode to MANUAL to change Exposure" << endl;
						break;
					}
					else
					{
						cout << "Exposure control for the device " << ctrlDeviceIndex + 1 << endl;

						if (!(GetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_EXPSOURE_ABS, &gProp) > 0))
						{
							cout << "Query UVC property failed" << endl;
							return false;
						}
						do
						{
							cout << "Current EXPOSURE value is : " << gProp.cur << endl;
							cout << "Enter relavent EXPOSURE value between " << gProp.min << " to " << gProp.max << " : " << endl;
							scanf("%d", &value);
							while (getchar() != '\n' && getchar() != EOF)
							{
							}
						} while ((value < gProp.min) || (value > gProp.max));
						if (SetUVCControl(deviceHandleList[ctrlDeviceIndex], TOF_UVC_CID_EXPSOURE_ABS, value) < 0) {
							cout << "\nFailed to set EXPOSURE value" << endl;
							return false;
						}
						if (exploreOpt == exploreOptions::singleCam)
							break;
					}
				}
			}
			break;
    }
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

		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd]  = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}

			if (DeInitialize() > 0)
			{
				destroyAllWindows();
			}
		exit(0);

	case 1:
	case 2:
		exploreCam();
		break;

	default:
		for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
		{

			if (dataMode <= Depth_IR_RGB_HD_Mode)
			{
				if (SetDepthRange(deviceHandleList[ctrlDeviceIndex], option - 3) < 0) {
					pError("SetDepthRange");
					return false;
				}
				cout << "SetDepthRange success for " << ctrlDeviceIndex << endl;
				depthRange = option - 3;
			}
			if (depthRange == DepthRange::NearRange)
			{
				Depth_min = NEAR_MODE_MIN;
				Depth_max = NEAR_MODE_MAX;
			}
			else
			{
				Depth_min = FAR_MODE_MIN;
				Depth_max = FAR_MODE_MAX;
			}
			depth_offset = (Depth_max - Depth_min) * 0.20;
			UpdateColorMap(deviceHandleList[ctrlDeviceIndex], Depth_min, Depth_max + depth_offset, 4);
			if (exploreOpt == exploreOptions::singleCam)
				break;
		}

		break;

	}
	return true;
}

/**
* @brief 		Setting IMU Embedded data
* @return		bool    return true on successfully setting the IMU Embedded data, else retuns fail.
*/
bool imuEmbedDataMenu()
{
	cout << '\t' << "0 - Exit" << endl;
	cout << '\t' << "1 - Back" << endl;
	cout << '\t' << "2 - Main Menu" << endl;
	cout << '\t' << "3 - Disable IMU Embedded data" << endl;
	cout << '\t' << "4 - Enable IMU Embedded data" << endl;

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

		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}
			if (DeInitialize() > 0)
			{
				destroyAllWindows();
			}
		exit(0);

	case 1:
		tofControlSettings();
		break;
	case 2:
		exploreCam();
		break;

	default:
		for (uint16_t ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
		{

			PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:before SetIMUEmbeddedData ctrlDeviceIndex is : " + to_string(ctrlDeviceIndex) );

			if (SetIMUEmbeddedData(deviceHandleList[ctrlDeviceIndex], (uint8_t)option - 3) < 0) {
				pError("SetIMUEmbeddedData");
				return false;
			}
			if (exploreOpt == exploreOptions::singleCam)
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
bool irGainMenu()
{
	cout << '\t' << "0 - Exit" << endl;
	cout << '\t' << "1 - Back" << endl;
	cout << '\t' << "2 - Main Menu" << endl;
	cout << '\t' << "3 - Get IR Gain" << endl;
	cout << '\t' << "4 - Set IR Gain" << endl;

	int option = -1;
	uint16_t irGain = 65535;
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

		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}
			if (DeInitialize() > 0)
			{
				destroyAllWindows();
			}
		exit(0);

	case 1:
		tofControlSettings();
		break;
	case 2:
		exploreCam();
		break;

	case 3:
		for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
		{

			if (GetTOFIRGain(deviceHandleList[ctrlDeviceIndex], &irGain) > 0)
			{
				cout << "Current IR Gain value is : " << irGain << endl;
			}
			else
			{
				cout << "Getting IR Gain value failed" << endl;
				return false;
			}
			if (exploreOpt == exploreOptions::singleCam)
				break;
		}
		break;
	case 4:
		while ((irGain < 1) || (irGain > 100))
		{
			printf("\n Pick a Relevant IR Gain between 1 to 100: \t");
			scanf("%d", &irGain);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
		}
		for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
		{

			if (SetTOFIRGain(deviceHandleList[ctrlDeviceIndex], irGain) > 0)
			{
				cout << "TOF IR Gain is successfully set to : " << irGain << endl;
			}
			else
			{
				cout << "Setting TOF IR Gain value failed" << endl;
				return false;
			}
			if (exploreOpt == exploreOptions::singleCam)
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
bool tofCoringMenu()
{
	cout << '\t' << "0 - Exit" << endl;
	cout << '\t' << "1 - Back" << endl;
	cout << '\t' << "2 - Main Menu" << endl;
	cout << '\t' << "3 - Get TOF Coring" << endl;
	cout << '\t' << "4 - Set TOF Coring" << endl;
	int option = -1;
	uint16_t tofCoring = 0;
        UVCProp gProp;
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

		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}
			if (DeInitialize() > 0)
			{
				destroyAllWindows();
			}
		exit(0);

	case 1:
		return tofControlSettings();
		break;
	case 2:
		return exploreCam();
		break;

	case 3:
		PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:maxDeviceIndex : " + to_string(maxDeviceIndex)) ;
		for (uint64_t coringGetCtrlId = 0; coringGetCtrlId < deviceHandleList.size(); coringGetCtrlId++)
		{

                if (GetTOFCoring(deviceHandleList[coringGetCtrlId], &tofCoring) > 0)
                {
                        cout << "Current TOF Mask value of camera " << coringGetCtrlId << " is : " << tofCoring << endl;
                }
                else
                {
                        cout << "Getting TOF Mask value failed" << endl;
                        return false;
                }

			if (exploreOpt == exploreOptions::singleCam)
				break;
			else
				PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:exploreOpt != exploreOptions::singleCam		" + to_string(coringGetCtrlId) );

		}
		break;
	case 4:
        while ((tofCoring <= 0) || (tofCoring > 16383))
		{
            printf("\n Pick a Relevant Mask value between 1 to 16383: \t");
			scanf("%d", &tofCoring);
			while (getchar() != '\n' && getchar() != EOF)
			{
			}
		}
		for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
		{

			if (SetTOFCoring(deviceHandleList[ctrlDeviceIndex], tofCoring) > 0)
			{
				cout << "TOF Mask value is successfully set to : " << tofCoring  << " in camera "<< ctrlDeviceIndex << endl;
			}
			else
			{
				return false;
			}
			if (exploreOpt == exploreOptions::singleCam)
				break;
		}
	}
        return true;

}

/**
* @brief 		Setting TOF Controls to the Device
* @return		bool    return true on successfully setting the TOF Controls of the device, else retuns fail.
*/
bool tofControlSettings()
{
	cout << '\t' << "0 - Exit" << endl;
	cout << '\t' << "1 - Back" << endl;
	cout << '\t' << "2 - Main Menu" << endl;
	cout << '\t' << "3 - Depth Range" << endl;
	cout << '\t' << "4 - TOF Coring" << endl;
	cout << '\t' << "5 - IR Gain" << endl;
	cout << '\t' << "6 - IMU Embedded data" << endl;

	int option = -1;
  bool rt;
	while ((option < 0) || (option >= 7))
	{
		printf("\n Pick a Relevant TOF Control: \t");
		scanf("%d", &option);
		while (getchar() != '\n' && getchar() != EOF)
		{
		}
	}


	switch (option)
	{
    case EXIT:
        bSwitch = true;

		for (int deviceInd = 0; deviceInd < deviceHandleList.size() ; deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}

			if (DeInitialize() > 0)
			{
				destroyAllWindows();
			}
		exit(0);

	case 1:
	case 2:
		exploreCam();
		break;
	case 3:
		if (!depthRangeMenu())
		{
			cout << "Setting depth Range failed" << endl;
			return false;
		}
		break;
	case 4:
		if (!tofCoringMenu())
		{
			cout << "TOF Coring control failed" << endl;
			return false;
		}
		break;
	case 5:
		if (!irGainMenu())
		{
			cout << "TOF IR Gain control failed" << endl;
			return false;
		}
		break;
	case 6:
		if (!imuEmbedDataMenu())
		{
			cout << "IMU embed control failed" << endl;
			return false;
		}
	}
        return true;

}

/**
* @brief 		Spatial temporal and edge detection menu
* @return		bool    return true on successfully exploration, else retuns fail.
*/
bool filtersMenu(uint8_t filterID)
{
	cout << endl;
	cout << '\t' << "0 - Exit" << endl;
	cout << '\t' << "1 - Back" << endl;
	cout << '\t' << "2 - Main Menu" << endl;
	cout << '\t' << "3 - " << post_processing[filterID] << " OFF" << endl;
	cout << '\t' << "4 - " << post_processing[filterID] << " ON" << endl;

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

		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}
			if (DeInitialize() > 0)
			{
				destroyAllWindows();

			}
		exit(0);

	case 1:
		postProcessing();
		break;
	case 2:
		exploreCam();
		break;

	default:
		for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
		{

			if (SetFilterType(deviceHandleList[ctrlDeviceIndex], filterID, option - 3) < 0)
			{
				pError("SetFilterType");
				return false;
			}
			else
			{
				cout << "SetFilterType success "<< endl;

			}
			if (exploreOpt == exploreOptions::singleCam)
				break;
		}
		break;
	}
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

		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}
            if (DeInitialize() > 0)
            {
                destroyAllWindows();

            }
        exit(0);

	case 1:
		postProcessing();
		break;
	case 2:
		exploreCam();
		break;

    default:
		for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
		{

			if (planarizationState[ctrlDeviceIndex] != option - 3)
			{
				if (SetPlanarization(deviceHandleList[ctrlDeviceIndex], option - 3) < 0) {
					pError("SetPlanarization");
					return false;
				}
				planarizationState[ctrlDeviceIndex] = option - 3;
			}

		}
        break;
    }
    return true;
}


/**
* @brief 		Undistorting Depth
* @return		bool    return true on successfully Undistorting Depth, else retuns fail.
*/
bool depthUndistortion()
{
    cout << endl;
    cout << '\t' << "0 - Exit" << endl;
    cout << '\t' << "1 - Back" << endl;
    cout << '\t' << "2 - Main Menu" << endl;
    cout << '\t' << "3 - Undistortion OFF" << endl;
    cout << '\t' << "4 - Undistortion ON" << endl;

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

		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}
            if (DeInitialize() > 0)
            {
                destroyAllWindows();

            }
        exit(0);

	case 1:
		postProcessing();
		break;
	case 2:
		exploreCam();
		break;

    default:
		for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
		{
			PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:ctrlDeviceIndex for SetUnDistortion : " + to_string(ctrlDeviceIndex) );

			if (undistortionState[ctrlDeviceIndex] != option - 3)
			{
				if (SetUnDistortion(deviceHandleList[ctrlDeviceIndex], option - 3) < 0) {
					pError("SetUndistortion");
					return false;
				}
				undistortionState[ctrlDeviceIndex] = option - 3;
			}

		}
        break;
    }
    return true;
}

/**
* @brief 		Exploring the post processing features
* @return		int		1 on successfull exploration of post processing filters.
						0 on fail
*/
int postProcessing()
{
	cout << endl;
	cout << '\t' << "0 - Exit" << endl;
	cout << '\t' << "1 - Back" << endl;
    cout << '\t' << "2 - Main Menu" << endl;
	cout << '\t' << "3 - Planarization" << endl;
	cout << '\t' << "4 - Spatial Filter" << endl;
	cout << '\t' << "5 - Temporal Filter" << endl;
	cout << '\t' << "6 - Edge detection" << endl;
	cout << '\t' << "7 - Undistort depth" << endl;

	int option = -1;
	while ((option < 0) || (option >= 8))
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

		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}
			if (DeInitialize() > 0)
			{
				destroyAllWindows();

			}
		exit(0);

	case 1:
	case 2:
		exploreCam();
		break;

	case 3:

			if (!depthPlanarization())
			{
				cout << endl << "Depth Planarization Failed" << endl;
				return 0;
			}
			else
			{
				return 1;
			}
		break;
	case 4:
	case 5:
	case 6:

			if (!filtersMenu(option - 4))
			{
				cout << "filtersMenu failed" << endl;
				return 0;
			}
			else
			{
				return 1;
			}

		break;
	case 7:
			if (!depthUndistortion())
			{
				cout << endl << "Depth Undistortion Failed" << endl;
			}
			else
			{
				return 1;
			}
		break;
	}

	return 1;
}

/**
* @brief 		Mapping Depth to RGB Depth
* @return		bool    return true on successfully RGB-D mapping, else retuns fail.
*/
bool rgbdMapping()
{
	std::cout << endl;
	std::cout << '\t' << "0 - Exit" << endl;
	std::cout << '\t' << "1 - Back" << endl;
	std::cout << '\t' << "2 - Main Menu" << endl;
	std::cout << '\t' << "3 - RGB-D Mapping OFF" << endl;
	std::cout << '\t' << "4 - RGB-D Mapping ON" << endl;

	int option = -1;
	int returnVal;
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
		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
			if (bDetach)
			{
				if (streamThread[deviceInd].joinable())
				{
					streamThread[deviceInd].detach();
				}
				if (previewThread[deviceInd].joinable())
				{
					previewThread[deviceInd].detach();
				}
			}
#endif
			if (CloseDevice(deviceHandleList[deviceInd]) > 0)
			{
			}
		}
		if (DeInitialize() > 0)
		{
			destroyAllWindows();

		}
		exit(0);

	case 1:
	case 2:
		exploreCam();
		break;

	default:
		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{

				for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
				{
					bSwitch = true;
					bPreviewSet(1, false, ctrlDeviceIndex);
					streamStarted = false;

					PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:ctrlDeviceIndex for SetUnDistortion : " + to_string(ctrlDeviceIndex));
					returnVal = SetRGBDMapping(deviceHandleList[ctrlDeviceIndex], option - 3);
					if (returnVal < 0 && returnVal != Result::RGB_DCalibNotFound) {
						pError("SetRGBDMapping");
						startStream();
						return false;
					}
					else if (returnVal == Result::RGB_DCalibNotFound) {
						std::cout << endl << "RGBD Calibration data not found. RGB to Depth Mapping failed" << endl;
						startStream();
						return false;
					}
					if (option - 3)
						rgbdMappingflag[ctrlDeviceIndex] = true;
					else
						rgbdMappingflag[ctrlDeviceIndex] = false;
				}
				startStream();

			break;
		}
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
    int length_depth;
    unsigned char* buffer;
    //FILE* fp;
    FILE* fp_rgb;
    FILE* fp_extr;
    uint16_t err_no;
    int read_length;
	int calibrationReadStatus = 0;
	bool closeDeviceRes = false;

    while (true)
    {
        int choice = -1;
		PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:before explore options exploreOpt : " + to_string(exploreOpt));
		if (exploreOpt == exploreOptions::singleCam)
		{
			cout << endl << '\t' << "0   - Exit" << endl;
			cout << '\t' << "1   - Back" << endl;
			cout << '\t' << "2   - Streaming Mode" << endl;
			cout << '\t' << "3   - TOF Controls" << endl;
			cout << '\t' << "4   - UVC Controls" << endl;
			cout << '\t' << "5   - RGB-D Mapping" << endl;
			cout << '\t' << "6   - Post Processing" << endl;
			cout << '\t' << "7   - Capture Frames" << endl;
			cout << '\t' << "8   - Unique ID" << endl;
			cout << '\t' << "9   - Read Firmware Version" << endl;
			cout << '\t' << "10   - Get Depth value" << endl;
			while (1)
			{
				printf("\n Pick a Relevant Choice of Camera Properties : \t");
				scanf("%d", &choice);
				while (getchar() != '\n' && getchar() != EOF)
				{
				}
				if ((choice >= 0) && (choice < 11))
				{
					break;
				}
			}
		}
		else if (exploreOpt == exploreOptions::multipleCam)
		{
			cout << endl << '\t' << "0   - Exit" << endl;
			cout << '\t' << "1   - Streaming Mode" << endl;
			cout << '\t' << "2   - TOF Controls" << endl;
			cout << '\t' << "3   - UVC Controls" << endl;
			cout << '\t' << "4   - RGB-D Mapping" << endl;
			cout << '\t' << "5   - Post Processing" << endl;
			cout << '\t' << "6   - Capture Frames" << endl;
			cout << '\t' << "7   - Unique ID" << endl;
			cout << '\t' << "8   - Read Firmware Version" << endl;
			cout << '\t' << "9   - Get Depth value" << endl;
			while (1)
			{
				printf("\n Pick a Relevant Choice of Camera Properties : \t");
				scanf("%d", &choice);
				while (getchar() != '\n' && getchar() != EOF)
				{
				}
				if((choice >= 0) && (choice < 10))
				{
					break;
				}
			}
			if (choice != 0)
				choice++;
		}



        switch (choice)
        {
    case EXIT:
        bSwitch = true;

		for (int deviceInd = 0; deviceInd < deviceHandleList.size(); deviceInd++)
		{
			bPreviewSet(1, false, deviceInd);
			previewThreadCtrlFlag[deviceInd] = false;
			streamThreadCtrlFlag[deviceInd] = false;
#ifdef _WIN32
		if (bDetach)
		{
			if (streamThread[deviceInd].joinable())
			{
				streamThread[deviceInd].detach();
			}
			if (previewThread[deviceInd].joinable())
			{
				previewThread[deviceInd].detach();
			}
		}
#endif
		}
			for (int itr = 0; itr < deviceHandleList.size(); itr++)
			{

        Result closeDevRes = CloseDevice(deviceHandleList[itr]) ;
				if (closeDevRes > 0)
				{
					closeDeviceRes = true;
				}
        else
        {
          std::cout << "Close Device failed with ret : " << closeDevRes <<"\n";
        }

			}
			if (closeDeviceRes == true)
			{
				PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:CloseDevice success\n");
				if (DeInitialize() > 0)
				{
					PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole: DeInitialize success\n");
					destroyAllWindows();
					PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole: destroyAllWindows success\n");
				}
			}

			exit(0);

        case 1:
            if (!listDevices())
            {
                cout << endl << "List Devices Information failed in explore cam\n" << endl;
            }
            cout << endl << "Connected Devices were Listed" << endl;
            break;

        case 2:
            if (!selectStreamingMode())
            {
                cout << endl << "Data Mode Selection Failed" << endl;
            }
            cout << endl << "Data Mode Selected" << endl;
            break;

        case 3:
            if (!tofControlSettings())
            {
                cout << endl << "TOF Controls selection Failed" << endl;
            }
            cout << endl << "TOF Controls Selected" << endl;
            break;

        case 4:
            if (!uvcControlMenu())
            {
                cout << endl << "UVC controls Selection Failed" << endl;
            }
            cout << endl << "UVC controls Selected" << endl;
            break;

        case 5:
            if (!rgbdMapping())
            {
                cout << endl << "RGB-D Mapping Failed" << endl;
            }
            break;

		case 6:
            if (!postProcessing())
            {
                cout << endl << "Depth Planarization Failed" << endl;
            }
            break;

        case 7:
			if (bPreviewSet(3, true, 0))
			{
				memset(depthFrameFileNameBuf, 0, MAX_FILENAME);
				memset(RGBFrameFileNameBuf, 0, MAX_FILENAME);
				memset(IRFrameFileNameBuf, 0, MAX_FILENAME);
				memset(depthRawFrameFileNameBuf, 0, MAX_FILENAME);
				memset(PLY3DFileNameBuf, 0, MAX_FILENAME);
				saveFrames[0] = true;
#ifdef _WIN32
				while (1)
				{
					DWORD dWait = WaitForSingleObject(saveEvent[0], 1000);
					if (dWait != WAIT_TIMEOUT)
					{
						break;
					}
				}

#elif __linux__
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
#endif
				if (exploreOpt != 0)
				{
          for (int itr = 1; itr < deviceHandleList.size(); itr++)
          {
					saveFrames[itr] = true;
#ifdef _WIN32
					while (1)
					{
						DWORD dWait = WaitForSingleObject(saveEvent[1], 1000);
						if (dWait != WAIT_TIMEOUT)
						{
							break;
						}
					}
#elif __linux__
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
#endif
				}
      }
            }
            else
                cout << endl << "Image Capture Failed, Please start preview before capturing frames" << endl;
            cout << endl << "Frame Capture ends" << endl;

            break;

        case 8:
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{
				GetUniqueID(deviceHandleList[ctrlDeviceIndex], &uniqueID);
				cout << endl << "Unique ID of the Camera " << ctrlDeviceIndex <<" is " << uniqueID << endl;
			}
            break;

        case 9:
			for (int ctrlDeviceIndex = 0; ctrlDeviceIndex < deviceHandleList.size(); ctrlDeviceIndex++)
			{
				ReadFirmwareVersion(deviceHandleList[ctrlDeviceIndex], &gMajorVersion, &gMinorVersion1, &gMinorVersion2, &gMinorVersion3);
				cout << endl << "Firmware Version in camera " << ctrlDeviceIndex << "  : " << (uint16_t)gMajorVersion << "." << (uint16_t)gMinorVersion1 << "." << gMinorVersion2 << "." << gMinorVersion3 << endl;
			}
            break;

        case 10:
            if(Depthstreamstarted)
            {
                if (!selectGetDepthValue())
                {
                    cout << endl << "Get Depth value Failed" << endl;
                }
            }
            else
            {
                cout<<endl<<" Start Depth stream to get depth value"<<endl;
            }
            break;
        }
    }

    return true;
}


void registerNotificationCb(int id, int index)
{
  bSwitch = true;

  for(int itr = 0 ; itr < deviceHandleList.size()  ; itr++)
  {
	  bPreviewSet(1, false, itr);
	  streamStarted = false;
    streamThreadCtrlFlag[itr] = false;
	previewThreadCtrlFlag[itr] = false;
#ifdef _WIN32
	if (bDetach)
	{
		if (streamThread[itr].joinable())
		{
			streamThread[itr].detach();
		}
		if (previewThread[itr].joinable())
		{
			previewThread[itr].detach();
		}
	}
#endif
  }

  if(id == 0)
  {
    CloseDevice(deviceHandleList[index]);
  }
  deviceHandleList.erase(deviceHandleList.begin() +  index);
  rgbdMappingflag.erase(rgbdMappingflag.begin() + index);
  destroyAllWindows();


  for(int itr = 0 ; itr < deviceHandleList.size()  ; itr++)
  {
    tempDeviceIndex[itr] = itr;
    streamThreadCtrlFlag[itr] = true;
  	previewThreadCtrlFlag[itr] = true;
#ifdef _WIN32
	PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:Before creating thread\n");

	previewThread[itr] = thread(&preview, itr);
	streamThread[itr] = thread(&stream, itr);

	PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:After creating thread\n");
#elif __linux__
            pthread_create(&streamThread[itr], NULL, stream, &tempDeviceIndex[itr]);
            pthread_create(&previewThread[itr], NULL, preview, &tempDeviceIndex[itr]);
#endif
	}
#if 1
  maxDeviceIndex--;

  if(deviceHandleList.size() >= 1)
    startStream();
  else
  {
    	//Open a Camera Device
    	if (!(listDevices()))
    	{
    		cout << endl << "List Devices Information Failed in register notification\n" << endl;
			exit(0);
    	}
    	PrintLog(LOG_HIGH_DEBUG, "\nDepthVistaConsole:Before explore cam" );

        if (!(exploreCam()))
        {
            cout << endl << "Camera Exploration Failed" << endl;
        }
  }
#endif

}

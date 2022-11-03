///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2022, e-Con Systems.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
// ANY DIRECT/INDIRECT DAMAGES HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
#pragma once
#ifdef  __linux__
#include <signal.h>
#include <math.h>
#include <unistd.h>
#elif _WIN32
#include <windows.h>
#include "strsafe.h"
#include <conio.h>
#endif
#include <math.h>
#include "DepthVistaSDK/DepthVista.h"
#include "opencv2/opencv.hpp"

#define		M_PI				3.14159265358979323846
#define		HALF_PI				(M_PI / 2)
#define		DEG2RAD				(M_PI / 180.f)
#define		RAD2DEG				(180.f / M_PI)

#define DEBUG_ENABLED (1)
#define RE_ENUMERATE (1)
/* IMU VALUES CONTROL */
#define IMU_AXES_VALUES_MIN					(1)
#define IMU_AXES_VALUES_MAX					(65535)

/* IMU VALUE UPDATE MODE */
#define IMU_CONT_UPDT_EN					 (0x01)
#define IMU_CONT_UPDT_DIS					 (0x02)

/* IMU MODE */
#define IMU_ACC_GYRO_DISABLE (uint8_t)(0x00)
#define IMU_ACC_ENABLE (uint8_t)(0x01)
#define IMU_GYRO_ENABLE (uint8_t)(0x02)
#define IMU_ACC_GYRO_ENABLE (uint8_t)(0x03)

/* ACC AXIS CONTROL */
#define IMU_ACC_X_Y_Z_ENABLE (uint8_t)(0x07)
#define IMU_ACC_X_ENABLE (uint8_t)(0x04)
#define IMU_ACC_Y_ENABLE (uint8_t)(0x02)
#define IMU_ACC_Z_ENABLE (uint8_t)(0x01)

/* ACC SENSITIVITY CONTROL */
#define IMU_ACC_SENS_2G (uint8_t)(0x00)
#define IMU_ACC_SENS_4G (uint8_t)(0x02)
#define IMU_ACC_SENS_8G (uint8_t)(0x03)
#define IMU_ACC_SENS_16G (uint8_t)(0x01)

/* ODR CONTROL */
#define IMU_ODR_12_5HZ (uint8_t)(0x01)
#define IMU_ODR_26HZ (uint8_t)(0x02)
#define IMU_ODR_52HZ (uint8_t)(0x03)
#define IMU_ODR_104HZ (uint8_t)(0x04)
#define IMU_ODR_208HZ (uint8_t)(0x05)
#define IMU_ODR_416HZ (uint8_t)(0x06)
#define IMU_ODR_833HZ (uint8_t)(0x07)
#define IMU_ODR_1666HZ (uint8_t)(0x08)

/* GYRO AXIS CONTROL */
#define IMU_GYRO_X_Y_Z_ENABLE (uint8_t)(0x07)
#define IMU_GYRO_X_ENABLE (uint8_t)(0x04)
#define IMU_GYRO_Y_ENABLE (uint8_t)(0x02)
#define IMU_GYRO_Z_ENABLE (uint8_t)(0x01)

/* GYRO SENSITIVITY CONTROL */
#define IMU_GYRO_SENS_250DPS (uint8_t)(0x00)
#define IMU_GYRO_SENS_500DPS (uint8_t)(0x01)
#define IMU_GYRO_SENS_1000DPS (uint8_t)(0x02)
#define IMU_GYRO_SENS_2000DPS (uint8_t)(0x03)

class IMU_Sample
{
public:

	//Constructor
	IMU_Sample(void);

	//Initialises the variables
	int Init();

	// Function declarations
	void getInclination(double w_x, double w_y, double w_z, double a_x, double a_y, double a_z);
	int enumerate_again;

private:
	//TOF camera enumeration
	DeviceInfo* gDevicesList;

	double angleX, angleY, angleZ; // Rotational angle for cube [NEW]
	double RwEst[3];
	double glIMU_Interval;

	//TaraRev g_eRev;

	double squared(double x);

	/* Drawing Points in circles for illustration*/
	cv::Point PointOnCircle(double radius, double angleInDegrees, cv::Point origin);

	/* Drawing angles in circles for illustration */
	void updateCircles();

	/*
	 *  Name		:	GetIMUIntervalTime											   *
	 *  Returns		:	Interval time in float										   *
	 *  Description	:   Returns the interval time for sampling the values of the IMU.  *
	*/
	double GetIMUIntervalTime(IMUCONFIG_TypeDef	lIMUConfig);


}IMU_SampleObj;

IMUDATAINPUT_TypeDef	glIMUInput;
bool glIMUAbortThread;
#ifdef _WIN32
HANDLE	IMUDataReadyEvent;

//WINAPI
DWORD WINAPI GetIMUValueThread(void* lpParameter);

DWORD WINAPI UpdateIMUValueThread(void* lpParameter);
#elif __linux__

pthread_mutex_t				    IMUDataReadyEvent;

//Pthread call routine
void*	GetIMUValueThread(void *lpParameter);

void*	UpdateIMUValueThread(void *lpParameter);
#endif



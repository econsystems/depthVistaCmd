// TOF_IMU_Application.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "DepthVistaIMU.h"

using namespace cv;
using namespace std;

/*
  **********************************************************************************************************
 *  Name		:	GetIMUIntervalTime											   *
 *  Returns		:	Interval time in float										   *
 *  Description	:   Returns the interval time for sampling the values of the IMU.  *
  **********************************************************************************************************
*/
double IMU_Sample::GetIMUIntervalTime(IMUCONFIG_TypeDef	lIMUConfig)
{

    double lIMUIntervalTime = 10;

    if (lIMUConfig.IMU_MODE == IMU_ACC_GYRO_ENABLE)
    {
        switch (lIMUConfig.IMU_ODR_CONFIG)
        {
        case IMU_ODR_12_5HZ:
            lIMUIntervalTime = 1000.00 / 12.5;
            break;

        case IMU_ODR_26HZ:
            lIMUIntervalTime = 1000.00 / 26.0;
            break;

        case IMU_ODR_52HZ:
            lIMUIntervalTime = 1000.00 / 52.0;
            break;

        case IMU_ODR_104HZ:
            lIMUIntervalTime = 1000.00 / 104.0;
            break;

        case IMU_ODR_208HZ:
            lIMUIntervalTime = 1000.00 / 208.0;
            break;

        case IMU_ODR_416HZ:
            lIMUIntervalTime = 1000.00 / 416.00;
            break;

        case IMU_ODR_833HZ:
            lIMUIntervalTime = 1000.00 / 833.00;
            break;

        case IMU_ODR_1666HZ:
            lIMUIntervalTime = 1000.00 / 1666.00;
            break;
        }
    }
    return lIMUIntervalTime;
}

/* Initialises all the values */
IMU_Sample::IMU_Sample(void)
{
    glIMUAbortThread = false;
    glIMU_Interval = 0.0f;

    angleX = 0.0f;
    angleY = 0.0f;
    angleZ = 0.0f;
}

/* Square of a number */
double IMU_Sample::squared(double x)
{
    return x * x;
}

/* Computes the angle of rotation with respect to the axes */
void IMU_Sample::getInclination(double g_x, double g_y, double g_z, double a_x, double a_y, double a_z)
{
    int w = 0;
    double tmpf = 0.0;
    int signRzGyro;
    static bool firstSample = true;
    double wGyro = 10.0;
    double norm;

    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;

    double RwAcc[3] = { a_x, a_y, a_z };
    double RwGyro[3] = { g_x, g_y, g_z };
    double Awz[2];
    double Gyro[3];

    if (firstSample)
    {
        //initialize with accelerometer readings
        for (w = 0; w <= 2; w++)
        {
            RwEst[w] = RwAcc[w];
        }
    }
    else
    {
        //evaluate Gyro vector
        if (fabs(RwEst[2]) < 0.1)
        {
            //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
            //in this case skip the gyro data and just use previous estimate
            for (w = 0; w <= 2; w++)
            {
                Gyro[w] = RwEst[w];
            }
        }
        else {
            //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
            for (w = 0; w <= 1; w++)
            {
                tmpf = RwGyro[w];                        //get current gyro rate in deg/s
                tmpf *= glIMU_Interval / 1000.0f;                     //get angle change in deg
                Awz[w] = atan2(RwEst[w], RwEst[2]) * RAD2DEG;   //get angle and convert to degrees
                Awz[w] += tmpf;             //get updated angle according to gyro movement
            }

            //estimate sign of RzGyro by looking in what qudrant the angle Axz is,
            //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
            signRzGyro = (cos(Awz[0] * DEG2RAD) >= 0) ? 1 : -1;

            //reverse calculation of Gyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
            Gyro[0] = sin(Awz[0] * DEG2RAD);
            Gyro[0] /= sqrt(1 + squared(cos(Awz[0] * DEG2RAD)) * squared(tan(Awz[1] * DEG2RAD)));
            Gyro[1] = sin(Awz[1] * DEG2RAD);
            Gyro[1] /= sqrt(1 + squared(cos(Awz[1] * DEG2RAD)) * squared(tan(Awz[0] * DEG2RAD)));
            Gyro[2] = signRzGyro * sqrt(1 - squared(Gyro[0]) - squared(Gyro[1]));
        }

        //combine Accelerometer and gyro readings
        for (w = 0; w <= 2; w++)
            RwEst[w] = (RwAcc[w] + wGyro * Gyro[w]) / (1 + wGyro);

        //Normalizing the estimates
        norm = sqrt(RwEst[0] * RwEst[0] + RwEst[1] * RwEst[1] + RwEst[2] * RwEst[2]);
        RwEst[0] /= norm;
        RwEst[1] /= norm;
        RwEst[2] /= norm;
    }

    firstSample = false;

    //Computing the angles
    angleX = RwEst[0] * HALF_PI * RAD2DEG;
    angleY = RwEst[1] * HALF_PI * RAD2DEG;
    angleZ = RwEst[2] * HALF_PI * RAD2DEG;
}

/* Drawing Points in circles for illustration */
Point IMU_Sample::PointOnCircle(double radius, double angleInDegrees, Point origin)
{
    //radius -> Radius of Circle & Origin -> Circle Centre.
    // Convert from degrees to radians via multiplication by PI/180
    double x = (radius * cos(angleInDegrees * DEG2RAD)) + origin.x;
    double y = (radius * sin(angleInDegrees * DEG2RAD)) + origin.y;

    return Point((int)x, (int)y);
}

/* Drawing angles in circles for illustration */
void IMU_Sample::updateCircles()
{
    Mat drawImage = Mat::zeros(Size(800, 400), CV_8UC3);

    //Static Labelling.
    putText(drawImage, "x", Point(150, 50), FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(0, 0, 255), 1);
    putText(drawImage, "y", Point(400, 50), FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(0, 255, 0), 1);
    putText(drawImage, "z", Point(650, 50), FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(255, 0, 0), 1);

    //Labelling the angles.
    stringstream ss;
    ss << (int)angleX << " deg";
    putText(drawImage, ss.str(), Point(150, 350), FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 0, 255), 1);
    ss.str("");

    ss << (int)angleY << " deg";
    putText(drawImage, ss.str(), Point(400, 350), FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 0), 1);
    ss.str("");

    ss << (int)angleZ << " deg";
    putText(drawImage, ss.str(), Point(650, 350), FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 0, 0), 1);

    //Drawing x angles.
    Point P1 = PointOnCircle(100, -angleX + 180, Point(150, 200));
    Point P2 = PointOnCircle(100, -angleX, Point(150, 200));

    ellipse(drawImage, Point(150, 200), Size(100, 100), 0, -angleX, -angleX - 180, Scalar(20, 20, 20), -1);
    ellipse(drawImage, Point(150, 200), Size(100, 100), 0, -angleX, -angleX + 180, Scalar(0, 0, 255), -1);
    circle(drawImage, P1, 2, Scalar(0, 0, 255), 2);
    circle(drawImage, P2, 2, Scalar(0, 0, 255), 2);
    line(drawImage, P1, P2, Scalar(0, 0, 255), 2);

    //Drawing y angles.
    P1 = PointOnCircle(100, -angleY + 180, Point(400, 200));
    P2 = PointOnCircle(100, -angleY, Point(400, 200));

    ellipse(drawImage, Point(400, 200), Size(100, 100), 0, -angleY, -angleY - 180, Scalar(20, 20, 20), -1);
    ellipse(drawImage, Point(400, 200), Size(100, 100), 0, -angleY, -angleY + 180, Scalar(0, 255, 0), -1);
    circle(drawImage, P1, 2, Scalar(0, 255, 0), 2);
    circle(drawImage, P2, 2, Scalar(0, 255, 0), 2);
    line(drawImage, P1, P2, Scalar(0, 255, 0), 2);


    //Drawing z angles.
    P1 = PointOnCircle(100, -angleZ + 180, Point(650, 200));
    P2 = PointOnCircle(100, -angleZ, Point(650, 200));

    ellipse(drawImage, Point(650, 200), Size(100, 100), 0, -angleZ, -angleZ - 180, Scalar(20, 20, 20), -1);
    ellipse(drawImage, Point(650, 200), Size(100, 100), 0, -angleZ, -angleZ + 180, Scalar(255, 0, 0), -1);
    circle(drawImage, P1, 2, Scalar(255, 0, 0), 2);
    circle(drawImage, P2, 2, Scalar(255, 0, 0), 2);
    line(drawImage, P1, P2, Scalar(255, 0, 0), 2);

    imshow("Inclination", drawImage);
    waitKey(1);
}

#ifdef __linux__

/* Killing the thread */
void KillThread(int sig)
{
    pthread_exit(0);
}

/* UpdateIMUValue Thread calls the HID command and value updation */
void* UpdateIMUValueThread(void *lpParameter)
{
    IMUDATAOUTPUT_TypeDef *lIMUOutputAdd, *lIMUOutput = NULL;
    lIMUOutputAdd = lIMUOutput = (IMUDATAOUTPUT_TypeDef*)(lpParameter);
    signal(SIGUSR1, KillThread);

    //Blocking call waits for unlock event
    for (; ((glIMUAbortThread == false) && ((glIMUInput.imu_update_mode == IMU_CONT_UPDT_EN) ||
        (lIMUOutput->imu_value_id <= glIMUInput.imu_num_of_values))); )
    {
        if (glIMUInput.imu_update_mode != IMU_CONT_UPDT_DIS)
        {
            if (GetIMUValue(lIMUOutput))
            {
                usleep(1000);
                //Calculating angles based on the current raw values from IMU
                IMU_SampleObj.getInclination(lIMUOutput->gyroX, lIMUOutput->gyroY, lIMUOutput->gyroZ,
                    lIMUOutput->accX, lIMUOutput->accY, lIMUOutput->accZ);
                lIMUOutput = lIMUOutputAdd;
            }
        }
    }
    return NULL;
}
#elif _WIN32
/* UpdateIMUValue Thread uses the values from the GetIMUValue everytime when the semaphore is set */
DWORD WINAPI UpdateIMUValueThread(void* lpParameter)
{
    IMUDATAOUTPUT_TypeDef* lIMUOutputAdd, *lIMUOutput = NULL;
    lIMUOutputAdd = lIMUOutput = static_cast<IMUDATAOUTPUT_TypeDef*>(lpParameter);

    for (; ((glIMUAbortThread == FALSE) && ((glIMUInput.imu_update_mode == IMU_CONT_UPDT_EN) ||
        (lIMUOutput->imu_value_id <= glIMUInput.imu_num_of_values))); )
    {
        if (glIMUInput.imu_update_mode != IMU_CONT_UPDT_DIS)
        {
            if (GetIMUValue(lIMUOutput))
            {
                Sleep(1);
                IMU_SampleObj.getInclination(lIMUOutput->gyroX, lIMUOutput->gyroY, lIMUOutput->gyroZ,
                    lIMUOutput->accX, lIMUOutput->accY, lIMUOutput->accZ);
            }
        }
        //Round robin mechanism to use the same buffer
        if (lIMUOutput->imu_value_id < IMU_AXES_VALUES_MAX)
            lIMUOutput++;
        else
            lIMUOutput = lIMUOutputAdd;
    }
    return WAIT_OBJECT_0;
}
#endif
//Init the camera and read the values
int IMU_Sample::Init()
{
    std::cout << endl << "		IMU Sample Application " << endl << endl;
    std::cout << " Application to illustrate the IMU unit ICM-20789 integrated with DepthVista Camera" << endl;
    std::cout << " Demonstrating the rotations of camera around x-axis and y-axis " << endl;
    std::cout << " IMU values are limited from -90 to +90 degrees for illustration " << endl << endl;

    IMUDATAINPUT_TypeDef lIMUInput;
    IMUCONFIG_TypeDef lIMUConfig;
    uint8_t uStatus = 0;
    uint32_t count = 0;
    uint32_t device_to_open = 0;
    while (1)
    {
        //Initialize the camera
        if (Initialize() < 0) {
            pError();
        }

        //Get the Nunmber of devices connected
        if (GetDeviceCount(&count) < 0) {
            pError("GetDeviceCount");

        }
        if (count > 0) {
            std::cout << "\r\nNumber of devices connected : " << count << "\r\n";
            gDevicesList = new DeviceInfo[count];
            if (GetDeviceListInfo(count, gDevicesList) < 0) {
                pError("GetDeviceListInfo");
            }
            for (int i = 0; i < count; i++) {
                std::string devicename = gDevicesList[i].deviceName;
                std::cout << "\r\n Device Id : " << i + 1 << ", Device Name : " << gDevicesList[i].deviceName << "\r\n";
            }
            break;
        }
        else if (count == 0)
        {
            DeInitialize();
            std::cout << "\n0 : Exit\r\n";
            std::cout << "\n1 : Enumerate Again\r\n";
            std::cin >> enumerate_again;
            if (enumerate_again == RE_ENUMERATE)
            {
                continue;
            }
            else
            {
                return 0;
            }
        }
    }

    std::cout << "\r\nEnter the device ID to proceed\r\n";
    std::cin >> device_to_open;
    while (1)
    {
        if (device_to_open > count && device_to_open <= 0)
        {
            std::cout << "Invalid device ID, Please select Valid one\r\n";
            std::cin >> device_to_open;
            continue;
        }
        else
        {
            std::cout << "\r\nDevice ID Selected is " << device_to_open << "\r\n";
            //Open the selected Camera
            uStatus = OpenDevice(device_to_open - 1);
            if (!uStatus)
            {
                pError("OpenDevice:");
                return 1;
            }
            break;
        }
    }
    if (uStatus)
    {
        //Configuring IMU rates.
        lIMUConfig.IMU_MODE = IMU_ACC_GYRO_ENABLE;
        lIMUConfig.ACC_AXIS_CONFIG = IMU_ACC_X_Y_Z_ENABLE;
        lIMUConfig.IMU_ODR_CONFIG = IMU_ODR_104HZ;
        lIMUConfig.ACC_SENSITIVITY_CONFIG = IMU_ACC_SENS_2G;
        lIMUConfig.GYRO_AXIS_CONFIG = IMU_GYRO_X_Y_Z_ENABLE;
        lIMUConfig.GYRO_SENSITIVITY_CONFIG = IMU_GYRO_SENS_250DPS;

#ifdef __linux__
        usleep(500 * 1000);
#elif _WIN32
        Sleep(500);
#endif
        //Setting the configuration using HID command
        uStatus = SetIMUConfig(lIMUConfig);
        if (!uStatus)
        {
            std::cout << "Set IMU Config Failed\r\n";
            return 1;
        }
        else
        {
            //To enable after fixing GetIMUConfig in firmware

            //Reading the configuration to verify the values are set
            uStatus = GetIMUConfig(&lIMUConfig);
            if (!uStatus)
            {
                std::cout << "Get IMU Config Failed\r\n";
                return 0;
            }
            //Finding the sampling interval time
            glIMU_Interval = GetIMUIntervalTime(lIMUConfig); //Only gyro and acc enabled are used

        }
    }

    //Configuring IMU update mode.
    lIMUInput.imu_update_mode = IMU_CONT_UPDT_EN;
    lIMUInput.imu_num_of_values = IMU_AXES_VALUES_MIN;

    //Setting the IMU update mode using HID command
    uStatus = ControlIMUCapture(&lIMUInput);
    if (uStatus == false)
    {
        std::cout << "Control IMU Capture  Failed\r\n";
        return 1;
    }
    else
    {
        glIMUInput = lIMUInput;
    }
#ifdef _WIN32
    //Getting the IMU values.
    DWORD lThreadID;
    HANDLE lPrintHandle;
    //BOOL lPrintThreadStatus = false;

#elif __linux__
    //Getting the IMU values
    pthread_t updateIMUThread;
#endif
    IMUDATAOUTPUT_TypeDef* lIMUOutput = NULL;

    //Allocating buffers for output structure
    if (glIMUInput.imu_update_mode == IMU_CONT_UPDT_EN)
    {
        lIMUOutput = (IMUDATAOUTPUT_TypeDef*)malloc(IMU_AXES_VALUES_MAX * sizeof(IMUDATAOUTPUT_TypeDef));
    }
    else
    {
        lIMUOutput = (IMUDATAOUTPUT_TypeDef*)malloc(1 * sizeof(IMUDATAOUTPUT_TypeDef));
    }

    //Memory validation
    if (lIMUOutput == NULL)
    {
        std::cout << "Memory Allocation for output failed\n";
        return 1;
    }

    lIMUOutput->imu_value_id = 0;

#ifdef _WIN32
    std::cout << "\nPress any key to stop!!!\n";
#elif __linux__
    std::cout << "\nUse keyboard interupt (ctrl+c) to stop\n";
#endif

#ifdef _WIN32

    lPrintHandle = CreateThread(0, 0, UpdateIMUValueThread, (void*)lIMUOutput, 0, &lThreadID);

    for (; (!_kbhit() && (glIMUInput.imu_update_mode != IMU_CONT_UPDT_DIS));)
    {
        //Wait untill inclination data is calculated from received IMU data
        //if (!lPrintThreadStatus)
        //{
            //lAPIStatus = WaitForSingleObject(lPrintHandle, 0);
            //if (lAPIStatus == WAIT_FAILED)
            //{
            //	std::cout << "Print thread Error!\r\n";
            //	break;
            //}
            //else if (lAPIStatus == WAIT_OBJECT_0)
            //{
            //	lPrintThreadStatus = TRUE;
            //}
        //}
        updateCircles();
        Sleep(1);
    }

#elif __linux__
    //Thread creation
    if (pthread_create(&updateIMUThread, NULL, UpdateIMUValueThread, (void*)lIMUOutput) != 0)
    {
        cout << "Update IMU value thread creation failed\n";
        return false;
    }

    //Calling function
    for (; (glIMUInput.imu_update_mode != IMU_CONT_UPDT_DIS);)
    {
        updateCircles();
        usleep(1);
    }
    glIMUAbortThread = true;
#endif


    lIMUInput.imu_update_mode = IMU_CONT_UPDT_DIS;
    lIMUInput.imu_num_of_values = IMU_AXES_VALUES_MIN;

    //Resetting the IMU update to disable mode
    uStatus = ControlIMUCapture(&lIMUInput);
    if (uStatus == false)
    {
        std::cout << "Control IMU Capture Failed\r\n";
        return 1;
    }
#ifdef  _WIN32

    WaitForSingleObject(lPrintHandle, 1000);
    TerminateThread(lPrintHandle, 0);
    CloseHandle(lPrintHandle);
    glIMUAbortThread = false;
#elif __linux__
    //Releasing the threads and mutex
    pthread_kill(updateIMUThread, SIGUSR1);
    glIMUAbortThread = false;

#endif
    //Freeing the memory
    if (lIMUOutput)
        free(lIMUOutput);

    CloseDevice();
    return 0;
}

int main()
{
    int ReturnStatus = -1;

    //Initialise the methods
    ReturnStatus = IMU_SampleObj.Init();

    if (ReturnStatus) //check for a valid return
    {
        cout << endl << "Exit: IMU Sample Application" << endl << endl;
#ifdef __linux__
        usleep(500 * 1000);
#elif _WIN32
        Sleep(500);
#endif
    }
    return 1;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started:
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

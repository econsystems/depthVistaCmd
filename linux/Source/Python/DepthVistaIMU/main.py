import cv2
import ctypes
from ctypes import*
from input import get_integer
import threading
import numpy as np
import time
import sys
import math

M_PI = 3.14159265358979323846
HALF_PI = (M_PI / 2)
DEG2RAD	= (M_PI / 180)
RAD2DEG	= (180 / M_PI)

IMU_ODR_12_5HZ = ctypes.c_uint8(0x01)
IMU_ODR_26HZ = ctypes.c_uint8(0x02)
IMU_ODR_52HZ = ctypes.c_uint8(0x03)
IMU_ODR_104HZ = ctypes.c_uint8(0x04)
IMU_ODR_208HZ = ctypes.c_uint8(0x05)
IMU_ODR_416HZ = ctypes.c_uint8(0x06)
IMU_ODR_833HZ = ctypes.c_uint8(0x07)
IMU_ODR_1666HZ = ctypes.c_uint8(0x08)
IMU_CONT_UPDT_DIS = (0x02)

IMU_ACC_GYRO_ENABLE = ctypes.c_uint8(0x03)
IMU_ACC_X_Y_Z_ENABLE = ctypes.c_uint8(0x07)
IMU_ACC_SENS_2G = ctypes.c_uint8(0x00)
IMU_GYRO_X_Y_Z_ENABLE = ctypes.c_uint8(0x07)
IMU_GYRO_SENS_250DPS = ctypes.c_uint8(0x00)

IMU_CONT_UPDT_EN = (0x01)
IMU_AXES_VALUES_MIN	= (1)
IMU_AXES_VALUES_MAX = (65535)

'''
    Defining DeviceInfo Structure using ctypes
'''
class DeviceInfo(ctypes.Structure):
    _fields_ = [  ("deviceName",ctypes.c_char * 50),
                  ("vid",ctypes.c_char * 5),
                  ("pid",ctypes.c_char * 5),
                  ("devicePath",ctypes.c_char * 250),
                  ("serialNo",ctypes.c_char * 50)
               ]

'''
    Defining IMUCONFIG_TypeDef Structure using ctypes
'''
class IMUCONFIG_TypeDef(ctypes.Structure):
    _fields_ = [  ("IMU_MODE",ctypes.c_uint8),
                  ("ACC_AXIS_CONFIG",ctypes.c_uint8),
                  ("ACC_SENSITIVITY_CONFIG",ctypes.c_uint8),
                  ("GYRO_AXIS_CONFIG",ctypes.c_uint8),
                  ("GYRO_SENSITIVITY_CONFIG",ctypes.c_uint8),
                  ("IMU_ODR_CONFIG",ctypes.c_uint8),
            ]

'''
    Defining IMUDATAOUTPUT_TypeDef Structure using ctypes
'''
class IMUDATAOUTPUT_TypeDef(ctypes.Structure):
    _fields_ = [  ("imu_value_id",ctypes.c_uint16),
                  ("accX",ctypes.c_double),
                  ("accY",ctypes.c_double),
                  ("accZ",ctypes.c_double),
                  ("gyroX",ctypes.c_double),
                  ("gyroY",ctypes.c_double),
                  ("gyroZ",ctypes.c_double),
            ]

'''
    Defining IMUDATAINPUT_TypeDef Structure using ctypes
'''
class IMUDATAINPUT_TypeDef(ctypes.Structure):
    _fields_ = [  ("imu_update_mode",ctypes.c_uint8),
                  ("imu_num_of_values",ctypes.c_uint16),
            ]

if(sys.platform == "linux"):
    sharedLibrary = "libDepthVistaSDK.so"
elif(sys.platform == "win32"):
    sharedLibrary = "DepthVistaSDK.dll"
depthVistaLib = ctypes.CDLL(sharedLibrary)

'''
    Main Class: The initiator class. The program starts from here.
'''
class MainClass:

    glIMU_Interval = 0
    glIMUInput = IMUDATAINPUT_TypeDef()
    lpParameter = IMUDATAOUTPUT_TypeDef()
    lIMUOutput = IMUDATAOUTPUT_TypeDef()
    glIMUAbortThread = False
    glIMU_Interval = 0.0
    angleX = 0.0
    angleY = 0.0
    angleZ = 0.0
    RwEst = []

    initializeresult = depthVistaLib.Initialize
    initializeresult.restype = ctypes.c_int

    deviceCountResult = depthVistaLib.GetDeviceCount
    deviceCountResult.restype = ctypes.c_int
    deviceCountResult.argtypes = [ ctypes.POINTER(ctypes.c_uint32) ]

    getDeviceListInfoResult = depthVistaLib.GetDeviceListInfo
    getDeviceListInfoResult.restype = ctypes.c_int
    getDeviceListInfoResult.argtypes = [ctypes.c_uint32, ctypes.POINTER(DeviceInfo)]

    getDeviceInfoResult = depthVistaLib.GetDeviceInfo
    getDeviceInfoResult.restype = ctypes.c_int
    getDeviceInfoResult.argtypes = [ctypes.c_uint32, ctypes.POINTER(DeviceInfo)]
    
    openDeviceResult =  depthVistaLib.OpenDevice
    openDeviceResult.restype = ctypes.c_int
    openDeviceResult.argtypes = [ctypes.c_uint32]

    isOpenResult = depthVistaLib.IsOpened
    isOpenResult.restype = ctypes.c_int

    closeDeviceResult = depthVistaLib.CloseDevice
    closeDeviceResult.restype = ctypes.c_int

    deinitializeResult = depthVistaLib.DeInitialize
    deinitializeResult.restype = ctypes.c_int

    setIMUConfigResult = depthVistaLib.SetIMUConfig
    setIMUConfigResult.restype = ctypes.c_int
    setIMUConfigResult.argtypes = [IMUCONFIG_TypeDef]

    getIMUConfigResult = depthVistaLib.GetIMUConfig
    getIMUConfigResult.restype = ctypes.c_int
    getIMUConfigResult.argtypes = [ctypes.POINTER(IMUCONFIG_TypeDef)]

    controlIMUCaptureResult = depthVistaLib.ControlIMUCapture
    controlIMUCaptureResult.restype = ctypes.c_int
    controlIMUCaptureResult.argtypes = [ctypes.POINTER(IMUDATAINPUT_TypeDef)]

    getIMUValueResult = depthVistaLib.GetIMUValue
    getIMUValueResult.restype = ctypes.c_int
    getIMUValueResult.argtypes = [ctypes.POINTER(IMUDATAOUTPUT_TypeDef)]

    thread = threading.Thread()
    ctlIMUConfigThread = threading.Thread()

    '''
        METHOD NAME : __init__
        DESCRIPTION : First method to initiate all process
    '''
    def __init__(self):
        self.vid = None
        self.pid = None
        self.device_path = None
        self.device_name = None
        self.No_of_devices = None
        self.serialNo = None
        self.initialize()

    '''
        METHOD NAME : getIMUIntervalTime
        RETURN      : Interval time in float
        DESCRIPTION : Returns the interval time for sampling the values of the IMU
    '''
    def getIMUIntervalTime(self,lIMUConfig=IMUCONFIG_TypeDef()):
        lIMUIntervalTime = int(10)
        if(lIMUConfig.IMU_MODE==IMU_ACC_GYRO_ENABLE):
            if(lIMUConfig.IMU_ODR_CONFIG == IMU_ODR_12_5HZ):
                lIMUIntervalTime = 1000.00 / 12.5
            elif(lIMUConfig.IMU_ODR_CONFIG == IMU_ODR_26HZ):
                lIMUIntervalTime = 1000.00 / 26.0
            elif(lIMUConfig.IMU_ODR_CONFIG == IMU_ODR_52HZ):
                lIMUIntervalTime = 1000.00 / 52.0
            elif(lIMUConfig.IMU_ODR_CONFIG == IMU_ODR_104HZ):
                lIMUIntervalTime = 1000.00 / 104.0
            elif(lIMUConfig.IMU_ODR_CONFIG == IMU_ODR_208HZ):
                lIMUIntervalTime = 1000.00 / 208.0
            elif(lIMUConfig.IMU_ODR_CONFIG == IMU_ODR_416HZ):
                lIMUIntervalTime = 1000.00 / 416.00
            elif(lIMUConfig.IMU_ODR_CONFIG == IMU_ODR_833HZ):
                lIMUIntervalTime = 1000.00 / 833.00
            elif(lIMUConfig.IMU_ODR_CONFIG == IMU_ODR_1666HZ):
                lIMUIntervalTime = 1000.00 / 1666.00
        return lIMUIntervalTime

    '''
        METHOD NAME : UpdateIMUValueThread
        DESCRIPTION : UpdateIMUValue thread uses the values from the GetIMUValue everytime when the semaphore is set
    '''
    def UpdateIMUValueThread(self):
        while((self.glIMUAbortThread == False) and ((self.glIMUInput.imu_update_mode == IMU_CONT_UPDT_EN) or (self.lIMUOutput.imu_value_id <= self.glIMUInput.imu_num_of_values))):
            if(self.glIMUInput.imu_update_mode != IMU_CONT_UPDT_DIS):
                if(self.getIMUValueResult(ctypes.byref(self.lIMUOutput))==1):
                    ret = self.getInclination(self.lIMUOutput.gyroX, self.lIMUOutput.gyroY, self.lIMUOutput.gyroZ, self.lIMUOutput.accX, self.lIMUOutput.accY, self.lIMUOutput.accZ)
                    if(ret == True):
                        self.updateCircles()

    '''
        METHOD NAME : squared
        RETURN      : Returns square value
    '''
    def squared(self, x):
        return x *x

    '''
        METHOD NAME : getInclination
        RETURN      : returns True
        DESCRIPTION : Computes the angle of rotation with respect to the axes
    '''
    def getInclination(self, g_x, g_y, g_z, a_x, a_y, a_z):
        w=0
        tmpf=0.0
        signRzGyro = 0
        firstSample = True
        wGyro = 10.0

	    # Normalise the accelerometer measurement
        norm = math.sqrt(a_x * a_x + a_y * a_y + a_z * a_z)
        a_x /= norm
        a_y /= norm
        a_z /= norm

        RwAcc = [ a_x, a_y, a_z ]
        RwGyro = [ g_x, g_y, g_z ]
        Awz = []
        Gyro = []

        if(firstSample == True):
            #initialize with accelerometer readings
            for w in range(w<=2):
                self.RwEst = RwAcc
        
        else:
            #evaluate Gyro vector
            if(math.fabs(self.RwEst[2]<0.1)):
                #Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
                #in this case skip the gyro data and just use previous estimate
                for w in range(w<=2):
                    Gyro[w]=self.RwEst[w]
            else:
                #get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
                for w in range(w<=1):
                    tmpf = RwGyro[w]
                    tmpf*=self.glIMU_Interval/1000.0
                    Awz[w]=math.atan2(self.RwEst[w], self.RwEst[2])*RAD2DEG
                    Awz[w] += tmpf

                #estimate sign of RzGyro by looking in what qudrant the angle Axz is,
                #RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
                if(math.cos(Awz[0] * DEG2RAD) >= 0): 
                    signRzGyro =  1
                else:
                    signRzGyro = -1

                #reverse calculation of Gyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
                Gyro[0] = math.sin(Awz[0] * DEG2RAD)
                Gyro[0] /= math.sqrt(1 + self.squared(math.cos(Awz[0] * DEG2RAD)) * self.squared(math.tan(Awz[1] * DEG2RAD)))
                Gyro[1] = math.sin(Awz[1] * DEG2RAD)
                Gyro[1] /= math.sqrt(1 + self.squared(math.cos(Awz[1] * DEG2RAD)) * self.squared(math.tan(Awz[0] * DEG2RAD)))
                Gyro[2] = signRzGyro * math.sqrt(1 - self.squared(Gyro[0]) - self.squared(Gyro[1]))

            #combine Accelerometer and gyro readings
            for w in range(w<=2):
                self.RwEst[w] = (RwAcc[w] + wGyro * Gyro[w]) / (1 + wGyro)

            #Normalizing the estimates
            norm = math.sqrt(self.RwEst[0] * self.RwEst[0] + self.RwEst[1] * self.RwEst[1] + self.RwEst[2] * self.RwEst[2]);
            self.RwEst[0] /= norm
            self.RwEst[1] /= norm
            self.RwEst[2] /= norm

        firstSample = False

        #Computing the angles
        self.angleX = self.RwEst[0] * HALF_PI * RAD2DEG
        self.angleY = self.RwEst[1] * HALF_PI * RAD2DEG
        self.angleZ = self.RwEst[2] * HALF_PI * RAD2DEG
        # print(f"angles : {self.angleX}, {self.angleY}, {self.angleZ}")
        return True

    '''
        METHOD NAME : PointOnCircle
        RETURN      : returns points tuple
        DESCRIPTION : Drawing Points in circles for illustration
    '''
    def PointOnCircle(self, radius, angleInDegrees , origin):
        #radius -> Radius of Circle & Origin -> Circle Centre.
        #Convert from degrees to radians via multiplication by PI/180        
        x = (radius * math.cos(angleInDegrees * DEG2RAD)) + origin[0]
        y = (radius * math.sin(angleInDegrees * DEG2RAD)) + origin[1]
        return (int(x), int(y))

    '''
        METHOD NAME : updateCircles
        DESCRIPTION : Drawing angles in circles for illustration
    '''
    def updateCircles(self):
        drawImage = np.zeros((400, 800, 3), dtype=np.uint8)

        #Static Labelling.
        cv2.putText(drawImage, "x", (150, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (0, 0, 255), 1)
        cv2.putText(drawImage, "y", (400, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (0, 255, 0), 1)
        cv2.putText(drawImage, "z", (650, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (255, 0, 0), 1)

        #Labelling the angles.
        ss = "%d deg" %int(self.angleX)
        cv2.putText(drawImage, ss, (150, 350), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
        ss=""

        ss = "%d deg" %int(self.angleY)
        cv2.putText(drawImage, ss, (400, 350), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
        ss=""

        ss = "%d deg" %int(self.angleZ)
        cv2.putText(drawImage, ss, (650, 350), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 0), 1)

        P1=self.PointOnCircle(100, -self.angleX+180, (150, 200))
        P2=self.PointOnCircle(100, -self.angleX, (150, 200))

        cv2.ellipse(drawImage, (150, 200), (100, 100), 0, -self.angleX, -self.angleX - 180, (20, 20, 20), -1)
        cv2.ellipse(drawImage, (150, 200), (100, 100), 0, -self.angleX, -self.angleX + 180, (0, 0, 255), -1)
        cv2.circle(drawImage, P1, 2, (0, 0, 255), 2)
        cv2.circle(drawImage, P2, 2, (0, 0, 255), 2)
        cv2.line(drawImage, P1, P2, (0, 0, 255), 2)

        P1=self.PointOnCircle(100, -self.angleY+180, (400, 200))
        P2=self.PointOnCircle(100, -self.angleY, (400, 200))

        cv2.ellipse(drawImage, (400, 200), (100, 100), 0, -self.angleY, -self.angleY - 180, (20, 20, 20), -1)
        cv2.ellipse(drawImage, (400, 200), (100, 100), 0, -self.angleY, -self.angleY + 180, (0, 255, 0), -1)
        cv2.circle(drawImage, P1, 2, (0, 255, 0), 2)
        cv2.circle(drawImage, P2, 2, (0, 255, 0), 2)
        cv2.line(drawImage, P1, P2, (0, 255, 0), 2)

        P1=self.PointOnCircle(100, -self.angleZ+180, (650, 200))
        P2=self.PointOnCircle(100, -self.angleZ, (650, 200))

        cv2.ellipse(drawImage, (650, 200), (100, 100), 0, -self.angleZ, -self.angleZ - 180, (20, 20, 20), -1)
        cv2.ellipse(drawImage, (650, 200), (100, 100), 0, -self.angleZ, -self.angleZ + 180, (255, 0, 0), -1)
        cv2.circle(drawImage, P1, 2, (255, 0, 0), 2)
        cv2.circle(drawImage, P2, 2, (255, 0, 0), 2)
        cv2.line(drawImage, P1, P2, (255, 0, 0), 2)

        cv2.imshow("Inclination", drawImage)
        cv2.waitKey(1)

    '''
        METHOD NAME : initialize
        DESCRIPTION : Initiates the main menu to list available device(s)
    '''
    def initialize(self):

        print(" IMU Sample Application ".center(100, " "))
        print(" Application to illustrate the IMU unit ICM-20789 integrated with DepthVista Camera ".center(100, " "))
        print(" Demonstrating the rotations of camera around x-axis and y-axis ".center(100, " "))
        print(" IMU values are limited from -90 to +90 degrees for illustration ".center(100, " "))
        
        if(self.initializeresult() < 0):
            print("Device Initialization failed")

        numberOfDevices = ctypes.c_uint32 ()
        if(self.deviceCountResult(ctypes.byref(numberOfDevices)) < 0):
            print("Get Device count failed")

        if(numberOfDevices.value > 0):
            print(f'\nNumber of devices connected : {numberOfDevices.value}')

            DeviceList = DeviceInfo()
            DeviceList.deviceName = b'None'
            DeviceList.pid = b'None'
            DeviceList.vid = b'None'
            DeviceList.devicePath = b'None'
            DeviceList.serialNo = b'None'

            for i in range(0, numberOfDevices.value):
                if(self.getDeviceInfoResult(i, ctypes.byref(DeviceList)) > 0):
                    deviceNameValue = DeviceList.deviceName.decode('utf-8')
                    vidValue        = DeviceList.vid.decode('utf-8')
                    pidValue        = DeviceList.pid.decode('utf-8')
                    devicePathValue = DeviceList.devicePath.decode('utf-8')
                    serialNoValue   = DeviceList.serialNo.decode('utf-8')
                    print(f"\nDevice Id : {i+1}, Device Name : {deviceNameValue}")

        elif(numberOfDevices.value == 0):
            self.deinitializeResult()
            print("\n0 : Exit\r\n")
            print("\n1 : Enumerate Again\r\n")
            enumerate_again = get_integer("", 0, 1)
            if(enumerate_again == 0):
                return None
            else:
                self.initialize()
            
        deviceID = get_integer("\nEnter the device ID to proceed : ", 1, numberOfDevices.value+1)
        if(self.openDeviceResult(deviceID-1) < 0):
            print("Error in Opening the Device")
        else:
            sIMUConfig = IMUCONFIG_TypeDef()
            sIMUConfig.IMU_MODE = IMU_ACC_GYRO_ENABLE
            sIMUConfig.ACC_AXIS_CONFIG = IMU_ACC_X_Y_Z_ENABLE
            sIMUConfig.IMU_ODR_CONFIG = IMU_ODR_104HZ
            sIMUConfig.ACC_SENSITIVITY_CONFIG = IMU_ACC_SENS_2G
            sIMUConfig.GYRO_AXIS_CONFIG = IMU_GYRO_X_Y_Z_ENABLE
            sIMUConfig.GYRO_SENSITIVITY_CONFIG = IMU_GYRO_SENS_250DPS

            time.sleep(0.5)
            if(self.setIMUConfigResult(sIMUConfig)<0):
                print("\nSet IMU Config Failed")
            else:
                gIMUConfig = IMUCONFIG_TypeDef()
                if(self.getIMUConfigResult(ctypes.byref(gIMUConfig))<0):
                    print("\nGet IMU Config Failed")
                    return None
                else:
                    self.glIMU_Interval = self.getIMUIntervalTime(gIMUConfig)
        	
        lIMUInput = IMUDATAINPUT_TypeDef()
        lIMUInput.imu_update_mode = IMU_CONT_UPDT_EN
        lIMUInput.imu_num_of_values = IMU_AXES_VALUES_MIN

        if(self.controlIMUCaptureResult(ctypes.byref(lIMUInput))<0):
            print("Control IMU Capture  Failed\r\n")
            return None
        else:
            self.glIMUInput = lIMUInput

        self.lIMUOutput.imu_value_id = 0

        self.thread = threading.Thread(target=self.UpdateIMUValueThread, name="updateIMUValue_thread", daemon=False)
        self.thread.start()
        
        exitCh = get_integer("\nPress 0 key to stop : ", 0, 0)
        if(exitCh == 0):
            self.stopDraw()

    '''
        METHOD NAME : stopDraw
        DESCRIPTION : Exit call to close and deinitialize device
    '''
    def stopDraw(self):
        lIMUInput = IMUDATAINPUT_TypeDef()
        lIMUInput.imu_update_mode = IMU_CONT_UPDT_DIS
        lIMUInput.imu_num_of_values = IMU_AXES_VALUES_MIN
        uStatus = self.controlIMUCaptureResult(ctypes.byref(lIMUInput))
        if(uStatus == 0):
            print("\nControl IMU Capture Failed")
            return None
        
        self.glIMUAbortThread = True
        time.sleep(0.5)
        if(self.closeDeviceResult()==1):
            if(self.deinitializeResult()==1):
                cv2.destroyAllWindows()
        exit(0)

if __name__ == '__main__':
    main = MainClass()

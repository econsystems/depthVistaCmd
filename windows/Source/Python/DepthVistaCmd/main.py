import cv2
import ctypes
from ctypes import*
from input import get_integer
import threading
import numpy as np
import time
from enum import Enum
from datetime import datetime
import sys

depth_min = 500
depth_max = 3250
depth_range = (depth_max-depth_min)*0.20

PRE_RGB_VGA_WIDTH = PRE_DEPTH_WIDTH = PRE_IR_WIDTH = 640
PRE_RGB_VGA_HEIGHT = PRE_DEPTH_HEIGHT = PRE_IR_HEIGHT = 480

#Preview RGB HD Resolution 1280x720
PRE_RGB_HD_WIDTH = 1280
PRE_RGB_HD_HEIGHT = 720

# Preview RGB Full HD Resolution 1920x1080
PRE_RGB_FULL_HD_WIDTH = 1920
PRE_RGB_FULL_HD_HEIGHT = 1080

#Preview RGB original Resolution 1920x1200
PRE_RGB_ORIGINAL_WIDTH = 1920
PRE_RGB_ORIGINAL_HEIGHT = 1200

'''
    Defining tofFrame Structure using ctypes
'''
class tofFrame(ctypes.Structure):
    _fields_ = [ ("frame_data",ctypes.POINTER(ctypes.c_uint8)),
                 ("width",ctypes.c_int16),
                 ("height",ctypes.c_int16),
                 ("pixel_format",ctypes.c_int8),
                 ("total_size",ctypes.c_uint32)
               ]

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
    Defining DepthPtr Structure using ctypes
'''
class DepthPtr(ctypes.Structure):
    _fields_ = [  ("X", ctypes.c_int),
                  ("Y", ctypes.c_int)
               ]

'''
    Defining Datamode Enum
'''
class DataMode(Enum):
    ModeUnknown = 255
    Depth_IR_Mode = 0
    Raw_Mode = 254
    Depth_Mode = 2
    IR_Mode = 3
    Depth_IR_RGB_VGA_Mode = 4
    Depth_IR_RGB_HD_Mode = 5
    RGB_VGA_Mode = 6
    RGB_HD_Mode = 7
    RGB_Full_HD_Mode = 8
    RGB_Original_Mode = 9

'''
    Defining DepthRange Enum
'''
class DepthRange(Enum):
    NearRange = 0
    FarRange = 1

'''
    Defining FrameType Enum
'''
class FrameType(Enum):
	RawIRFrame = 0
	DepthPreviewFrame = 1
	IRPreviewFrame = 2
	DepthColorMap = 3
	RGBFrame = 4
	DepthIrRgbRawFrame = 5
	DepthIrRawFrame = 6
	DepthRawFrame = 7
	IRRawFrame = 8

'''
    Defining AvgRegion Enum
'''
class AvgRegion(Enum):
    Center = 0
    CustomPtr = 1
    Exit = -1

if(sys.platform == "linux"):
    sharedLibrary = "libDepthVistaSDK.so"
elif(sys.platform == "win32"):
    sharedLibrary = "DepthVistaSDK.dll"
depthVistaLib = ctypes.CDLL(sharedLibrary)

'''
    Main Class: The initiator class. The program starts from here.
'''
class MainClass:

    initializeresult = depthVistaLib.Initialize
    initializeresult.restype = ctypes.c_int

    deviceCountResult = depthVistaLib.GetDeviceCount
    deviceCountResult.restype = ctypes.c_int
    deviceCountResult.argtypes = [ ctypes.POINTER(ctypes.c_uint32) ]

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

    setdatamodevar = depthVistaLib.SetDataMode
    setdatamodevar.restype = ctypes.c_int
    setdatamodevar.argtypes = [ctypes.c_int32]

    getDataMode = depthVistaLib.GetDataMode
    getDataMode.restype = ctypes.c_int
    getDataMode.argtypes = [ ctypes.POINTER(ctypes.c_int32) ]

    setDepthRange = depthVistaLib.SetDepthRange
    setDepthRange.restype = ctypes.c_int
    setDepthRange.argtypes = [ctypes.c_uint16]

    getDepthRange = depthVistaLib.GetDepthRange
    getDepthRange.restype = ctypes.c_int
    getDepthRange.argtypes = [ ctypes.POINTER(ctypes.c_int16) ]

    nextFrameResult = depthVistaLib.GetNextFrame
    nextFrameResult.restype = ctypes.c_int

    ToFFrameResult = depthVistaLib.GetToFFrame
    ToFFrameResult.restype = ctypes.c_int
    ToFFrameResult.argtypes = [ctypes.c_uint32, ctypes.POINTER(tofFrame)]
    
    updateColorMapRes = depthVistaLib.UpdateColorMap
    updateColorMapRes.restype = ctypes.c_int
    updateColorMapRes.argtypes = [ctypes.c_int32, ctypes.c_int32, ctypes.c_int32]

    setPlanarization = depthVistaLib.SetPlanarization
    setPlanarization.restype = ctypes.c_int
    setPlanarization.argtypes = [ctypes.c_uint16]

    setUnDistortion = depthVistaLib.SetUnDistortion
    setUnDistortion.restype = ctypes.c_int
    setUnDistortion.argtypes = [ctypes.c_uint16]

    setAvgRegion = depthVistaLib.SetAvgRegion
    setAvgRegion.restype = ctypes.c_int
    setAvgRegion.argtypes = [ctypes.c_uint16]

    readfirmwareVersion = depthVistaLib.ReadFirmwareVersion
    readfirmwareVersion.restype = ctypes.c_int
    readfirmwareVersion.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint16)]

    getDepthIRValues = depthVistaLib.GetDepthIRValues
    getDepthIRValues.restype = ctypes.c_int
    getDepthIRValues.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]

    setDepthPos = depthVistaLib.SetDepthPos
    setDepthPos.restype = ctypes.c_int
    setDepthPos.argtypes = [DepthPtr]

    getUniqueID = depthVistaLib.GetUniqueID
    getUniqueID.restype = ctypes.c_int
    getUniqueID.argtypes = [ctypes.POINTER(ctypes.c_uint64)]

    Thread_end = threading.Event()
    modechange = threading.Event()

    ToFFrame = tofFrame()

    GetNextFrameThread = threading.Thread()
    
    thread_lock_flag = False
    thread_lock = threading.Lock()

    datamode = DataMode.Depth_IR_RGB_VGA_Mode.value
    depthrange = DepthRange.FarRange.value

    depth_cap = False
    rgb_cap = False
    IR_cap = False

    depthStreamStarted = False

    save_possible = False

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
        self.main_menu_init()

    print(" e-con's Sample Python script for DepthVista ".center(100, " "))
    print(" Demonstrates the working of e-con's DepthVistaSDK ".center(100, " "))
    print(" DepthVista SDK-Version = 1.0.3 ".center(100, " "))

    '''
        METHOD NAME : list_devices
        DESCRIPTION : Enumerates all the video devices connected and allows to select the device.
        RETURN      : device name, vendor id, product id, device path - if all the child functions are executes successfully
                     or None - if any of the child function is failed.
    '''
    def listDevices(self):

        #close and deinitialize device if already opened
        if(self.isOpenResult()==1):
            self.modechange.set()
            time.sleep(0.2)
            self.thread_lock_flag = True
            if(self.closeDeviceResult()):
                if(self.deinitializeResult() == 1):
                    cv2.destroyAllWindows()
        
        #Initializing all the APIs in shared library
        if(self.initializeresult() == 1):

            numberOfDevices = ctypes.c_uint32 ()
            #Getting device count
            if self.deviceCountResult(ctypes.byref(numberOfDevices)) == 1:
                numberOfDevices = numberOfDevices.value
                print(f'\nNumber of Depth_vista (TOF) Devices connected: {numberOfDevices}')
                print("\n\t0.Exit")

            #Getting device list information
            DeviceList = DeviceInfo()

            DeviceList.deviceName = b'None'
            DeviceList.pid = b'None'
            DeviceList.vid = b'None'
            DeviceList.devicePath = b'None'
            DeviceList.serialNo = b'None'

            #Passing DeviceList By Reference by using byref
            if(self.getDeviceInfoResult(0 ,ctypes.byref(DeviceList)) == 1):

                #Decoding the byte literals using decode()
                deviceNameValue = DeviceList.deviceName.decode('utf-8')
                vidValue        = DeviceList.vid.decode('utf-8')
                pidValue        = DeviceList.pid.decode('utf-8')
                devicePathValue = DeviceList.devicePath.decode('utf-8')
                serialNoValue   = DeviceList.serialNo.decode('utf-8')

                for i in range(0, 1):
                    print(f"\t{i + 1}.{deviceNameValue}")

                choice = get_integer("\nPick a Device to explore :\t", 0, 1)

                if choice == 0:
                    return None

                #Opening the device
                if(self.openDeviceResult(0) < 1):
                    print("Error in Opening the Device")

                return numberOfDevices, deviceNameValue, vidValue, pidValue, devicePathValue, serialNoValue

            else:
                print("\nNo Depthvista device found")
        else:
            print("\nFailed to initialize device")

    '''
        METHOD NAME : update_colormap
        DESCRIPTION : Calls the updatecolormap method from depthvista SDK
    '''
    def update_colormap(self):
        global depth_min, depth_max, depth_range
        if(self.updateColorMapRes(depth_min, depth_max + int(depth_range), 4)!=1):
            print("\nUpdate colormap failed")

    '''
        METHOD NAME : Preview
        DESCRIPTION : Provides the preview screen and render frames
    '''
    def Preview(self):

        while True:
            
            if(self.thread_lock_flag == True):
                self.thread_lock.acquire()

            #Thread end condition
            if(self.Thread_end.is_set()==True):
                cv2.destroyAllWindows()
                break
            
            #mode change loop
            while not self.modechange.is_set():

                self.save_possible = True
                
                if self.nextFrameResult() == 1:
                    #Depth colormap frame
                    if(self.datamode != DataMode.IR_Mode.value) and (self.datamode <= DataMode.Depth_IR_RGB_HD_Mode.value):
                        frametype = FrameType.DepthColorMap.value
                        self.update_colormap()
                        self.depthStreamStarted = True

                        #Acquiring thread lock to avoid race condition
                        lock = threading.Lock()
                        lock.acquire()

                        if(self.ToFFrameResult(frametype, ctypes.byref(self.ToFFrame)) == 1):
                            image_data = np.ctypeslib.as_array(self.ToFFrame.frame_data, (PRE_DEPTH_HEIGHT, PRE_DEPTH_WIDTH, 3))

                            #Releasing thread lock
                            lock.release()

                            if(lock.locked() == 0):
                                #capture frame condition
                                if(self.depth_cap==True):
                                    time=datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
                                    file_name = "DepthVista_Depth_"+time+".bmp"
                                    cv2.imwrite(file_name, image_data)
                                    self.ToFFrameResult(FrameType.DepthRawFrame.value, ctypes.byref(self.ToFFrame))
                                    capture_data = np.ctypeslib.as_array(self.ToFFrame.frame_data, (PRE_DEPTH_HEIGHT, PRE_DEPTH_WIDTH, 2))
                                    file_name = "DepthVista_Raw_"+time+".raw"
                                    try:
                                        fp = open(file_name, 'wb+')
                                        fp.write(capture_data)
                                        fp.close()
                                    except IOError:
                                        print("File operation error.Image is not saved!")
                                        return False
                                    self.depth_cap=False
                                #stream show
                                cv2.namedWindow("DepthVista_Depthcolormap_frame",cv2.WINDOW_GUI_NORMAL)
                                cv2.setWindowProperty("DepthVista_Depthcolormap_frame", cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_FULLSCREEN)
                                cv2.setMouseCallback("DepthVista_Depthcolormap_frame", self.mouseCallBck)
                                cv2.imshow("DepthVista_Depthcolormap_frame",image_data)
                                cv2.waitKey(1)

                        else:
                            print("\nFailed in getting TOF frames")

                    #rgb frame
                    if(self.datamode >= DataMode.Depth_IR_RGB_VGA_Mode.value) and (self.datamode != DataMode.Raw_Mode.value):
                        frametype = FrameType.RGBFrame.value

                        if(self.datamode == DataMode.RGB_VGA_Mode.value) or (self.datamode == DataMode.Depth_IR_RGB_VGA_Mode.value):
                            rgb_height = PRE_RGB_VGA_HEIGHT
                            rgb_width = PRE_RGB_VGA_WIDTH
                        if(self.datamode == DataMode.RGB_Original_Mode.value):
                            rgb_height = PRE_RGB_ORIGINAL_HEIGHT
                            rgb_width = PRE_RGB_ORIGINAL_WIDTH
                        if(self.datamode == DataMode.RGB_HD_Mode.value) or (self.datamode == DataMode.Depth_IR_RGB_HD_Mode.value):
                            rgb_height = PRE_RGB_HD_HEIGHT
                            rgb_width = PRE_RGB_HD_WIDTH
                        if(self.datamode == DataMode.RGB_Full_HD_Mode.value):
                            rgb_height = PRE_RGB_FULL_HD_HEIGHT
                            rgb_width = PRE_RGB_FULL_HD_WIDTH

                        #Acquiring thread lock to avoid race condition
                        lock = threading.Lock()
                        lock.acquire()

                        if(self.ToFFrameResult(frametype, ctypes.byref(self.ToFFrame)) == 1):

                            image = np.ctypeslib.as_array(self.ToFFrame.frame_data, (rgb_height, rgb_width, 2))
                            image_data = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)

                            #Releasing thread lock
                            lock.release()

                            if(lock.locked() == 0):
                                if(self.rgb_cap == True):
                                    #capture frame condition
                                    time=datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
                                    file_name = "DepthVista_rgb_"+time+".png"
                                    cv2.imwrite(file_name, image_data)
                                    self.rgb_cap=False
                                #Stream show
                                cv2.namedWindow("DepthVista_RGB_frame",cv2.WINDOW_GUI_NORMAL)
                                cv2.imshow("DepthVista_RGB_frame",image_data)
                                cv2.setWindowProperty("DepthVista_RGB_frame", cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_FULLSCREEN)
                                cv2.waitKey(1)

                        else:
                            print("\nFailed in getting TOF frames")

                    #IR frame
                    if(self.datamode != DataMode.Depth_Mode.value) and (self.datamode <=DataMode.Depth_IR_RGB_HD_Mode.value):
                        frametype = FrameType.IRPreviewFrame.value

                        #Acquiring thread lock to avoid race condition
                        lock = threading.Lock()
                        lock.acquire()

                        if(self.ToFFrameResult(frametype, ctypes.byref(self.ToFFrame)) == 1):
                            image = ctypes.cast(self.ToFFrame.frame_data, POINTER(ctypes.c_uint16))
                            image_data = np.ctypeslib.as_array(image, (PRE_IR_HEIGHT, PRE_IR_WIDTH, 1))

                            #Releasing thread lock
                            lock.release()

                            if(lock.locked() == 0):
                                #capture frame condition
                                if(self.IR_cap == True):
                                    time=datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
                                    file_name = "DepthVista_IR_"+time+".png"
                                    cv2.imwrite(file_name, image_data)
                                    self.IR_cap=False
                                #Stream show
                                cv2.namedWindow("DepthVista_IR_frame",cv2.WINDOW_GUI_NORMAL)
                                cv2.setWindowProperty("DepthVista_IR_frame", cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_FULLSCREEN)
                                cv2.imshow("DepthVista_IR_frame",image_data)
                                cv2.waitKey(1)

                        else:
                            print("\nFailed in getting TOF frames")
            
            self.depthStreamStarted=False
            self.save_possible = False
            cv2.destroyAllWindows()

    '''
        METHOD NAME : main_menu_init
        DESCRIPTION : Initiates the main menu to list available device(s)
    '''
    def main_menu_init(self):
        deviceInfo = self.listDevices()
        if deviceInfo is None:
            self.main_menu_exit()
        self.No_of_devices, self.device_name, self.vid, self.pid, self.device_path, self.serialNo = deviceInfo

        if(self.GetNextFrameThread.is_alive()):
            if(self.thread_lock_flag == True):

                self.datamode=self.getdatamode()
                time.sleep(0.5)
                self.depthrange=self.getdepthrange()
                time.sleep(0.5)
                self.modechange.clear()
                self.thread_lock.release()
                self.thread_lock_flag = False
                self.propertiesMenu()
            else:
                self.modechange.set()
                self.datamode=self.getdatamode()
                time.sleep(0.5)
                self.depthrange=self.getdepthrange()
                time.sleep(0.5)
                self.modechange.clear()
                self.propertiesMenu()

        self.datamode=self.getdatamode()
        time.sleep(0.5)
        self.depthrange=self.getdepthrange()
        time.sleep(0.5)
        self.modechange.clear()
        self.GetNextFrameThread = threading.Thread(target=self.Preview, name="preview_thread", daemon=False)
        self.GetNextFrameThread.start()

        self.setavgregion(AvgRegion.CustomPtr.value)

        self.propertiesMenu()

    '''
        METHOD NAME : propertiesMenu
        DESCRIPTION : Lists the properties and gets the user input to transfer control
    '''
    def propertiesMenu(self):
        propertiesMenuOptions = {
            0: self.main_menu_exit,
            1: self.__init__,
            2: self.streamingMode,
            3: self.depthRange,
            4: self.Planarization,
            5: self.depthUndistortion,
            6: self.captureFrames,
            7: self.uniqueId,
            8: self.readFirmwareVersion,
            9: self.getDepthValue,
        }
        while True:
            print("CAMERA PROPERTIES MENU".center(100, "*"))
            print("\n\t0.Exit")
            print("\t1.Back")
            print("\t2.Streaming Mode")
            print("\t3.Depth Range")
            print("\t4.Planarization")
            print("\t5.Depth Undistortion")
            print("\t6.Capture Frames")
            print("\t7.Unique Id")
            print("\t8.Read Firmware Version")
            print("\t9.Get Depth value\n")

            choice = get_integer("Pick a Relevant Type Of Camera Properties :\t", min(propertiesMenuOptions, key=int),max(propertiesMenuOptions, key=int))
            time.sleep(0.5)
            func = propertiesMenuOptions.get(choice, lambda: "Invalid Selection")

            if not func():
                self.main_menu_exit()

    '''
        METHOD NAME : streamingMode
        DESCRIPTION : Lists the streaming modes available and gets the user input to transfer control
    '''    
    def streamingMode(self):
        
        streamingModeOptions={
            0: self.main_menu_exit,
            1: self.propertiesMenu,
            2: self.__init__,
            3: DataMode.Depth_IR_Mode,
            4: DataMode.Depth_Mode,
            5: DataMode.IR_Mode,
            6: DataMode.Depth_IR_RGB_VGA_Mode,
            7: DataMode.Depth_IR_RGB_HD_Mode,
            8: DataMode.RGB_VGA_Mode,
            9: DataMode.RGB_HD_Mode,
            10: DataMode.RGB_Full_HD_Mode,
            11: DataMode.RGB_Original_Mode,
        }

        while True:
            print("\nTotal Number of Streaming Modes Supported By the Camera: 9")
            print("STREAMING MODE MENU".center(100, "*"))
            print("\n\t0.Exit")
            print("\t1.Back")
            print("\t2.Main Menu")
            print("\t3.Depth IR Mode")
            print("\t4.Depth Mode")
            print("\t5.IR Mode")
            print("\t6.Depth IR RGB(VGA) Mode")
            print("\t7.Depth IR RGB(HD) Mode")
            print("\t8.RGB(VGA) Mode")
            print("\t9.RGB(HD) Mode")
            print("\t10.RGB(Full HD) Mode")
            print("\t11.RGB(1200p) Mode")

            choice = get_integer("\nPick a relevant Streaming Mode :\t", min(streamingModeOptions, key=int),max(streamingModeOptions, key=int))
            func = streamingModeOptions.get(choice, lambda: "\nInvalid Selection")
            
            if(choice > 2)and(choice < 12):

                if(self.GetNextFrameThread.is_alive()):
                    if(self.thread_lock_flag == True):

                        self.setdatamode(func.value)
                        self.datamode=self.getdatamode()
                        time.sleep(0.5)
                        self.modechange.clear()
                        self.thread_lock.release()
                        self.thread_lock_flag = False
                        self.propertiesMenu()
                    else:
                        self.modechange.set()
                        self.setdatamode(func.value)
                        self.datamode=self.getdatamode()
                        time.sleep(0.5)
                        self.modechange.clear()
                        self.propertiesMenu()

                self.setdatamode(func.value)
                self.datamode=self.getdatamode()
                time.sleep(0.5)
                self.modechange.clear()
                self.GetNextFrameThread = threading.Thread(target=self.Preview, name="preview_thread", daemon=False)
                self.GetNextFrameThread.start()
                self.propertiesMenu()

            if(choice < 3)and(choice >= 0):
                if not func():
                    self.propertiesMenu()

    '''
        METHOD NAME : getdatamode
        DESCRIPTION : This method returns the value of active datamode
        RETURN      : Integer value of Datamode
    '''
    def getdatamode(self):
        datamode=ctypes.c_int32()
        if(self.getDataMode(ctypes.byref(datamode)) == 1):
            return datamode.value
        else:
            print("Datamode get failed")

    '''
        METHOD NAME : setdatamode
        DESCRIPTION : This method sets the datamode
    '''
    def setdatamode(self, datamode=ctypes.c_int32):
        if(self.setdatamodevar(datamode) < 1):
            print("Datamode set failed")

    def getdepthrange(self):
        depthrange=ctypes.c_int16()
        if(self.getDepthRange(ctypes.byref(depthrange)) == 1):
            return depthrange.value
        else:
            print("DepthRange get failed")

    '''
        METHOD NAME : setdepthrange
        DESCRIPTION : This method sets the depthrange
    '''
    def setdepthrange(self, depthrange=ctypes.c_int16):
        if(self.setDepthRange(depthrange) < 1):
            print("DepthRange set failed")
   
    '''
        METHOD NAME : setavgregion
        DESCRIPTION : This method sets the avgregion
    '''
    def setavgregion(self, avgregion=ctypes.c_int16):
        if(self.setAvgRegion(avgregion) < 1):
            print("Average Region set failed")

    '''
        METHOD NAME : depthRange
        DESCRIPTION : Lists the depth range available and gets the user input to transfer control
    '''
    def depthRange(self):
        depthRangeOptions={
            0: self.main_menu_exit,
            1: self.propertiesMenu,
            2: self.__init__,
            3: self.nearMode,
            4: self.farMode,
        }
        while True:
            print("DEPTH_RANGE MENU".center(100, "*"))
            print("\n\t0.Exit")
            print("\t1.Back")
            print("\t2.Main Menu")
            print("\t3.Near Mode")
            print("\t4.Far Mode")

            depthRangechoice = get_integer("\nPick a Relevant Depth Range :\t", min(depthRangeOptions, key=int),max(depthRangeOptions, key=int))
            func = depthRangeOptions.get(depthRangechoice, lambda: "Invalid Selection")

            if not func():
                self.main_menu_exit()

    '''
        METHOD NAME : nearMode
        DESCRIPTION : Change the depth range to near range
    '''
    def nearMode(self):
        global depth_max, depth_min
        if(self.setDepthRange(0) < 1):
            print("\nNear mode set failed")
        else:
            depth_min = 200
            depth_max = 1500
            print("\nDepth Range Near")
        self.propertiesMenu()

    '''
        METHOD NAME : farMode
        DESCRIPTION : Change the depth range to far range
    '''
    def farMode(self):
        global depth_max, depth_min
        if(self.setDepthRange(1) < 1):
            print("\nFar mode set failed")
        else:
            depth_min = 500
            depth_max = 3250
            print("\nDepth Range Far")
        self.propertiesMenu()

    '''
        METHOD NAME : Planarization
        DESCRIPTION : Lists the Planarization options available and gets the user input to transfer control
    '''
    def Planarization(self):
        planarizationMenuOptions = {
            0: self.main_menu_exit,
            1: self.propertiesMenu,
            2: self.__init__,
            3: self.planarizationOff,
            4: self.planarizationOn,
            }
        while True:
            print("PLANARIZATION MODE MENU".center(100, "*"))
            print("\n\t0.Exit")
            print("\t1.Back")
            print("\t2.Main Menu")
            print("\t3.Planarization OFF")
            print("\t4.Planarization ON")

            choice = get_integer("\nPick a relevant Planarization mode :\t", min(planarizationMenuOptions, key=int),max(planarizationMenuOptions, key=int))
            func = planarizationMenuOptions.get(choice, lambda: "Invalid Selection")

            if not func():
                self.main_menu_exit()

    '''
        METHOD NAME : planarizationOff
        DESCRIPTION : Turns OFF planarization at the instance of call
    '''
    def planarizationOff(self):
        if(self.setPlanarization(0) < 1):
            print("\nDepth Planarization OFF failed")
        else:
            print("\nDepth Planarization OFF")
        self.propertiesMenu()

    '''
        METHOD NAME : planarizationOn
        DESCRIPTION : Turns ON planarization at the instance of call
    '''
    def planarizationOn(self):
        if(self.setPlanarization(1) < 1):
            print("\nDepth Planarization ON failed")
        else:
            print("\nDepth Planarization ON")
        self.propertiesMenu()

    '''
        METHOD NAME : depthUndistortion
        DESCRIPTION : Lists the distortion mode options available and gets the user input to transfer control
    '''
    def depthUndistortion(self):
        depthUndistortionMenuOptions = {
            0: self.main_menu_exit,
            1: self.propertiesMenu,
            2: self.__init__,
            3: self.undistortionOff,
            4: self.undistortionOn,
            }
        while True:
            print("DEPTH UNDISTORTION MODE MENU".center(100, "*"))
            print("\n\t0.Exit")
            print("\t1.Back")
            print("\t2.Main Menu")
            print("\t3.Undistortion OFF")
            print("\t4.Undistortion ON")

            choice = get_integer("\nPick a relevant Distortion mode :\t", min(depthUndistortionMenuOptions, key=int),max(depthUndistortionMenuOptions, key=int))
            func = depthUndistortionMenuOptions.get(choice, lambda: "Invalid Selection")

            if not func():
                self.main_menu_exit()

    '''
        METHOD NAME : undistortionOff
        DESCRIPTION : Turns OFF Undistortion at the instance of call
    '''
    def undistortionOff(self):
        if(self.setUnDistortion(0) < 1):
            print("\nDepth Undistortion OFF failed")
        else:
            print("\nDepth Undistortion OFF")
        self.propertiesMenu()

    '''
        METHOD NAME : undistortionOn
        DESCRIPTION : Turns ON Undistortion at the instance of call
    '''
    def undistortionOn(self):
        if(self.setUnDistortion(1) < 1):
            print("\nDepth Undistortion ON failed")
        else:
            print("\nDepth Undistortion ON")
        self.propertiesMenu()

    '''
        METHOD NAME : captureFrames
        DESCRIPTION : Captures frame that is shown at the instance of call
    '''
    def captureFrames(self):
        if(self.save_possible == True):
            if(self.datamode != DataMode.IR_Mode.value) and (self.datamode <= DataMode.Depth_IR_RGB_HD_Mode.value):
                self.depth_cap = True
            if(self.datamode >= DataMode.Depth_IR_RGB_VGA_Mode.value) and (self.datamode != DataMode.Raw_Mode.value):
                self.rgb_cap = True
            if(self.datamode != DataMode.Depth_Mode.value) and (self.datamode <=DataMode.Depth_IR_RGB_HD_Mode.value):
                self.IR_cap = True
            print("\nFrame Captured")
        else:
            print("\nImage capture failed, Please start Preview Before Capturing Frames")
        self.propertiesMenu()

    '''
        METHOD NAME : uniqueId
        DESCRIPTION : Prints the uniqueID of the camera
    '''
    def uniqueId(self):
        uniqueID = ctypes.c_uint64()
        
        if(self.getUniqueID(ctypes.byref(uniqueID)) < 1):
            print("\nUnique ID read failed")
        else:
            print("\nUnique ID of the Camera is ",uniqueID.value)
        self.propertiesMenu()

    '''
        METHOD NAME : readFirmwareVersion
        DESCRIPTION : Prints the firmware version
    '''
    def readFirmwareVersion(self):
        gMajorVersion = ctypes.c_uint8()
        gMinorVersion1 = ctypes.c_uint8()
        gMinorVersion2 = ctypes.c_uint16()
        gMinorVersion3 = ctypes.c_uint16()
    
        if(self.readfirmwareVersion(ctypes.byref(gMajorVersion), ctypes.byref(gMinorVersion1), ctypes.byref(gMinorVersion2), ctypes.byref(gMinorVersion3)) < 1):
            print("\nFirmware version Read failed")
        else:
            FWV = '.'.join(str(x) for x in [gMajorVersion.value, gMinorVersion1.value, gMinorVersion2.value, gMinorVersion3.value])
            print("\nFirmware version :\t", FWV)

        self.propertiesMenu()

    '''
        METHOD NAME : mouseCallBck
        DESCRIPTION : Sets position of the point from where depth value is to be taken
    '''  
    def mouseCallBck(self, event, x, y, flags, param):
        	if event == cv2.EVENT_LBUTTONDOWN:
                    livemouseptr = DepthPtr()
                    livemouseptr.X = x
                    livemouseptr.Y = y
                    if(self.setDepthPos(livemouseptr)<1):
                        print("\n Failed to set mouse position")

    '''
        METHOD NAME : getDepthValue
        DESCRIPTION : Lists the Depth value options available and gets the user input to transfer control
    '''  
    def getDepthValue(self):
        depthvalueMenuOptions = {
            0: self.main_menu_exit,
            1: self.propertiesMenu,
            2: self.__init__,
            3: self.getdepthvalue_,
            4: self.getdepthvaluecustom,
            }
        
        if(self.depthStreamStarted):
            while True:
                print("GET DEPTH VALUE MENU".center(100, "*"))
                print("\n\t0.Exit")
                print("\t1.Back")
                print("\t2.Main Menu")
                print("\t3.Centre")
                print("\t4.Custom Co-ordinate")

                choice = get_integer("\nPick a relevant Position :\t", min(depthvalueMenuOptions, key=int),max(depthvalueMenuOptions, key=int))
                func = depthvalueMenuOptions.get(choice, lambda: "Invalid Selection")

                if not func():
                    self.main_menu_exit()

        else:
            print("\nStart Depth Stream to get Depth value")
            self.propertiesMenu()

    '''
        METHOD NAME : getdepthvalue_
        DESCRIPTION : Prints the average depth value of centre of the frame
    '''
    def getdepthvalue_(self):
        avgDepth = ctypes.c_int()
        stdDepth = ctypes.c_int()
        avgIR = ctypes.c_int()
        stdIR = ctypes.c_int()
        if(self.getDepthIRValues(ctypes.byref(avgDepth), ctypes.byref(stdDepth), ctypes.byref(avgIR), ctypes.byref(stdIR)) < 1):
            print("\nFailed to get Depth value")
        else:
            print("\nDepth value :\t", avgDepth.value)
        self.propertiesMenu()

    '''
        METHOD NAME : getdepthvaluecustom
        DESCRIPTION : Prints the average depth value of the given position
    '''
    def getdepthvaluecustom(self):

        ptr = DepthPtr()

        x_pos = get_integer("\nEnter X co-ordinate between the range 17 to 624 :\t", 16,624, "\nInvalid Input")
        y_pos = get_integer("\nEnter Y co-ordinate between the range 17 to 464 :\t", 16,464, "\nInvalid Input")

        ptr.X = ctypes.c_int(int(x_pos)).value
        ptr.Y = ctypes.c_int(int(y_pos)).value

        if(self.setDepthPos(ptr) < 1):
            print("\n Failed to set position")
        else:
            self.getdepthvalue_()
        self.propertiesMenu()

    '''
        METHOD NAME : main_menu_exit
        DESCRIPTION : Exit call to close and deinitialize device
    '''
    def main_menu_exit(self):
        print("\nExit")
        if(self.GetNextFrameThread.is_alive()):
            self.modechange.set()
            self.Thread_end.set()
            if(self.thread_lock_flag == True):
                self.thread_lock.release()
            time.sleep(0.5)
            if(self.closeDeviceResult()==1):
                if(self.deinitializeResult()==1):
                    cv2.destroyAllWindows()
        exit(0)

if __name__ == '__main__':
    main = MainClass()

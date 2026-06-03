import cv2
import ctypes
from ctypes import*
import threading
import numpy as np
import time
from enum import Enum,IntEnum
from datetime import datetime
import sys
from input import get_integer, DeviceHandle

if(sys.platform == "linux"):
    sharedLibrary = "libDepthVistaSDK.so"
elif(sys.platform == "win32"):
    sharedLibrary = r"D:\Projects\TOF\MET\lataset\TOF\DepthVista\DepthVistaPython\DepthVistaSDK.dll"
depthVistaLib = ctypes.CDLL(sharedLibrary)

class iTOFControls:

    setDepthdenoise = depthVistaLib.SetDepthNoise
    setDepthdenoise.argtypes = [DeviceHandle, c_uint32]
    setDepthdenoise.restype = c_int

    getDepthdenoise = depthVistaLib.GetDepthNoise
    getDepthdenoise.argtypes = [DeviceHandle, POINTER(c_uint32)]
    getDepthdenoise.restype = c_int

    setConfidencethreshold = depthVistaLib.SetConfidenceThreshold
    setConfidencethreshold.argtypes = [DeviceHandle, c_uint16]
    setConfidencethreshold.restype = c_int

    getConfidencethreshold = depthVistaLib.GetConfidenceThreshold
    getConfidencethreshold.argtypes = [DeviceHandle, POINTER(c_uint16)]
    getConfidencethreshold.restype = c_int

    setIntegration = depthVistaLib.SetIntegrationTime
    setIntegration.argtypes = [DeviceHandle, c_uint16]
    setIntegration.restype = c_int

    getIntegration = depthVistaLib.GetIntegrationTime
    getIntegration.argtypes = [DeviceHandle, POINTER(c_uint16)]
    getIntegration.restype = c_int

    setIRgain = depthVistaLib.SetTOFIRGain
    setIRgain.argtypes = [DeviceHandle, c_uint16]
    setIRgain.restype = c_int

    getIRgain = depthVistaLib.GetTOFIRGain
    getIRgain.argtypes = [DeviceHandle, POINTER(c_uint16)]
    getIRgain.restype = c_int

    getLasertemp = depthVistaLib.GetLaserBoardTemperatureData
    getLasertemp.argtypes = [DeviceHandle, POINTER(c_float)]
    getLasertemp.restype = c_int

    getSensortemp = depthVistaLib.GetSensorDieTemperatureData
    getSensortemp.argtypes = [DeviceHandle, POINTER(c_float)]
    getSensortemp.restype = c_int

    getBasetemp = depthVistaLib.GetBaseBoardTemperatureData
    getBasetemp.argtypes = [DeviceHandle, POINTER(c_float)]
    getBasetemp.restype = c_int

    setspatial = depthVistaLib.SetDepthSpatialFilter
    setspatial.argtypes = [DeviceHandle, c_int]
    setspatial.restype = c_int

    def get_input(self, prompt, min_val, max_val):
        while True:
            try:
                user_input = int(input(prompt))
                if min_val <= user_input <= max_val:
                    return user_input
                else:
                    print("Invalid Value!!")
            except ValueError:
                print("Invalid input. Please enter a valid integer.")

    def depth_denoise_menu(self, deviceHandle):

        print("DEPTH DENOISE MENU".center(50, "*"))
        print("\n\t0.Exit")
        print("\t1.Back")
        print("\t2.Main Menu")
        print("\t3.Get Denoise")
        print("\t4.Set Denoise")

        choice = self.get_input("\nPick a relevant Depth Denoise mode:\t", 0, 4)
        if choice == 0:
            return False
        elif choice == 1 or choice == 2:
            return True
        elif choice == 3:
            denoise_value = c_uint32()
            result = self.getDepthdenoise(deviceHandle, byref(denoise_value))
            if result < 1:
                print("Failed to get depth denoise value")
            else:
                print(f"Current Depth Denoise Value: {denoise_value.value}")
            return True
        elif choice == 4:
            new_denoise_value = self.get_input("Enter the new depth denoise value (0 to 15):\t", 0, 15)
            result = self.setDepthdenoise(deviceHandle, new_denoise_value)
            if result == 0:
                print("Failed to set depth denoise value")
            else:
                print(f"Depth Denoise Value set to: {new_denoise_value}")
            return True
    
    def confidence_menu(self, deviceHandle):

        print("CONFIDENCE THRESHOLD MENU".center(50, "*"))
        print("\n\t0.Exit")
        print("\t1.Back")
        print("\t2.Main Menu")
        print("\t3.Get Confidence Threshold")
        print("\t4.Set Confidence Threshold")

        choice = self.get_input("\nPick a relevant confidence threshold mode:\t", 0, 4)
        if choice == 0:
            return False
        elif choice == 1 or choice == 2:
            return True
        elif choice == 3:
            conf_value = c_uint16()
            result = self.getConfidencethreshold(deviceHandle, byref(conf_value))
            if result < 1:
                print("Failed to get confidence threshold value")
            else:
                print(f"Current Confidence Threshold Value: {conf_value.value}")
            return True
        elif choice == 4:
            new_conf_value = self.get_input("Enter the new confidence threshold value (0 to 4095):\t", 0, 4095)
            result = self.setConfidencethreshold(deviceHandle, new_conf_value)
            if result == 0:
                print("Failed to set confidence threshold value")
            else:
                print(f"Confidence Threshold Value set to: {new_conf_value}")
            return True
        
    def integration_menu(self, deviceHandle):

        print("INTEGRATION TIME MENU".center(50, "*"))
        print("\n\t0.Exit")
        print("\t1.Back")
        print("\t2.Main Menu")
        print("\t3.Get Integration Time")
        print("\t4.Set Integration Time")

        choice = self.get_input("\nPick a relevant Integration Time mode:\t", 0, 4)
        if choice == 0:
            return False
        elif choice == 1 or choice == 2:
            return True
        elif choice == 3:
            integration_value = c_uint16()
            result = self.getIntegration(deviceHandle, byref(integration_value))
            if result < 1:
                print("Failed to get integration time value")
            else:
                print(f"Current Integration Time Value: {integration_value.value}")
            return True
        elif choice == 4:
            new_integration_value = self.get_input("Enter the new integration time value (0 to 850):\t", 0, 850)
            result = self.setIntegration(deviceHandle, new_integration_value)
            if result == 0:
                print("Failed to set integration time value")
            else:
                print(f"Integration Time Value set to: {new_integration_value}")
            return True
        
    def irgain_menu(self, deviceHandle):

        print("IR GAIN MENU".center(50, "*"))
        print("\n\t0.Exit")
        print("\t1.Back")
        print("\t2.Main Menu")
        print("\t3.Get IR Gain")
        print("\t4.Set IR Gain")

        choice = self.get_input("\nPick a relevant IR gain mode:\t", 0, 4)
        if choice == 0:
            return False
        elif choice == 1 or choice == 2:
            return True
        elif choice == 3:
            ir_value = c_uint16()
            result = self.getIRgain(deviceHandle, byref(ir_value))
            if result < 1:
                print("Failed to get IR Gain time value")
            else:
                print(f"Current IR Gain Value: {ir_value.value}")
            return True
        elif choice == 4:
            new_ir_value = self.get_input("Enter the new IR gain value (0 to 100):\t", 0, 100)
            result = self.setIRgain(deviceHandle, new_ir_value)
            if result == 0:
                print("Failed to set IR Gain value")
            else:
                print(f"IR Gain Value set to: {new_ir_value}")
            return True
        
    def temperature_menu(self, deviceHandle):

        print("GET TEMPERATURE DATA MENU".center(50, "*"))
        print("\n\t0.Exit")
        print("\t1.Back")
        print("\t2.Main Menu")
        print("\t3.Get Leaser Board Temperature")
        print("\t4.Get Module Board Temperature")
        print("\t5.Get Sensor Die Temperature")

        choice = self.get_input("\nPick a relevant IR gain mode:\t", 0, 5)
        if choice == 0:
            return False
        elif choice == 1 or choice == 2:
            return True
        elif choice == 3:
            lb_value = c_float()
            result = self.getLasertemp(deviceHandle, byref(lb_value))
            if result < 1:
                print("Failed to get laser board temperature value")
            else:
                print(f"Current Laser Board Temperature: {lb_value.value}")
            return True
        elif choice == 4:
            mb_value = c_float()
            result = self.getBasetemp(deviceHandle, byref(mb_value))
            if result < 1:
                print("Failed to get module board temperature value")
            else:
                print(f"Current Module Board Temperature: {mb_value.value}")
            return True
        elif choice == 5:
            sd_value = c_float()
            result = self.getSensortemp(deviceHandle, byref(sd_value))
            if result < 1:
                print("Failed to get sensor die temperature value")
            else:
                print(f"Current Sensor Die Temperature: {sd_value.value}")
            return True

    def spatial_menu(self, deviceHandle):

        print("SPATIAL FILTER MENU".center(50, "*"))
        print("\n\t0.Exit")
        print("\t1.Back")
        print("\t2.Main Menu")
        print("\t3.Spatial ON")
        print("\t4.Spatial OFF")

        choice = self.get_input("\nPick a relevant IR gain mode:\t", 0, 4)
        if choice == 0:
            return False
        elif choice == 1 or choice == 2:
            return True
        elif choice == 3 or choice == 4:
            value = choice - 3
            result = self.setspatial(deviceHandle, value)
            if result < 1:
                print("Failed to set spatial filter")
            else:
                print(f"Set Spatial Filter Success")
            return True

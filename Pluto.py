from plutoMultiwii import *      # Importing the plutoMultiwii module for interfacing with Pluto's Multiwii flight controller
from threading import Thread       # Importing the Thread class for creating threads
import time     

TRIM_MAX = 1000     # Maximum trim value
TRIM_MIN = -1000    # Minimum trim value
MSP_STATUS = 101	# Multiwii Serial Protocol status code

class pluto():   # Define the Pluto class
    def __init__(self): # Constructor method
        # Initialize default RC values
        self.rcRoll = 1500
        self.rcPitch = 1500
        self.rcThrottle = 1500
        self.rcYaw = 1500
        self.rcAUX1 = 1500
        self.rcAUX2 = 1000
        self.rcAUX3 = 1500
        self.rcAUX4 = 1000
        self.commandType = 0
        self.droneRC = [1500,1500,1500,1500,1500,1000,1500,1000]    # Drone RC values
        self.NONE_COMMAND = 0   # No command type
        self.TAKE_OFF = 1   # Takeoff command type
        self.LAND = 2   # Land command type
        self.thread = Thread(target=self.writeFunction) # Create a thread for writeFunction
        self.thread.start() # Start the thread
        
    def arm(self):  # Method to arm the drone
        print("Arming")
        self.rcRoll = 1500
        self.rcYaw = 1500
        self.rcPitch = 1500
        self.rcThrottle = 1000
        self.rcAUX4 = 1500
		#self.isAutoPilotOn = 0
    
    def box_arm(self):  # Method to arm the drone in "box arm" mode
        print("boxarm")
        self.rcRoll=1500
        self.rcYaw=1500
        self.rcPitch =1500
        self.rcThrottle = 1800
        self.rcAUX4 =1500
		#self.isAutoPilotOn = 0

    def disarm(self):    # Method to disarm the drone
        print("Disarm")
        self.rcThrottle = 1300
        self.rcAUX4 = 1200
        
    # Methods to control drone movement 
    def forward(self):   
        print("Forward")
        self.rcPitch = 1600

    def backward(self):
        print("Backward")
        self.rcPitch =1400

    def left(self):
        print("Left Roll")
        self.rcRoll =1400

    def right(self):
        print("Right Roll")
        self.rcRoll =1600

    def left_yaw(self):
        print("Left Yaw")
        self.rcYaw = 1300

    def right_yaw(self):
        print("Right Yaw")
        self.rcYaw = 1600

    def reset(self):     # Method to reset drone movement
        self.rcRoll =1500
        self.rcThrottle =1500
        self.rcPitch =1500
        self.rcYaw = 1500
        self.commandType = 0

    def increase_height(self):  # Method to increase drone height
        print("Increasing height")
        self.rcThrottle = 1800 

    def decrease_height(self): # Method to decrease drone height   
        print("Decreasing height")
        self.rcThrottle = 1300
    
    def take_off(self): # Method to take off
        self.disarm()
        self.box_arm()
        print("take off")
        self.commandType = 1

    def land(self): # Method to land
        self.commandType = 2

    def rcValues(self): # Method to get RC values
        return [self.rcRoll, self.rcPitch, self.rcThrottle, self.rcYaw,
        self.rcAUX1, self.rcAUX2, self.rcAUX3, self.rcAUX4]
    
    def trim_left_roll(self):   # Method to trim left roll
        print("Trimming Left Roll")
        self.rcRoll = max(TRIM_MIN, self.rcRoll + 100)  
        
    def writeFunction(self):    # Method for writing drone commands
        requests = list()
        # List of Multiwii Serial Protocol requests
        requests.append(MSP_RC)
        requests.append(MSP_ATTITUDE)
        requests.append(MSP_RAW_IMU)
        requests.append(MSP_ALTITUDE)
        requests.append(MSP_ANALOG)
        sendRequestMSP_ACC_TRIM()   # Send MSP ACC TRIM request

        while True:
            self.droneRC[:] = self.rcValues()   # Update drone RC values

            # Send MSP SET RAW RC request with updated RC values
            sendRequestMSP_SET_RAW_RC(self.droneRC)
            # print(self.droneRC)
            sendRequestMSP_GET_DEBUG(requests)   # Send MSP GET DEBUG request

            if (self.commandType != self.NONE_COMMAND): # If there is a command to be executed
                sendRequestMSP_SET_COMMAND(self.commandType)    # Send MSP SET COMMAND request
                self.commandType = self.NONE_COMMAND    # Reset command type

            time.sleep(0.022)   # Sleep for 0.022 seconds
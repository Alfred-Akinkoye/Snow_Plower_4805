# Import the pythonAPI files from their directory
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__))+"/PythonAPI")

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import math


class API:
    '''
    API class contains methods which allows our Python code to connect to 
    CoppeliaSim robot
    Some of this code was built from the CoppeliaSim API Example code
    This class also utalizes the CoppeliaSim pythonAPI exampele "sim"
    library for connecting to CoppeliaSim
    '''
    def __init__(self):
        self.clientID = None

    # --- Connection and Disconnections ---
    def connect(self):
        '''
        Connect to CoppeliaSim through the API server
        (This code was built from the CoppeliaSim API Example code)
        '''
        print ('Connecting to remote API server...')
        sim.simxFinish(-1) # just in case, close all opened connections
        self.clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
        if self.clientID != -1:
            print ('Connected to remote API server')
            return True
        else:
            print ('Failed connecting to remote API server')
        print ('Program ended')
        return False

    def disconnect(self):
        '''
        Disconnects from CoppeliaSim
        '''
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(self.clientID)
        # Now close the connection to CoppeliaSim:
        sim.simxFinish(self.clientID)
    
    def stopSimulation(self):
        '''
        Stops the simulation from python
        '''
        sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot)

    # --- Message Sending ---
    def sendMessage(self, message):
        '''
        Sends a message to CoppeliaSim
        '''
        sim.simxAddStatusbarMessage(self.clientID, message, sim.simx_opmode_oneshot)

    # --- Object Getting ---
    def getObject(self, name):
        '''
        Get CoppeliaSim object from the object handle
        '''
        return sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_blocking)[1]

    # --- Sensor Reading ---
    def readVisionSensor(self, name):
        '''
        Get vision sensor data from robot in CoppeliaSim
        '''
        return sim.simxReadVisionSensor(self.clientID, name, sim.simx_opmode_blocking)

    def readProximitySensor(self, name):
        '''
        Get proximity sensor data from robot in CoppeliaSim
        '''
        return sim.simxReadProximitySensor(self.clientID, name, sim.simx_opmode_blocking)

    # --- Localization Methods (GPS/IMU eqivalents) ---
    def getObjectPosition(self, handle):
        '''
        Positional Tracking (GPS). Returns the position of the object in CoppeliaSim
        '''
        return sim.simxGetObjectPosition(self.clientID, handle, -1, sim.simx_opmode_blocking)

    def getObjectOrientation(self, handle):
        '''
        Orientation Tracking (GPS/IMU). Returns the orientation of the object in CoppeliaSim
        '''
        return sim.simxGetObjectOrientation(self.clientID, handle, -1, sim.simx_opmode_blocking) 

    # -- Joint Setting Methods
    def setJointVelocity(self, joint, velocity):
        '''
        Sets the velocity of the robot's wheels
        Velocity should be given in m/s, this method
        converts to deg/s based on the models wheel
        diameter before sending to coppeliaSim
        '''
        wheelCircumfrence = math.pi * 0.12705 # The robot wheels are 0.12705m in diameter
        linearVelocity = velocity # Velocities in python program should be in m/s (Linear)
        angularVelocity = linearVelocity * ((2 * math.pi) / wheelCircumfrence) # Velocities passed to CoppeliaSim should be in rads/s
        return sim.simxSetJointTargetVelocity(self.clientID, joint, angularVelocity, sim.simx_opmode_oneshot)

    def setJointPosition(self, joint, position):
        '''
        Sets the position of the plow joints for unfolding the plow.
        '''
        return sim.simxSetJointTargetPosition(self.clientID, joint, position, sim.simx_opmode_oneshot)


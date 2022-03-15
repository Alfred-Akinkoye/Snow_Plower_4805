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
    def __init__(self):
        self.clientID = None

    def connect(self):
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
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(self.clientID)
        # Now close the connection to CoppeliaSim:
        sim.simxFinish(self.clientID)
    
    def stopSimulation(self):
        sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot)

    def disconnected(self):
        connectionID = sim.simxGetConnectionId(self.clientID)
        if (connectionID == -1):
            return True
        return False

    # ADD FUNCTIONS TO CALL API FUNCTIONS HERE (AS DESIRED)

    # Messages (Logging)
    def sendMessage(self, message):
        sim.simxAddStatusbarMessage(self.clientID, message, sim.simx_opmode_oneshot)

    # Get Object by Handle
    def getObject(self, name):
        return sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_blocking)[1]

    # Return the position of a joint (This is just a checker, or a limiter)
    def getJointPosition(self, joint):
        return sim.simxGetJointPosition(self.clientID, joint, sim.simx_opmode_blocking)[1]

    # Get vision sensor Data
    def readVisionSensor(self, name):
        return sim.simxReadVisionSensor(self.clientID, name, sim.simx_opmode_blocking)

    # Positional Tracking
    def getObjectPosition(self, handle):
        return sim.simxGetObjectPosition(self.clientID, handle, -1, sim.simx_opmode_blocking)

    def getObjectOrientation(self, handle):
        return sim.simxGetObjectOrientation(self.clientID, handle, -1, sim.simx_opmode_blocking) 

    def getObjectQuaternionOrientation(self, handle):
        return sim.simxGetObjectQuaternion(self.clientID, handle, -1, sim.simx_opmode_blocking)    

    # Movement
    def setJointVelocity(self, joint, velocity):
        wheelCircumfrence = math.pi * 0.12705 # Robot Wheels are 0.12705m in diameter
        linearVelocity = velocity # Velocities in python program should be in m/s (Linear)
        angularVelocity = linearVelocity * ((2 * math.pi) / wheelCircumfrence) # Velocities passed to CoppeliaSim should be in rads/s
        return sim.simxSetJointTargetVelocity(self.clientID, joint, angularVelocity, sim.simx_opmode_oneshot)

    def setJointPosition(self, joint, position):
        return sim.simxSetJointTargetPosition(self.clientID, joint, position, sim.simx_opmode_oneshot)


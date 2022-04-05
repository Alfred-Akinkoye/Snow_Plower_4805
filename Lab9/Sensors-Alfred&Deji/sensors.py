# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, include
# a corresponding call to simxFinish at the end!

# Import the pythonAPI files from their directory
from asyncio.windows_events import NULL
import sys
import os
import time
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

# Addition imports
import time
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

    # ADD FUNCTIONS TO CALL API FUNCTIONS HERE (AS DESIRED)
    def sendMessage(self, message):
        sim.simxAddStatusbarMessage(self.clientID, message, sim.simx_opmode_oneshot)

    def getObject(self, name):
        return sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_blocking)[1]

    def getJointPosition(self, joint):
        return sim.simxGetJointPosition(self.clientID, joint, sim.simx_opmode_blocking)[1]

    def setJointVelocity(self, joint, velocity):
        sim.simxSetJointTargetVelocity(self.clientID, joint, velocity, sim.simx_opmode_oneshot)

    def setJointPosition(self, joint, position):
        return sim.simxSetJointTargetPosition(self.clientID, joint, position, sim.simx_opmode_oneshot_wait)

    def readVisionSensor(self,name):
        return sim.simxReadVisionSensor(self.clientID,name,sim.simx_opmode_blocking)

class ProximitySensor:
    def __init__(self):
        self.api = API()

    def connectAPI(self):
        return self.api.connect()

    def run(self):
        # MODIFY CODE HERE
        print("Car reading...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")

        self.LeftJoint = self.api.getObject('LeftJoint')
        self.RightJoint=self.api.getObject('RightJoint')

        vision = []

        nominalv = 2
        self.api.setJointVelocity(self.LeftJoint,nominalv)
        self.api.setJointVelocity(self.RightJoint,nominalv)

        self.stop()

    def stop(self):
        self.api.disconnect()
        print("Stoped Sensors ...")

class VisionSensor:
    def __init__(self):
        self.api = API()

    def connectAPI(self):
        return self.api.connect()

    def run(self):
        # MODIFY CODE HERE
        print("Car reading...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")

        self.LeftJoint = self.api.getObject('LeftJoint')
        self.RightJoint=self.api.getObject('RightJoint')

         # Get vision example joint
        self.BackSensor = self.api.getObject('Back_IR')
        self.FrontSensor = self.api.getObject('Front_IR')
        self.LeftWheelSensor = self.api.getObject('Left_IR')
        self.RightWheelSensor = self.api.getObject('Right_IR')
        vision = [self.FrontSensor,self.LeftWheelSensor,self.RightWheelSensor,self.BackSensor]

        nominalv = 2
        self.api.setJointVelocity(self.LeftJoint,nominalv)
        self.api.setJointVelocity(self.RightJoint,nominalv)
        while (True):
            sensor_values = [0,0,0,0]
            # for i in range(0,4):
            [temp,detectionState,data] = self.api.readVisionSensor(vision[0]);
            print(data[0][11])
            if data[0][11] < 0.3:
                rightv = nominalv
                leftv  = nominalv
                print("out of bounds")
                self.api.setJointVelocity(self.LeftJoint,nominalv*0)
                self.api.setJointVelocity(self.RightJoint,nominalv*0)
                break

                # if (detectionState[0] < 0):
                #     if detectionState[2][11] < 0.3:
                #         sensor_values[i] = 1
                #     else:
                #         sensor_values[i] = 0
        self.stop()

    def stop(self):
        self.api.disconnect()
        print("Stoped Sensors ...")

if __name__ == '__main__':
    print ('Program started')
    sensor = VisionSensor()
    if (sensor.connectAPI()):
        sensor.run()

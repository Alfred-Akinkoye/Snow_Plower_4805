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

class Plower:
    def __init__(self):
        self.api = API()

    def connectAPI(self):
        return self.api.connect()
    
    def getObjectHandles(self):
        # get motor joints
        self.LeftJoint = self.api.getObject("LeftJoint")
        self.RightJoint = self.api.getObject("RightJoint")

        # get plow control joints
        self.RotatorL = self.api.getObject("RotatorL")
        self.RotatorR = self.api.getObject("RotatorR")

    def run(self):
        # MODIFY CODE HERE

        print("Plower Running...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")

         # Get plow example joint
        joint = self.api.getObject('Revolute_joint')
        direction_forward = True

        while (True):
            if (self.api.getJointPosition(joint) > math.pi/2):
                direction_forward = False

            if (self.api.getJointPosition(joint) < 0):
                direction_forward = True
            
            if (direction_forward):
                self.api.setJointVelocity(joint, math.pi/10)
            else:
                self.api.setJointVelocity(joint, -math.pi/10)
        
        self.stop()

    def unfoldPlow(self):

        print(self.api.setJointPosition(self.RotatorL, math.pi/2))
        print(self.api.setJointPosition(self.RotatorR, math.pi/2))

    def foldPlow(self):

        print(self.api.setJointPosition(self.RotatorL, 0))
        print(self.api.setJointPosition(self.RotatorR, 0))

    def setMove(self, speed):
        print(self.api.setJointVelocity(self.LeftJoint,speed))
        print(self.api.setJointVelocity(self.RightJoint,speed))

    def setStop(self):
        print(self.api.setJointVelocity(self.LeftJoint,0))
        print(self.api.setJointVelocity(self.RightJoint,0))

    def inPlaceRotation(self, direction, speed):
        self.api.setJointVelocity(self.LeftJoint, speed * direction)
        self.api.setJointVelocity(self.RightJoint, speed * direction * -1)
    
    def stop(self):
        self.api.disconnect()
        print("Plower stopped.")

if __name__ == '__main__':
    print ('Program started')
    plower = Plower()
    if (plower.connectAPI()):
        plower.getObjectHandles()
        print("Unfolding")
        plower.unfoldPlow()
        time.sleep(5)
        print("Folding")
        plower.foldPlow()







### ---Example Code from Lab2--- ###
#     [returnCode,Left_Joint]=sim.simxGetObjectHandle(clientID,'Left_Motor',sim.simx_opmode_blocking)
#     [returnCode,Right_Joint]=sim.simxGetObjectHandle(clientID,'Right_Motor',sim.simx_opmode_blocking)

#     vision = [Left_Joint,Left_Joint,Left_Joint];

#     [returnCode,vision[0]] = sim.simxGetObjectHandle(clientID,'Left_Sensor',sim.simx_opmode_blocking)
#     [returnCode,vision[1]] = sim.simxGetObjectHandle(clientID,'Middle_Sensor',sim.simx_opmode_blocking)
#     [returnCode,vision[2]] = sim.simxGetObjectHandle(clientID,'Right_Sensor',sim.simx_opmode_blocking)

#     nominalv = -1
#     while(True):
#         sensor_values = [0,0,0];
#         for i in range(0,3):
#             [returnCode,detectionState,data] = sim.simxReadVisionSensor(clientID,vision[i],sim.simx_opmode_blocking);
#             #print(returnCode)
#             #print(detectionState)
#             #print(data)
#             if (detectionState > -1):
#                 if data[0][11] < 0.3:
#                     sensor_values[i] = 1;
#                 else:
#                     sensor_values[i] = 0;

#         rightv = nominalv;
#         leftv  = nominalv;
#         if(sensor_values[0] == 1):
#             leftv = 0;
#         if(sensor_values[2] == 1):
#             rightv = 0;

#         returnCode = sim.simxSetJointTargetVelocity(clientID,Left_Joint,leftv,sim.simx_opmode_oneshot);
#         returnCode = sim.simxSetJointTargetVelocity(clientID,Right_Joint,rightv,sim.simx_opmode_oneshot);
### --- End Example Code --- ###
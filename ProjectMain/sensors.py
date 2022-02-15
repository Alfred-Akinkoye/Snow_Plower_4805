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


class Sensor:
    def __init__(self):
        self.api = API()

    def connectAPI(self):
        return self.api.connect()
    
    def run(self):
        # MODIFY CODE HERE
        print("Car reading...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")

        [returnCode,Left_Joint]=sim.simxGetObjectHandle(clientID,'LeftJoint',sim.simx_opmode_blocking)
        [returnCode,Right_Joint]=sim.simxGetObjectHandle(clientID,'RightJoint',sim.simx_opmode_blocking)

         # Get vision example joint
        vision = [NULL,NULL,NULL,NULL]
        [returnCode,vision[0]] = sim.simxGetObjectHandle(clientID,'BackSensor',sim.simx_opmode_blocking)
        [returnCode,vision[1]] = sim.simxGetObjectHandle(clientID,'FrontSensor',sim.simx_opmode_blocking)
        [returnCode,vision[2]] = sim.simxGetObjectHandle(clientID,'LeftWheelSensor',sim.simx_opmode_blocking)
        [returnCode,vision[3]] = sim.simxGetObjectHandle(clientID,'RightWheelSensor',sim.simx_opmode_blocking)

        nominalv = -1

        while (True):
            sensor_values = [0,0,0,0]
            for i in range(0,4):
                [returnCode,detectionState,data] = sim.simxReadVisionSensor(clientID,vision[i],sim.simx_opmode_blocking);
                #print(returnCode)
                #print(detectionState)
                #print(data)
                if (detectionState > -1):
                    if data[0][11] < 0.3:
                        sensor_values[i] = 1
                    else:
                        sensor_values[i] = 0
            
            rightv = nominalv
            leftv  = nominalv
            if(sensor_values[0] == 1):
                leftv = 1
                rightv = 1
            if(sensor_values[2] == 1):
                rightv = 1

            returnCode = sim.simxSetJointTargetVelocity(clientID,Left_Joint,leftv,sim.simx_opmode_oneshot);
            returnCode = sim.simxSetJointTargetVelocity(clientID,Right_Joint,rightv,sim.simx_opmode_oneshot);
        self.stop()
    
    def stop(self):
        self.api.disconnect()
        print("Stoped Sensors ...")

if __name__ == '__main__':
    print ('Program started')
    sensor = Sensor()
    if (sensor.connectAPI()):
        sensor.run()

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
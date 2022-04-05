# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

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

import time

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')


    ### --- Wrtie all Main Running Code Here (This will be called once) ---

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello from Python!',sim.simx_opmode_oneshot)


    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)
    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')




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
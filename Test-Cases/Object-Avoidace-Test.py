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

# Import Local Classes
from api import API
from movementControl import MovementControl
from sensors import Sensors

# Addition imports
import time
import math


class OA_Test:
    def __init__(self):
        self.api = API()
        self.isEast = True
        self.isNorth = True
        self.hMove = True
        self.outBoundState = False    #True if out of bounds, false if in bounds
        self.onLine = False

    def connectAPI(self):
        if (self.api.connect()):
            self.movementControl = MovementControl(self, self.api)
            self.sensors = Sensors(self, self.api)

            self.leftPlowJoint = self.api.getObject("LeftPlowJoint")
            self.rightPlowJoint = self.api.getObject("RightPlowJoint")

            return True
        return False

    def run(self):
        # MODIFY CODE HERE

        print("Plower Running...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")
        self.unfoldPlow()
        self.movementControl.setVelocity(0.5)
        while True:
            # check if plower will collide with and object
            if (self.sensors.checkProxyArray("Front", 0.8)):
                self.api.sendMessage("Proxy triggered")
                self.objectAvoidance()

        self.stop()

    def objectAvoidance(self):
        '''

        '''
        #need to make recursive to help with multiple obstacles
        #need to make option to switch to overall N/S travel

        #setup
        self.movementControl.setVelocity(0)
        if(self.isEast):
            direct = "Left"
            facing = "E"
        else:
            direct = "Right"
            facing = "W"
        #Go south until obstacle is cleared
        self.movementControl.rotateTo("S",self.isEast)
        origin = self.movementControl.getPlowerPosition()
        self.movementControl.setVelocity(0.5)

        while(self.sensors.checkProxyArray(direct, 0.8) and not self.enteredLine()):
            continue
        if (self.onLine):
            self.edgeControl()
            return

        self.movementControl.setVelocity(0)

        # continue in direction until obstacle is cleared
        self.movementControl.rotateTo(facing, not self.isEast)
        self.movementControl.setVelocity(0.5)
        while(self.sensors.checkProxyArray(direct,1.5) and not self.enteredLine()):
            continue
        if (self.onLine):
            self.edgeControl()
            return
        self.movementControl.setVelocity(0)

        #head north until at original poisition
        self.movementControl.rotateTo("N", not self.isEast)
        # Should probably check for the edge while we do this move as well somehow
        self.movementControl.move(self.movementControl.getPlowerPositionDifference(origin, "pos-y"))
        self.movementControl.rotateTo(facing, self.isEast)

        self.movementControl.accelSetVelocity(1)

    def stop(self):
        '''
        This is a method run at the end of the simulation, it will
        stop the running simulation and disconnect sim module
        '''
        print("Stopping...")
        # stop the simulation
        self.api.stopSimulation()
        print("Simulation Stopped")
        self.api.disconnect()
        print("Plower Disconnected")

    # Plow Control Functions
    def unfoldPlow(self):
        '''
        Simple method to unfold the plow to deploy it
        '''
        # Rotate each joint 90 degress to drop plows into position
        self.api.setJointPosition(self.leftPlowJoint, math.pi/2)
        self.api.setJointPosition(self.rightPlowJoint, -math.pi/2)
        time.sleep(1)

    def foldPlow(self):
        '''
        Method that rotates the plow back into upright position
        Used to minimize surface area
        '''
        self.api.setJointPosition(self.leftPlowJoint, 0)
        self.api.setJointPosition(self.rightPlowJoint, 0)
        time.sleep(1)

    def enteredLine(self):
        if (self.sensors.checkFrontVisionSensor() and not self.onLine):
            self.onLine = True
            return True
        else:
            self.onLine = False
            return False

# This is the Main Script
if __name__ == '__main__':
    print ('Program started')
    OA = OA_Test()
    if (OA.connectAPI()):
        try:
            OA.run()
        except KeyboardInterrupt:
            OA.stop()
            #print(plower.ra.checkDiffTimes)
        except Exception as e:
            OA.stop()
            raise e
    # End Program Execution
    print("Exiting")

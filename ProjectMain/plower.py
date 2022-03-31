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

from randomAlgorithm import RandomAlgorithm

# Addition imports
import time
import math


class Plower:
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
        #Inital start code
        self.movementControl.move(1)
        self.unfoldPlow()
        self.movementControl.rotateTo("E", True)
        # self.movementControl.getPlowerOrientation()
        self.movementControl.accelSetVelocity(1)

        #print(self.movementControl.getPlowerPosition())
        while True:
            # check if plower will collide with and object
            if (self.sensors.checkProxyArray("Front", 1.2)):
                self.objectAvoidance()
            # if plower has left area, run edge control
            # to clear next level of map
            if (self.enteredLine()):
                self.edgeControl()
                
        self.stop()

    def enteredLine(self):
        if (self.sensors.checkFrontVisionSensor() and not self.onLine):
            self.onLine = True
            return True
        else:
            self.onLine = False
            return False

    def edgeControl(self):
        '''
        Code to have the plower move up one level of the map
        and continue plowing
        '''
        print(f"In edge control going {'in' if self.outBoundState else 'out'}")
        # will flip to outbound while crossing out of bounds
        # while will flip to inbound when plower crosses back
        # in bound
        self.outBoundState = not self.outBoundState
        # if plower is out of bound
        if(self.outBoundState):
            self.movementControl.decelStop()
            self.movementControl.move(1)
            # fold plow to prevent moving out of bound snow
            self.foldPlow()

            # if plower was going east, move up one meter,
            # then continue west, vice versa if originally
            # going west
            if(self.isEast):
                
                #self.movementControl.move(0.1)
                self.movementControl.rotateTo("N",False)
                self.movementControl.move(1)
                self.movementControl.rotateTo("W",False)
            else:
                #self.movementControl.move(0.1)
                self.movementControl.rotateTo("N",True)
                self.movementControl.move(1)
                self.movementControl.rotateTo("E",True)
            self.unfoldPlow()
            # flip is east because now moving in opposite direction
            self.isEast = not self.isEast

            self.movementControl.setVelocity(0.5)
        else:
            print("Increasing Velocity")
            self.movementControl.setVelocity(1)

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

        while(self.sensors.checkProxyArray(direct, 1.2) and not self.enteredLine()):
            continue
        if (self.onLine):
            self.edgeControl()
            return

        self.movementControl.setVelocity(0)

        # continue in direction until obstacle is cleared
        self.movementControl.rotateTo(facing, not self.isEast)
        self.movementControl.setVelocity(0.5)
        while(self.sensors.checkProxyArray(direct,2) and not self.enteredLine()):
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

# This is the Main Script
if __name__ == '__main__':
    print ('Program started')
    plower = Plower()
    if (plower.connectAPI()):
        try:
            plower.run()
        except KeyboardInterrupt:
            plower.stop()
            #print(plower.ra.checkDiffTimes)
        except Exception as e:
            plower.stop()
            raise e
    # End Program Execution
    print("Exiting")

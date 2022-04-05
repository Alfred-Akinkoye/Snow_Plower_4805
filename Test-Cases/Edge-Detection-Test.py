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


class Edge_Test:
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

        self.movementControl.accelSetVelocity(0.5)

        #print(self.movementControl.getPlowerPosition())
        while True:
            if (self.enteredLine()):
                self.edgeControl()
                break
        time.sleep(5)
        self.stop()

    def enteredLine(self):
        if (self.sensors.checkFrontVisionSensor() and not self.onLine):
            self.onLine = True
            return True
        else:
            self.onLine = False
            return False

    def edgeControl(self,movement = True):
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
            if(movement):
                self.movementControl.decelStop()
                self.movementControl.move(0.8)
            # fold plow to prevent moving out of bound snow
            self.foldPlow()

            # if plower was going east, move up one meter,
            # then continue west, vice versa if originally
            # going west
            if(self.isEast):

                #self.movementControl.move(0.1)
                self.movementControl.rotateTo("N")
                self.movementControl.move(1)
                self.movementControl.rotateTo("W")
            else:
                #self.movementControl.move(0.1)
                self.movementControl.rotateTo("N")
                self.movementControl.move(1)
                self.movementControl.rotateTo("E")
            self.unfoldPlow()
            # flip is east because now moving in opposite direction
            self.isEast = not self.isEast
            self.movementControl.setVelocity(0.5)

        print("is east is " + str(self.isEast))
        print("outbound is " + str(self.outBoundState))

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
    edge = Edge_Test()
    if (edge.connectAPI()):
        try:
            edge.run()
        except KeyboardInterrupt:
            edge.stop()
            #print(plower.ra.checkDiffTimes)
        except Exception as e:
            edge.stop()
            raise e
    # End Program Execution
    print("Exiting")

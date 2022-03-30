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


class Plower:
    def __init__(self):
        self.api = API()
        self.isEast = True
        self.outBoundState = False    #true if inbound, false if out of bounds
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
        print(self.movementControl.getPlowerOrientation())
        self.unfoldPlow()

        self.movementControl.setVelocity(0.55)

        # Paragraph of code below is for testing purposes. Comment out or delete
        # when finished
        self.movementControl.rotateTo("N",True)
        self.movementControl.move(1)
        self.movementControl.rotateTo("W",False)
        self.movementControl.move(1)
        # self.movementControl.rotateTo("S",False)
        # self.movementControl.move(1)
        # self.movementControl.rotateTo("E",False)
        # self.movementControl.move(1)
        # self.movementControl.rotateTo("N",False)
        # self.movementControl.move(1)
        # self.movementControl.rotateTo("E",True)
        # self.movementControl.move(1)
        # self.movementControl.rotateTo("S",True)
        # self.movementControl.move(1)
        # self.movementControl.rotateTo("W",True)
        # self.movementControl.move(1)
        # self.movementControl.rotateTo("N",True)

        while True:
            #self.sensors.F_Proximity.getDistance()
            if(self.sensors.checkAllVisionSensors()):
                time.sleep(2)
                print("sensors flared up")
                self.movementControl.stop()
                self.edgeControl()
                self.movementControl.setVelocity(0.55)
        self.stop()

    def edgeControl(self):
        print("In edge control")
        self.outBoundState = not self.outBoundState
        print(self.outBoundState)
        if(self.outBoundState):
            self.foldPlow()
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
            self.isEast = not self.isEast
        print(self.movementControl.getPlowerOrientation())



    def stop(self):
        print("Stopping...")
        # stop the simulation
        self.api.stopSimulation()
        print("Simulation Stopped")
        self.api.disconnect()
        print("Plower Disconnected")

    # Plow Control Functions
    def unfoldPlow(self):
        self.api.setJointPosition(self.leftPlowJoint, math.pi/2)
        self.api.setJointPosition(self.rightPlowJoint, -math.pi/2)
        time.sleep(1)

    def foldPlow(self):
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
        except Exception as e:
            plower.stop()
            raise e
    # End Program Execution
    print("Exiting")

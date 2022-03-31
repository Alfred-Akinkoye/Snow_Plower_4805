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
        self.isNorth = True
        self.hMove = True
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
        #Inital start code
        # self.movementControl.move(1)
        # self.movementControl.rotateTo("E",True)
        # self.movementControl.getPlowerOrientation()
        self.unfoldPlow()
        self.movementControl.setVelocity(0.55)
        print(self.movementControl.getPlowerPosition())
        while True:
             if (self.sensors.checkProxyArray("Front", 0.8)):
                 self.objectAvoidance()
                 self.movementControl.setVelocity(0.55)
             if(self.sensors.checkFrontVisionSensor()):
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
        #print(self.movementControl.getPlowerOrientation())
    
    def objectAvoidance(self):
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
        self.movementControl.setVelocity(0.4)
        while(self.sensors.checkProxyArray(direct,1)):
            continue
        self.movementControl.setVelocity(0)

        # continue in direction until obstacle is cleared
        self.movementControl.rotateTo(facing, not self.isEast)
        self.movementControl.setVelocity(0.55)
        while(self.sensors.checkProxyArray(direct,1.1)):
            continue
        self.movementControl.setVelocity(0)

        #head north until at original poisition
        self.movementControl.rotateTo("N", not self.isEast)
        self.movementControl.move(self.movementControl.getPlowerPositionDifference(origin,"y"))
        self.movementControl.rotateTo(facing, self.isEast)

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

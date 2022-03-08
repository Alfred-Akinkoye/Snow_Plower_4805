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
        
    def connectAPI(self):
        if (self.api.connect()):
            self.movementControl = MovementControl(self, self.api)
            self.sensors = Sensors(self, self.api)
            return True
        return False
    
    def linkObjectHandles(self):
        # get plow control joints
        self.plowLeftJoint = self.api.getObject("LeftPlowJoint")
        self.plowRightJoint = self.api.getObject("RightPlowJoint")
        



    def run(self):
        # MODIFY CODE HERE

        print("Plower Running...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")

        self.linkObjectHandles()

        print(self.movementControl.getObjectPosition())
        self.movementControl.timedMove(2, 2)

        print("Unfolding Plow")
        self.unfoldPlow()
        print(self.movementControl.getObjectPosition())
        self.movementControl.timedRotate(2, 2)

        self.movementControl.setVelocity(4)

        while True:
            time.sleep(8)
            print(self.movementControl.getObjectPosition())
            self.movementControl.timedRotate(2, 2.3)
            self.movementControl.setVelocity(4)
    
    def stop(self):
        print("Stopping...")
        # stop the simulation
        self.api.stopSimulation()
        print("Simulation Stopped")
        self.api.disconnect()
        print("Plower Disconnected")
        




    # Plow Control Functions
    def unfoldPlow(self):
        self.api.setJointPosition(self.plowLeftJoint, math.pi/2)
        self.api.setJointPosition(self.plowRightJoint, -math.pi/2)
        time.sleep(2)

    def foldPlow(self):
        self.api.setJointPosition(self.plowLeftJoint, 0)
        self.api.setJointPosition(self.plowRightJoint, 0)
        time.sleep(2)





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
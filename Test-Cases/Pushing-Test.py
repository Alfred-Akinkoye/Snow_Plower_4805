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

# Addition imports
import time
import math
from api import API

class PlowDeployment:
    def __init__(self):
        self.api = API()

    def connectAPI(self):
        if self.api.connect():
            self.leftPlowJoint = self.api.getObject("LeftPlowJoint")
            self.rightPlowJoint = self.api.getObject("RightPlowJoint")
            return True
        return False


    def run(self):
        # MODIFY CODE HERE
        print("Car reading...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")

        self.unfoldPlow()
        print("Plow Unfolded")
        self.api.sendMessage("Plow Unfolded")
        self.api.setJointVelocity(self.LeftJoint,0.5)
        self.api.setJointVelocity(self.RightJoint,0.5)
        time.sleep(8)
        self.stop()

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
        print("Disconnected")

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


# This is the Main Script For testing
if __name__ == '__main__':
    print ('Program started')
    plow = PlowDeployment()
    if (plow.connectAPI()):
        try:
            plow.run()
        except KeyboardInterrupt:
            plow.stop()
            #print(plower.ra.checkDiffTimes)
        except Exception as e:
            sensor.stop()
            raise e
    # End Program Execution
    print("Exiting")

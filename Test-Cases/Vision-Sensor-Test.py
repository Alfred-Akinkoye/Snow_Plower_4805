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

class VisionSensor:
    def __init__(self):
        self.api = API()

    def connectAPI(self):
        if self.api.connect():
            self.LeftJoint = self.api.getObject('LeftJoint')
            self.RightJoint=self.api.getObject('RightJoint')

             # We are only using the front vision sensor
            self.FrontSensor = self.api.getObject('Front_IR')
            return True
        return False

    def run(self):
        # MODIFY CODE HERE
        print("Car reading...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")

        nominalv = 0.25
        self.api.setJointVelocity(self.LeftJoint,nominalv)
        self.api.setJointVelocity(self.RightJoint,nominalv)
        while (True):
            [temp,detectionState,data] = self.api.readVisionSensor(self.FrontSensor);
            print(data[0][11])
            if (data and len(data) > 0 and len(data[0]) > 11):
                if data[0][11] < 0.1 and data[0][11] > 0:
                    print("Boundary Case Met")
                    self.api.sendMessage("Boundary Case Met")
                    self.api.setJointVelocity(self.LeftJoint,0)
                    self.api.setJointVelocity(self.RightJoint,0)
                    break
            #simply here to let us see the stop before reseting
        time.sleep(10)
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


# This is the Main Script For testing
if __name__ == '__main__':
    print ('Program started')
    sensor = VisionSensor()
    if (sensor.connectAPI()):
        try:
            sensor.run()
        except KeyboardInterrupt:
            sensor.stop()
            #print(plower.ra.checkDiffTimes)
        except Exception as e:
            sensor.stop()
            raise e
    # End Program Execution
    print("Exiting")

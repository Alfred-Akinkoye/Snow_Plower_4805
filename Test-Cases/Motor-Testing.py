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
from movementControl import MovementControl

class Motor_Test:
    def __init__(self):
        self.api = API()

    def connectAPI(self):
        if self.api.connect():
            self.movementControl = MovementControl(self, self.api)
            return True
        return False


    def run(self):
        # MODIFY CODE HERE
        print("Car reading...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")

        self.api.sendMessage("Accelerating")
        self.movementControl.accelSetVelocity(0.5)
        time.sleep(3)
        self.api.sendMessage("Stopped")
        self.movementControl.stop()
        time.sleep(3)
        self.api.sendMessage("Turn Right")
        self.movementControl.turnRight()
        time.sleep(3)
        self.api.sendMessage("Moving 1 meter foward")
        self.movementControl.move(1)
        time.sleep(2)
        self.api.sendMessage("Turn Left")
        self.movementControl.turnLeft()
        time.sleep(2)
        self.api.sendMessage("Face East")
        self.movementControl.rotateTo("E")
        time.sleep(2)
        self.api.sendMessage("Face South")
        self.movementControl.rotateTo("S")
        time.sleep(2)
        self.api.sendMessage("Face West")
        self.movementControl.rotateTo("W")
        time.sleep(2)
        self.api.sendMessage("Face North")
        self.movementControl.rotateTo("N")
        time.sleep(7)
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
    motor = Motor_Test()
    if (motor.connectAPI()):
        try:
            motor.run()
        except KeyboardInterrupt:
            motor.stop()
            #print(plower.ra.checkDiffTimes)
        except Exception as e:
            motor.stop()
            raise e
    # End Program Execution
    print("Exiting")

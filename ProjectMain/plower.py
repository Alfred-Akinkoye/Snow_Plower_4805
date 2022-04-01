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

# Import Local Classes
from api import API
from movementControl import MovementControl
from sensors import Sensors

# Addition imports
#import time
import math


class Plower:
    '''
    The main class for the Plower. This class contains the main algorithm for the plower's brain.
    The Plower Class can be instanciated, connected to the CoppeliaSim API and run.
    The Plower Class creates it's own connection to CoppeliaSim through the API class and controls
    it's movement and sensor readings through the MovementControl and Sensors classes.
    '''
    def __init__(self):
        self.api = API()
        self.isEast = True
        self.isNorth = True
        self.hMove = True
        self.outBoundState = False    # True if out of bounds, false if in bounds
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

        # Inital start code
        self.movementControl.move(1)
        self.unfoldPlow()
        self.movementControl.rotateTo("E")
        self.movementControl.setVelocity(1)

        while True:
            self.movementControl.setVelocity(1)
            # check if plower will collide with and object
            if (self.sensors.checkProxyArray("Front", 1.1)):
                self.objectAvoidance()
                if (not self.sensors.checkProxyArray("Front", 1.1)):
                    self.movementControl.setVelocity(0.5)

            # if plower has left area, run edge control
            # to clear next level of map
            if (self.enteredLine()):
                self.edgeControl()
                if (not self.sensors.checkProxyArray("Front", 1.1)):
                    self.movementControl.setVelocity(0.5)
            
                
        self.stop()

    def enteredLine(self):
        # Check if we are on the line
        if (self.sensors.checkFrontVisionSensor()):
            # If we are already on the line return false
            if (self.onLine):
                return False
            else:
                self.onLine = True
                return True
        else:
            self.onLine = False
            return False

    def edgeControl(self, movement=True):
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
                self.movementControl.stop()
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
            # flip is east because now moving in opposite direction
            self.isEast = not self.isEast
        else:
            # redeploy plow when reentering area
            self.unfoldPlow()
            
        print("is east is " + str(self.isEast))
        print("outbound is " + str(self.outBoundState))

    def objectAvoidance(self):
        '''
        
        '''
        #need to make recursive to help with multiple obstacles
        #need to make option to switch to overall N/S travel
        
        #setup
        print("Entering OA")
        self.movementControl.setVelocity(0)
        if(self.isEast):
            direct = "Left"
            facing = "E"
        else:
            direct = "Right"
            facing = "W"
        # Go south until obstacle is cleared
        self.movementControl.rotateTo("S", swing=True)
        origin = self.movementControl.getPlowerPosition()
        self.movementControl.setVelocity(0.75)
        print("OA Moving South")
        edgeMove = self.OAloop(facing,direct)

        self.movementControl.setVelocity(0)
        print("OA Moving E/W")
        # Continue in direction until obstacle is cleared
        self.movementControl.rotateTo(facing, swing=True)
        self.movementControl.setVelocity(0.75)
        edgeAdjust = self.OAloop(facing, direct)
        
        self.movementControl.setVelocity(0)

        # Head north until at original poisition
        print("OA returning North")
        self.movementControl.rotateTo("N", swing=True)
        # Should probably check for the edge while we do this move as well somehow
        self.movementControl.move(self.movementControl.getPlowerPositionDifference(origin, "pos-y")) # Modify pos-y to a variable getting the axis at start and going 90 deg off 

        if(edgeMove or edgeAdjust):
            print("OA Entering Edge Control")
            self.edgeControl(False)
            print("OA Exiting Edge Control")
            if(self.isEast):
                direct = "Left"
                facing = "E"
            else:
                direct = "Right"
                facing = "W"
        self.movementControl.rotateTo(facing)


    def OAloop(self,facing,direction):
        """
        Waiting loop used in OA to detect if obstacle was cleared by plow
        Also has if conditional to check if edge detection has occured during loop
        """
        edgeAdjust = False
        while(self.sensors.checkProxyArray(direction,1.5)):
            # Edge Detection during OA
            if (self.enteredLine() and not edgeAdjust):
                edgeAdjust = True
                if (facing != self.movementControl.getPlowerDirection()):
                    self.movementControl.setVelocity(0)
                    self.movementControl.rotateTo(facing, swing=True)
                    self.movementControl.setVelocity(0.75)

            # Recursive OA here

        return edgeAdjust

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
        #time.sleep(1)

    def foldPlow(self):
        '''
        Method that rotates the plow back into upright position
        Used to minimize surface area
        '''
        self.api.setJointPosition(self.leftPlowJoint, 0)
        self.api.setJointPosition(self.rightPlowJoint, 0)
        #time.sleep(1)

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

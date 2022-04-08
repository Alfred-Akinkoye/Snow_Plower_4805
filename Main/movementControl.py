import time
import math

class MovementControl():
    '''
    All movement of the plow is controlled in this class
    The various methods described here can be called from
    the main plower class.
    '''
    def __init__(self, plow, api):
        self.plow = plow
        self.api = api

        self.LeftWheel = Wheel(api, 'LeftJoint')
        self.RightWheel = Wheel(api, 'RightJoint')
        self.plowerOb = self.api.getObject("Plower")

    # --- Basic Movement Functions ---
    def setVelocity(self, speed):
        '''
        Set both wheels to rotate at a certain speed
        based on given input
        '''
        self.LeftWheel.setVelocity(speed)
        self.RightWheel.setVelocity(speed)

    def rotate(self, speed, swing=False):
        '''
        Rotates the plower in place at a rate based
        on the speed given to the method
        if swing is true the rotation is done by
        only turning one wheel, swinging the plow
        '''
        if not swing:
            self.LeftWheel.setVelocity(-speed)
            self.RightWheel.setVelocity(speed)
        else:
            if (speed > 0):
                #CCW
                self.LeftWheel.setVelocity(0)
                self.RightWheel.setVelocity(2*abs(speed))
            else:
                #CW
                self.RightWheel.setVelocity(0)
                self.LeftWheel.setVelocity(2*abs(speed))
                
    def stop(self):
        '''
        Stops all motor movement in the plower
        '''
        self.LeftWheel.stop()
        self.RightWheel.stop()

    # --- Movement Methods ---
    def rotateTo(self, orientation, swing=False):
        """
        Rotate plower into specific cardinal direction 
        (This method now takes the shortest direction)
        Args:
        orientation = enter N,E,S, or W for direction to turn to
        swing = true if the rotations should be swings
        """
        od = {"N":0, "E": -math.pi/2, "S": math.pi, "W": math.pi/2}
        target = od[orientation]

        # Determine shortest direction
        difference = self.getPlowerOrientationDifference(target)
        prevDifferenceSign = difference / abs(difference)

        # Rotate until the threshold is reached at the given speed
        self.rotateAtSpeedUntilThreshold(0.3, 0.4, target, swing) # math.pi/9 = 0.348

        self.rotateAtSpeedUntilThreshold(0.05, 0.2, target, swing)

        self.rotateAtSpeedUntilThreshold(0.02, 0.02, target, swing)

        self.stop()

    def rotateAtSpeedUntilThreshold(self, speed, threshold, target, swing=False):
        '''
        Rotates the robot at the given speed until the orientation
        is within the given threshold of the target
        This method stops overshoot by rotating back
        '''
        difference = self.getPlowerOrientationDifference(target)
        prevDifferenceSign = 0
        
        while(abs(difference) > threshold):
            difference = self.getPlowerOrientationDifference(target)
            differenceSign = math.copysign(1, difference)
            # If we have overshot we switch directions
            if prevDifferenceSign == 0 or not differenceSign == prevDifferenceSign:
                prevDifferenceSign = differenceSign
                if (differenceSign > 0):
                    self.rotate(speed, swing)
                else:
                    self.rotate(-speed, swing)

    def turnLeft(self):
        '''
        Rotate the plow 90 degrees CCW
        '''
        curDir = self.getPlowerDirection()
        newDir = ""
        if curDir == "N":
            newDir = "W"
        elif curDir == "E":
            newDir = "N"
        elif curDir == "S":
            newDir = "E"
        elif curDir == "W":
            newDir = "S"

        self.rotateTo(newDir, swing=False)

    def turnRight(self):
        '''
        Rotate the plow 90 degrees Cw
        '''
        curDir = self.getPlowerDirection()
        newDir = ""
        if curDir == "N":
            newDir = "E"
        elif curDir == "E":
            newDir = "S"
        elif curDir == "S":
            newDir = "W"
        elif curDir == "W":
            newDir = "N"

        self.rotateTo(newDir, swing=False)

    def move(self, distance):
        '''
        Have the plower move a specific distance in metres
        No checks take place while this method runs and blocks, use care with calling this method
        '''
        origin = self.getPlowerPosition()
        axis = self.getPlowerAxis()
        target = self.getTargetPosition(origin, distance, axis)

        self.setVelocity(0.5)
        while self.getPlowerPositionDifference(target, axis) > 0.1:
            continue
        self.setVelocity(0.05)
        while self.getPlowerPositionDifference(target, axis) > 0.05:
            continue

        self.stop()

    # --- Localization Methods ---
    def getPlowerOrientation(self):
        '''
        Return the current orientation of plower ranging from
        -pi to pi
        '''
        returnCode, ori = self.api.getObjectOrientation(self.plowerOb)
        #print(ori)
        return ori[2]

    def getPlowerPosition(self):
        '''
        Returns the current location of the plower on the map
        '''
        return self.api.getObjectPosition(self.plowerOb)[1]

    # --- Direction, Axis, and Target Methods ---
    def getPlowerDirection(self):
        '''
        Get the current facing of the plow (N, S, E, or W)
        '''
        orientation = self.getPlowerOrientation()
        orentationDegrees = orientation * 180 / math.pi
        roundedDegrees = round(orentationDegrees/90)*90
        #print(orientation)
        #print(orentationDegrees)
        #print(roundedDegrees)
        if (roundedDegrees == 0):
            return "N"
        elif (roundedDegrees == -90):
            return "E"
        elif (roundedDegrees == 180 or roundedDegrees == -180):
            return "S"
        elif (roundedDegrees == 90):
            return "W"
        else:
            print("DIRECTION ERROR")
            raise Exception("DIRECTION ERROR")

    def getPlowerAxis(self):
        '''
        Get the current axis the plower is moving on and facing
        '''
        direction = self.getPlowerDirection()
        if (direction == "N"):
            return "pos-y"
        elif (direction == "S"):
            return "neg-y"
        elif (direction == "E"):
            return "pos-x"
        elif (direction == "W"):
            return "neg-x"
        else:
            print("AXIS ERROR")
            raise Exception("AXIS ERROR")

    def getTargetPosition(self, origin, distance, axis):
        '''
        adds a value 'distance' to a coordinate origin
        to either x or y axis to return a target position
        '''
        target = origin[:]
        if (axis == "neg-x"):
            target[0] = target[0] - distance
        elif (axis == "pos-x"):
            target[0] = target[0] + distance
        elif (axis == "neg-y"):
            target[1] = target[1] - distance
        elif (axis == "pos-y"):
            target[1] = target[1] + distance
        else:
            print(f"BAD AXIS: {axis}")
            raise Exception("BAD AXIS")
        return target

    # --- Difference Methods ---
    def getPlowerPositionDifference(self, target, axis):
        '''
        Takes a list of [x,y,z] coordinates origin, and 
        a current axis 'x' or 'y'
        Will return the distance between current position
        and origin along the current axis 
        '''
        currentPosition = self.getPlowerPosition()
        if (axis == "neg-x"):
            return currentPosition[0]-target[0]
        elif (axis == "pos-x"):
            return target[0]-currentPosition[0]
        elif (axis == "neg-y"):
            return currentPosition[1]-target[1]
        elif (axis == "pos-y"):
            return target[1]-currentPosition[1]
        else:
            print(f"BAD AXIS: {axis}")
            raise Exception("BAD AXIS")

    def getPlowerOrientationDifference(self, target):
        '''
        Returns the differnece in rotation from the current
        orientation to the target orientation as a value
        between -pi and pi
        '''
        difference = target - self.getPlowerOrientation()
        if (difference < -math.pi):
            difference = difference + 2*math.pi
        if (difference > math.pi):
            difference = difference - 2*math.pi

        return difference


class Wheel():
    '''
    Defines a class for a wheel object
    This allows for precise control of each wheel
    '''
    def __init__(self, api, handle):
        self.api = api
        self.obj = self.api.getObject(handle)

    def setVelocity(self, speed):
        '''
        Sets the velocity of the wheel, speed is converted to CoppeliaSim
        by the API class and should be passed in here as m/s
        '''
        self.api.setJointVelocity(self.obj, speed)

    def stop(self):
        '''
        Stops the wheel by setting it's velocity to 0.
        '''
        self.api.setJointVelocity(self.obj, 0)
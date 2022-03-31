import time
import math

class MovementControl():
    def __init__(self, plow, api):
        self.plow = plow
        self.api = api

        self.LeftWheel = Wheel(api, 'LeftJoint')
        self.RightWheel = Wheel(api, 'RightJoint')
        self.plowerOb = self.api.getObject("Plower")

    # Basic Movement Function
    def setVelocity(self, speed):
        '''
        Set both wheels to rotate at a certain speed
        based on given input
        '''
        self.LeftWheel.setVelocity(speed)
        self.RightWheel.setVelocity(speed)

    def accelSetVelocity(self, speed):
        self.setVelocity(0.4)
        time.sleep(0.2)
        self.setVelocity(0.8)
        time.sleep(0.2)
        self.setVelocity(speed)

    def rotate(self, speed):
        '''
        rotate plower in place at a rate based
        on the speed given to the method
        '''
        self.LeftWheel.setVelocity(speed)
        self.RightWheel.setVelocity(-speed)

    def stop(self):
        '''
        Stops all motor movement in the plower
        '''
        self.LeftWheel.stop()
        self.RightWheel.stop()

    def decelStop(self):
        self.setVelocity(0.7)
        time.sleep(0.2)
        self.setVelocity(0.3)
        time.sleep(0.2)
        self.stop()

    # Movement Methods
    def rotateRadians(self, radians):
        '''
        Rotate a set number of radians
        in diection based on ± input
        '''
        # Rotate a set number of radians
        current = self.getPlowerOrientation()
        target = current + radians
        #print("DOING Rotation: radians, From, To")
        #print(radians)
        #print(current)
        #print(target)

        # if new angle greater than pi, readjust angle to fit on -pi to pi axis
        if (target > math.pi):
            target = -(math.pi - (target-math.pi))
        elif (target < -math.pi):
            target = math.pi+(target+math.pi)

        direction = -1 if (radians < 0) else 1
        # Initially set rotation speed to 0.3
        self.rotate(direction*0.3)
        while(self.getPlowerOrientationDifference(target) > math.pi/9):
            pass
        self.rotate(direction*0.08)
        while(self.getPlowerOrientationDifference(target) > 0.08):
            pass

        print(f"Angle Difference After Rotation: {self.getPlowerOrientationDifference(target)})")

        self.stop()

    def rotateTo(self, orientation, direction):
        """
        Rotate plower into specific cardinal direction
        Args:
        orientation = enter N,E,S, or W for direction to turn to
        direction = enter True for CW rotation, false for CCW
        """
        od = {"N":0, "E": -math.pi/2, "S": math.pi, "W": math.pi/2}
        target = od[orientation]

        direction = -1 if (direction) else 1
        # Initially set rotation speed to 0.3
        self.rotate(direction*0.3)
        while(self.getPlowerOrientationDifference(target) > math.pi/9):
            pass
        self.rotate(direction*0.08)
        while(self.getPlowerOrientationDifference(target) > 0.08):
            pass

        self.stop()
        print(f"Angle Difference After Rotation: {self.getPlowerOrientationDifference(target)})")

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

        self.rotateTo(newDir, False)

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

        self.rotateTo(newDir, True)

    def move(self, distance):
        '''
        Have the plower move a specific distance
        '''
        origin = self.getPlowerPosition()
        axis = self.getPlowerAxis()
        target = self.getTargetPosition(origin, distance, axis)
        print(f"Doing Move")

        print(origin)
        print(target)
        print(axis)

        self.setVelocity(0.5)
        while self.getPlowerPositionDifference(target, axis) > 0.1:
            print(f"Position Difference: {self.getPlowerPositionDifference(target, axis)}")
            continue
        print("Slowing Move")
        self.setVelocity(0.05)
        while self.getPlowerPositionDifference(target, axis) > 0.05:
            continue

        self.stop()
        print(f"Position Difference After Move: {self.getPlowerPositionDifference(target, axis)}")

    # Localization Methods
    def getPlowerOrientation(self):
        '''
        Return the current orientation of plower ranging from
        -pi to pi
        '''
        returnCode, ori = self.api.getObjectOrientation(self.plowerOb)
        #print(ori)
        return ori[2]

    def getPlowerDirection(self):
        """
        Get the current facing of the plow
        """
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
        get the current axis the plower is moving on
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

    # Positional Methods
    def getPlowerPosition(self):
        '''
        Returns the current location of the plower on the map
        '''
        return self.api.getObjectPosition(self.plowerOb)[1]

    def getPlowerPositionDifference(self, origin, axis):
        '''
        Takes a list of [x,y,z] coordinates origin, and 
        a current axis 'x' or 'y'
        Will return the distance between current position
        and origin on current axis
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

    def getTargetPosition(self, origin, distance, axis):
        '''
        adds a value 'distance' to a coordinate origin
        to either x or y axis
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

    def getPlowerOrientationDifference(self, target):
        difference = 0
        if (target == math.pi):
            difference = abs(target - abs(self.getPlowerOrientation()))
        else:
            difference = abs(target - self.getPlowerOrientation())
        return difference


class Wheel():
    def __init__(self, api, handle):
        self.api = api
        self.obj = self.api.getObject(handle) # Use API to get object handle

    def setVelocity(self, speed):
        # We may need to convert "speed" into rads/s? based on wheel size?
        self.api.setJointVelocity(self.obj, speed)

    def stop(self):
        self.api.setJointVelocity(self.obj, 0)
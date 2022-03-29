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
        self.LeftWheel.setVelocity(speed)
        self.RightWheel.setVelocity(speed)

    def rotate(self, speed):
        self.LeftWheel.setVelocity(speed)
        self.RightWheel.setVelocity(-speed)

    def stop(self):
        self.LeftWheel.stop()
        self.RightWheel.stop()

    # Timed Movements
    def timedRotate(self, speed, duration):
        self.rotate(speed)
        time.sleep(duration)
        self.stop()

    def timedMove(self, speed, duration):
        self.setVelocity(speed)
        time.sleep(duration)
        self.stop()

    # Rotational Methods
    def rotateRadians(self, radians):
        # Rotate a set number of radians
        current = self.getPlowerOrientation()
        target = current + radians
        print("DOING Rotation: radians, From, To")
        print(radians)
        print(current)
        print(target)

        # if new angle greater than pi, readjust
        if (target > math.pi):
            target = -(math.pi - (target-math.pi))
        elif (target < -math.pi):
            target = math.pi+(target+math.pi)

        #rotate based on direction given
        if (radians > 0):
            self.rotate(0.25)
        else:
            self.rotate(-0.25)

        #rotate until target equals point
        while (abs(target - self.getPlowerOrientation()) > 0.5):
            print(f"Rotation Difference: {abs(target - self.getPlowerOrientation())}")

        if (radians > 0):
            self.rotate(0.05)
        else:
            self.rotate(-0.05)

        while (abs(target - self.getPlowerOrientation()) > 0.025):
            print(f"Rotation Difference: {abs(target - self.getPlowerOrientation())}")

        self.stop()
        print("DONE ROTATION")


    def rotateTo(self, orientation, direction):
        """
        Rotate plower into specific cardinal direction
        Args:
        orientation = enter N,E,S, or W for direction to turn to
        direction = enter True for CW rotation, false for CCW
        """
        od = {"N":0, "E": -0.7068747282028198, "S":1, "W":0.7075492739677429}
        target = od[orientation]

        if (direction):
            self.rotate(-0.08)
        else:
            self.rotate(0.08)

        difference = abs(target - self.getPlowerOrientation())
        while(difference> 0.008):
            #print(difference)
            difference = abs(target - self.getPlowerOrientation())
        print("DONE ROTATION")
        self.stop()

    def forwardRotation(self, degrees):
        # Rotate while moving forward (A set distance) in order to keep the snow in the plow
        pass

    def getPlowerOrientation(self):
        return self.api.getObjectQuaternionOrientation(self.plowerOb)[1][2]

    def getPlowerDirection(self):
        orientation = self.getPlowerOrientation()
        orentationDegrees = orientation * 180 / math.pi
        roundedDegrees = round(orentationDegrees/90)*90
        if (roundedDegrees == 0):
            return "N"
        elif (roundedDegrees == 90):
            return "E"
        elif (roundedDegrees == 180):
            return "S"
        elif (roundedDegrees == 270):
            return "W"
        else:
            print("DIRECTION ERROR")
            raise Exception("DIRECTION ERROR")

    def getPlowerAxis(self):
        direction = self.getPlowerDirection()
        if (direction == "N" or direction == "S"):
            return "y"
        elif (direction == "E" or direction == "W"):
            return "x"
        else:
            print("AXIS ERROR")
            raise Exception("AXIS ERROR")

    def move(self, distance):
        origin = self.getPlowerPosition()
        axis = self.getPlowerAxis()
        target = self.getTargetPosition(origin, distance, axis)
        print("DOING MOVE: Axis, From, To")
        print(axis)
        print(origin)
        print(target)
        # We are sometimes missing
        self.setVelocity(0.25)
        while self.getPlowerPositionDifference(target, axis) > 0.1:
            print(f"Position Difference: {self.getPlowerPositionDifference(target, axis)}")
        self.setVelocity(0)
        self.setVelocity(0)
        print("DONE MOVE")

    # Positional Methods
    def getPlowerPosition(self):
        return self.api.getObjectPosition(self.plowerOb)[1]

    def getPlowerPositionDifference(self, origin, axis):
        currentPosition = self.getPlowerPosition()
        if (axis == "x"):
            return abs(currentPosition[0]-origin[0])
        elif (axis == "y"):
            return abs(currentPosition[1]-origin[1])
        else:
            print("BAD AXIS")
            raise Exception("BAD AXIS")

    def getTargetPosition(self, origin, distance, axis):
        target = origin[:]
        if (axis == "x"):
            target[0] = target[0] + distance
        elif (axis == "y"):
            target[1] = target[1] + distance
        else:
            print("BAD AXIS")
            raise Exception("BAD AXIS")
        return target



class Wheel():
    def __init__(self, api, handle):
        self.api = api
        self.obj = self.api.getObject(handle) # Use API to get object handle


    def setVelocity(self, speed):
        # We may need to convert "speed" into rads/s? based on wheel size?
        self.api.setJointVelocity(self.obj, speed)

    def stop(self):
        self.api.setJointVelocity(self.obj, 0)

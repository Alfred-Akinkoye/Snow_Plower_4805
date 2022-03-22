import time
import math

class MovementControl():
    def __init__(self, plow, api):
        self.plow = plow
        self.api = api

        self.LeftWheel = Wheel(api, 'LeftJoint')
        self.RightWheel = Wheel(api, 'RightJoint')
        self.plowerOb = self.api.getObject("Plower")
        

    def setVelocity(self, speed):
        self.LeftWheel.setVelocity(speed)
        self.RightWheel.setVelocity(speed)

    def stop(self):
        self.LeftWheel.stop()
        self.RightWheel.stop()

    def rotate(self, speed):
        self.LeftWheel.setVelocity(speed)
        self.RightWheel.setVelocity(-speed)
    
    def timedRotate(self, speed, duration):
        self.rotate(speed)
        time.sleep(duration)
        self.stop()

    def timedMove(self, speed, duration):
        self.setVelocity(speed)
        time.sleep(duration)
        self.stop()

    def getObjectPosition(self):
        result = self.api.getObjectPosition(self.plowerOb)
        return result[1]

    def getObjectOrientaion(self):
        result = self.api.getObjectOrientation(self.plowerOb)
        return result[1][2]
    
    def rotateDegrees(self, degrees):
        # Rotate a set number of degrees (Either ***by checking orientation*** or by time? or by wheel encoding)
        pass
    
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
        print(origin)
        print(axis)
        print(target)
        self.setVelocity(0.5)
        while self.getPlowerPositionDifference(target, axis) > 0.1: pass
        self.setVelocity(0.05)
        while self.getPlowerPositionDifference(target, axis) > 0.01: pass
        self.setVelocity(0.005)
        while self.getPlowerPositionDifference(target, axis) > 0.001: pass
        self.setVelocity(0)
        print("DONE")

    # Positional Methods
    def getPlowerPosition(self):
        return self.getObjectPosition()
    
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
        target = origin
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


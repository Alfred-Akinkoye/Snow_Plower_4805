import time
from math import pi

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

    def rotate_Amount(self, radians):
        # Rotate a set number of degrees (Either ***by checking orientation*** or by time? or by wheel encoding)
        current = self.getObjectOrientaion()
        target = current + radians
        print(target)
        
        #if new angle greater than pi, readjust
        if (target > pi):
            target = -(pi - (target-pi))
        elif (target < -pi):
            target = pi+(target+pi)
        
        print(target)
        #rotate based on direction given
        if (radians >0):
            self.rotate(1)
        else:
            self.rotate(-1)

        #rotate until target equals point
        difference = abs(target - self.getObjectOrientaion())
        while(difference> 0.025):
            #print(difference)
            difference = abs(target - self.getObjectOrientaion())
        self.stop()

    def rotate_Facing(self,orientation,direction):
        """
        Rotate plower into specific cardinal direction
        Args:
        orientation = enter N,E,S, or W for direction to turn to
        direction = enter True for CW rotation, false for CCW
        """
        od = {"N":0, "E": -pi/2, "S":pi, "W":pi/2}
        target = od[orientation]

        if (direction >0):
            self.rotate(-1)
        else:
            self.rotate(1)

        difference = abs(target - self.getObjectOrientaion())
        while(difference> 0.025):
            #print(difference)
            difference = abs(target - self.getObjectOrientaion())
        self.stop()

    
class Wheel():
    def __init__(self, api, handle):
        self.api = api
        self.obj = self.api.getObject(handle) # Use API to get object handle
        

    def setVelocity(self, speed):
        # We may need to convert "speed" into rads/s? based on wheel size?
        self.api.setJointVelocity(self.obj, speed)

    def stop(self):
        self.api.setJointVelocity(self.obj, 0)


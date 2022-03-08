import time

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

    def rotateDegrees(self, degrees):
        # Rotate a set number of degrees (Either ***by checking orientation*** or by time? or by wheel encoding)
        pass
    
    def forwardRotation(self, degrees):
        # Rotate while moving forward (A set distance) in order to keep the snow in the plow
        pass
    
class Wheel():
    def __init__(self, api, handle):
        self.api = api
        self.obj = self.api.getObject(handle) # Use API to get object handle
        

    def setVelocity(self, speed):
        # We may need to convert "speed" into rads/s? based on wheel size?
        self.api.setJointVelocity(self.obj, speed)

    def stop(self):
        self.api.setJointVelocity(self.obj, 0)


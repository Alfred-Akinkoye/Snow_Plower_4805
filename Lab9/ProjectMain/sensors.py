class Sensors:
    def __init__(self, plow, api):
        self.plow = plow
        self.api = api

        self.BackVisionSensor = VisionSensor('Back_IR')
        self.FrontVisionSensor = VisionSensor('Front_IR')
        self.LeftVisionSensor = VisionSensor('Lef_IR')
        self.RightVisionSensor = VisionSensor('Right_IR')
        self.visionSensors = [self.FrontSensor,self.LeftWheelSensor,self.RightWheelSensor,self.BackSensor]

        # Register Proximity Sensors


    def checkAllVisionSensors(self):
        for sensor in self.visionSensors:
            if (sensor.checkForLine()):
                return True
        return False

class VisionSensor():
    def __init__(self, objectHandle):
        self.object = self.api.getObject(objectHandle) #use the api to get the object from the handle
    
    def checkForLine(): # Returns a bool
        [temp, detectionState, data] = self.api.readVisionSensor(self.object)
        if data[0][11] < 0.3:
            return True
        return False

class ProximitySensor():
    def __init__(self, objectHandle):
        pass

    def getDistance(): # Returns a float 
        pass
        return 0.0

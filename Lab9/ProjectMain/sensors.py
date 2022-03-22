class Sensors:
    def __init__(self, plow, api):
        self.plow = plow
        self.api = api

        #Vision Sensors
        self.BackVisionSensor = VisionSensor(self.api, 'Back_IR')
        self.FrontVisionSensor = VisionSensor(self.api, 'Front_IR')
        self.LeftVisionSensor = VisionSensor(self.api, 'Left_IR')
        self.RightVisionSensor = VisionSensor(self.api, 'Right_IR')
        self.visionSensors = [self.FrontVisionSensor,self.LeftVisionSensor,self.RightVisionSensor,self.BackVisionSensor]

        # Register Proximity Sensors
        # self.BackVisionSensor = VisionSensor(self.api, 'Back_IR')
        # self.FrontVisionSensor = VisionSensor(self.api, 'Front_IR')
        # self.LeftVisionSensor = VisionSensor(self.api, 'Lef_IR')
        # self.RightVisionSensor = VisionSensor(self.api, 'Right_IR')
        # self.proximitySensors = [self.FrontSensor,self.LeftWheelSensor,self.RightWheelSensor,self.BackSensor]

    def checkAllVisionSensors(self):
        for sensor in self.visionSensors:
            if (sensor.checkForLine()):
                return True
        return False

class VisionSensor():
    def __init__(self, api, objectHandle):
        self.api = api
        self.object = self.api.getObject(objectHandle) #use the api to get the object from the handle

    def checkForLine(self): # Returns a bool
        [temp, detectionState, data] = self.api.readVisionSensor(self.object)
        print(data[0][11])
        if data[0][11] < 0.3:
            return True
        return False

class ProximitySensor():
    def __init__(self, api, objectHandle):
        self.api = api
        self.object = self.api.getObject(objectHandle) #use the api to get the object from the handle

    def getDistance(): # Returns a float
        pass
        return 0.0

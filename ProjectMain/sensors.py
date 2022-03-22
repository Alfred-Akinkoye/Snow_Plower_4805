class Sensors:
    def __init__(self, plow, api):
        self.plow = plow
        self.api = api

        self.BackVisionSensor = VisionSensor(self.api, 'Back_IR')
        self.FrontVisionSensor = VisionSensor(self.api, 'Front_IR')
        self.LeftVisionSensor = VisionSensor(self.api, 'Left_IR')
        self.RightVisionSensor = VisionSensor(self.api, 'Right_IR')
        self.visionSensors = [self.FrontVisionSensor,self.LeftVisionSensor,self.RightVisionSensor,self.BackVisionSensor]

        # Register Proximity Sensors

    # Vision Sensor Methods
    def checkAllVisionSensors(self):
        for sensor in self.visionSensors:
            if (sensor.checkForLine()):
                return True
        return False

    def checkFrontVisionSensor(self):
        if (self.FrontVisionSensor.checkForLine()):
            return True
        return False

    def checkBlackVisionSensor(self):
        if (self.FrontVisionSensor.checkForLine()):
            return True
        return False

    # Proximity Sensor Methods
    def objectAhead(self):
        return False



class VisionSensor():
    def __init__(self, api, objectHandle):
        self.api = api
        self.object = self.api.getObject(objectHandle) #use the api to get the object from the handle
    
    def checkForLine(self): # Returns a bool
        [returnCode, detectionState, data] = self.api.readVisionSensor(self.object)
        #print(detectionState)
        #print(auxPacket1)
        #print(data)
        if (data and len(data) > 0 and len(data[0]) > 11):
            if data[0][11] < 0.1 and data[0][11] > 0:
                print(f"SENSOR TRIGGERED: {data[0][11]}")
                return True
        return False

class ProximitySensor():
    def __init__(self, api, objectHandle):
        self.api = api
        pass

    def getDistance(self): # Returns a float 
        pass
        return 0.0

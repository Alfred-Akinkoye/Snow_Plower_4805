import math

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
        # Proximity Sensors
        self.F_Proximity = ProximitySensor(self.api, 'F_Proximity')
        self.R_Proximity = ProximitySensor(self.api, 'R_Proximity')
        self.FL_Proximity = ProximitySensor(self.api, 'FL_Proximity')
        self.FR_Proximity = ProximitySensor(self.api, 'FR_Proximity')
        self.FFL_Proximity = ProximitySensor(self.api, 'FFL_Proximity')
        self.FFR_Proximity = ProximitySensor(self.api, 'FFR_Proximity')

        self.B_Proximity = ProximitySensor(self.api, 'B_Proximity')
        self.BL_Proximity = ProximitySensor(self.api, 'BL_Proximity')
        self.BR_Proximity = ProximitySensor(self.api, 'BR_Proximity')

        self.L_Proximity = ProximitySensor(self.api, 'L_Proximity')
        self.R_Proximity = ProximitySensor(self.api, 'R_Proximity')
        self.LFL_Proximity = ProximitySensor(self.api, 'LFL_Proximity')
        self.RRFR_Proximity = ProximitySensor(self.api, 'RFR_Proximity')

        self.proximitySensors = [self.F_Proximity, self.R_Proximity, self.FL_Proximity, self.FR_Proximity, self.FFL_Proximity, self.FFR_Proximity, self.B_Proximity, self.BL_Proximity, self.BR_Proximity, self.L_Proximity, self.R_Proximity, self.LFL_Proximity, self.RRFR_Proximity]

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

    # Check all vision sensors at once. In the event we need to check all the
    # vision sensors at once. Unlikely that we will need it though.
    def checkAllProximitySensors(self):
        for sensor in self.proximitySensors:
            if (sensor.getDistance()):
                return True
        return False



class VisionSensor():
    def __init__(self, api, objectHandle):
        self.api = api
        self.object = self.api.getObject(objectHandle) #use the api to get the object from the handle

    def checkForLine(self): # Returns a bool
        [returnCode, detectionState, data] = self.api.readVisionSensor(self.object)
        if (data and len(data) > 0 and len(data[0]) > 11):
            if data[0][11] < 0.1 and data[0][11] > 0:
                print(f"SENSOR TRIGGERED: {data[0][11]}")
                return True
        return False

class ProximitySensor():
    def __init__(self, api, objectHandle):
        self.api = api
        self.objectHandle = objectHandle
        self.object = self.api.getObject(objectHandle) #use the api to get the object from the handle

    def getDistance(self): # Returns a float
        returnedData = self.api.readProximitySensor(self.object)

        [returnCode,
        detectionState,
        detectedPoint,
        detectedObjectHandle,
        detectedSurfaceNormalVector] = returnedData
        if detectionState == True
            return detectedPoint[2]

        # If nothing is detected return an infinite distance
        return math.inf

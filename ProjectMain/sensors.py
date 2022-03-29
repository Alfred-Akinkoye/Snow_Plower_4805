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
        #Proximity Sensors
        self.FrontProx = ProximitySensor(self.api, 'F_Proximity')
        self.RightProx = ProximitySensor(self.api, 'R_Proximity')
        #self.FrontLeftProx = ProximitySensor(self.api, 'FL_Proximity')
        #self.FrontRightProx = ProximitySensor(self.api, 'FR_Proximity')
        #self.FrontFLProx = ProximitySensor(self.api, 'FFL_Proximity')
        #self.FrontFRProx = ProximitySensor(self.api, 'FFR_Proximity')

        #self.BackProx = ProximitySensor(self.api, 'B_Proximity')
        #self.BackLeftProx = ProximitySensor(self.api, 'BL_Proximity')
        #self.BackRightProx = ProximitySensor(self.api, 'BR_Proximity')

        #self.LeftProx = ProximitySensor(self.api, 'L_Proximity')
        #self.RightProx = ProximitySensor(self.api, 'R_Proximity')
        #self.LeftFrontProx = ProximitySensor(self.api, 'LFL_Proximity')
        #self.RigtFrontProx = ProximitySensor(self.api, 'RFR_Proximity')

        self.proximitySensors = [self.FrontProx, self.RightProx]

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
        self.objectHandle = objectHandle
        self.object = self.api.getObject(objectHandle) #use the api to get the object from the handle

    def getDistance(self): # Returns a float
        returnedData = self.api.readProximitySensor(self.object)
        
        [returnCode, 
        detectionState, 
        detectedPoint, 
        detectedObjectHandle, 
        detectedSurfaceNormalVector] = returnedData

        print(f"{self.objectHandle}: {returnedData}")
        #print("Det point" + detectedPoint)
        #print(detectionState)
        #if detectionState:
        #    return True
        #return False

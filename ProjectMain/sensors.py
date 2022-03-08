class Sensors:
    def __init__(self, plow, api):
        self.plow = plow
        self.api = api


        self.frontVisionSensor = VisionSensor("frontSensor") # IDK what the actual handle is
        #... register other sensors
        # get vision example joint
        #self.BackVisionSensor = self.api.getObject('BackSensor')
        #self.FrontVisionSensor = self.api.getObject('FrontSensor')
        ##self.LeftWheelSensor = self.api.getObject('LeftWheelSensor')
        #self.RightWheelSensor = self.api.getObject('RightWheelSensor')

        pass

class VisionSensor():
    def __init__(self, objectHandle):
        self.object = None #use the api to get the object from the handle
        pass
    
    def checkForLine(): # Returns a bool
        pass
        return False

class ProximitySensor():
    def __init__(self, objectHandle):
        pass

    def getDistance(): # Returns a float 
        pass
        return 0.0


# Sensor Code moved from Main
# vision = [self.BackSensor,self.FrontSensor,self.LeftWheelSensor,self.RightWheelSensor]

#     nominalv = 6
#     self.setMove(nominalv)
#     try:
#         print("Running Sensor Loop")
#         while (True):
#             pass
#             sensor_values = [0,0,0,0]
#             for i in range(0,4):
#                 detectionState = self.api.readVisionSensor(vision[i]);
#                 #print(detectionState)
#                 if (detectionState[0] < 0):
#                     if detectionState[2][11] < 0.3:
#                         sensor_values[i] = 1
#                     else:
#                         sensor_values[i] = 0

#             rightv = nominalv
#             leftv  = nominalv
#             if(sensor_values[0] == 1 or sensor_values[1] == 1 or sensor_values[2] == 1 or sensor_values[3] == 1):
#                 leftv = leftv*-1
#                 rightv = rightv*-1

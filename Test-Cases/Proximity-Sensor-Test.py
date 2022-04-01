# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, include
# a corresponding call to simxFinish at the end!

# Import the pythonAPI files from their directory
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__))+"/PythonAPI")

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

# Addition imports
import time
import math
from api import API
from movementControl import MovementControl

class ProximitySensor:
    def __init__(self):
        self.api = API()

    def connectAPI(self):
        if self.api.connect():
            self.LeftJoint = self.api.getObject('LeftJoint')
            self.RightJoint=self.api.getObject('RightJoint')

             # All proximity sensors in use
            self.F_Proximity = self.api.getObject('F_Proximity')
            self.FL_Proximity = self.api.getObject('FL_Proximity')
            self.FR_Proximity = self.api.getObject('FR_Proximity')
            self.FFL_Proximity = self.api.getObject('FFL_Proximity')
            self.FFR_Proximity = self.api.getObject('FFR_Proximity')
            self.proximitySensorsFront = [self.F_Proximity, self.FFL_Proximity, self.FFR_Proximity]

            self.B_Proximity = self.api.getObject('B_Proximity')
            self.BL_Proximity = self.api.getObject('BL_Proximity')
            self.BR_Proximity = self.api.getObject('BR_Proximity')

            self.proximitySensorsBack = [self.B_Proximity, self.BL_Proximity, self.BR_Proximity]

            self.L_Proximity = self.api.getObject('L_Proximity')
            self.R_Proximity = self.api.getObject('R_Proximity')
            self.LFL_Proximity = self.api.getObject('LFL_Proximity')
            self.RRFR_Proximity = self.api.getObject('RFR_Proximity')
            self.RBR_Proximity = self.api.getObject('RBR_Proximity')
            self.LBL_Proximity = self.api.getObject('LBL_Proximity')

            self.proximitySensorsLeft = [self.FL_Proximity, self.FFL_Proximity,self.L_Proximity,self.LFL_Proximity, self.BL_Proximity, self.LBL_Proximity]
            self.proximitySensorsRight = [self.FR_Proximity,self.FFR_Proximity,self.BR_Proximity, self.R_Proximity, self.RRFR_Proximity, self.RBR_Proximity]
            return True
        return False

    def run(self):
        # MODIFY CODE HERE
        print("Car reading...")
        # Send some data to CoppeliaSim in a non-blocking fashion:
        self.api.sendMessage("Hello from Python! :)")

        nominalv = 0.25
        self.api.setJointVelocity(self.LeftJoint,nominalv)
        self.api.setJointVelocity(self.RightJoint,nominalv)

        while (True):
            if(self.checkProxyArray("Front",0.8)):
                print("Front array of sensors triggered")
                self.api.sendMessage("Front array of sensors triggered")
                self.api.setJointVelocity(self.LeftJoint,0)
                self.api.setJointVelocity(self.RightJoint,0)
                time.sleep(5)
                break
            if(self.checkProxyArray("Right",0.8)):
                print("Right array of sensors triggered")
                self.api.sendMessage("Right array of sensors triggered")
                self.api.setJointVelocity(self.LeftJoint,0)
                self.api.setJointVelocity(self.RightJoint,0)
                time.sleep(5)
                break
            if(self.checkProxyArray("Left",0.8)):
                print("Right array of sensors triggered")
                self.api.sendMessage("Right array of sensors triggered")
                self.api.setJointVelocity(self.LeftJoint,0)
                self.api.setJointVelocity(self.RightJoint,0)
                time.sleep(5)
                break
        self.stop()

    def stop(self):
        '''
        This is a method run at the end of the simulation, it will
        stop the running simulation and disconnect sim module
        '''
        print("Stopping...")
        # stop the simulation
        self.api.stopSimulation()
        print("Simulation Stopped")
        self.api.disconnect()
        print("Disconnected")

    def checkProxyArray(self,direction, distance):
        Directdict = {"Front":self.proximitySensorsFront,"Right":self.proximitySensorsRight,"Left":self.proximitySensorsLeft,"Back":self.proximitySensorsBack}
        array = Directdict[direction]
        for sensor in array:
            if (self.getDistance(sensor) < distance):
                return True
        return False

    def getDistance(self,sensor): # Returns a float
        returnedData = self.api.readProximitySensor(sensor)

        [returnCode,
        detectionState,
        detectedPoint,
        detectedObjectHandle,
        detectedSurfaceNormalVector] = returnedData
        if detectionState == True:
            return detectedPoint[2]

        # If nothing is detected return an infinite distance
        return math.inf


# This is the Main Script For testing
if __name__ == '__main__':
    print ('Program started')
    sensor = ProximitySensor()
    if (sensor.connectAPI()):
        try:
            sensor.run()
        except KeyboardInterrupt:
            sensor.stop()
            #print(plower.ra.checkDiffTimes)
        except Exception as e:
            sensor.stop()
            raise e
    # End Program Execution
    print("Exiting")

import random
import math
import time

class RandomAlgorithm:
    def __init__(self, plow):
        self.plow = plow
        self.api = plow.api
        self.movementControl = plow.movementControl
        self.sensors = plow.sensors

        self.wonkyTurnDirection = 1
        self.leaving = "backInside"

        self.lastCheckTime = time.time()
        self.checkDiffTimes = []

    def run(self):

        # Leave Garage
        self.movementControl.move(0.8)

        # Open Plow
        self.plow.unfoldPlow()

        # Turn Right
        self.movementControl.turnRight()

        # Go forward
        self.movementControl.accelSetVelocity(1)

        # LOOP
        while True:

            # If Object Encountered (Turn random degrees) (Go forward)
            #print(f"Checking Object Sensors: {time.time()*1000}")
            if (self.sensors.objectAhead(1.2)):
                print("Object Ahead")
                self.movementControl.decelStop()
                # Rotate either +/- pi/2 to pi radians
                self.movementControl.rotateRadians((random.random()*0.5 + 0.5) *  math.pi * (-1 if random.random() > 0.5 else 1))
                self.movementControl.accelSetVelocity(1)


            # If Edge Encountered (Dump Snow and turn around)
            self.checkDiffTimes.append(round((time.time()-self.lastCheckTime)*1000))
            self.lastCheckTime = time.time()
            if (self.sensors.checkFrontVisionSensor()):
                print("Vision Sensor Triggered")
                if (self.leaving == "backInside"):
                    self.movementControl.decelStop()
                    self.wonkyEdgeAlgorithm()
                    self.movementControl.accelSetVelocity(1)
                else:
                    if (self.sensors.checkBackVisionSensor() and self.leavingState == "backOutsideLine"):
                        self.leavingState == "backOnLine"
                    if (not self.sensors.checkBackVisionSensor() and self.leavingState == "backOnLine"):
                        self.leavingState == "backInside"

            # Sometimes randomly adjust direction
            # if (random.randint(0, 50) == 0):
            #     print("Random")
            #     self.movementControl.stop()
            #     # Rotate between -0.1 pi to 0.1 pi
            #     self.movementControl.rotateRadians((random.random()*0.1 - 0.2) *  math.pi)
            #     self.movementControl.setVelocity(0.5)
                   
    def wonkyEdgeAlgorithm(self):
        print("Wonky Edge Algorithm")
        self.movementControl.move(0.5)
        self.plow.foldPlow()
        self.movementControl.rotateRadians(math.pi/2 * self.wonkyTurnDirection)
        self.movementControl.move(1)
        self.movementControl.rotateRadians(math.pi/2 * self.wonkyTurnDirection)
        self.plow.unfoldPlow()
    
        self.wonkyTurnDirection = -self.wonkyTurnDirection
        self.leavingState = "backOutsideLine"

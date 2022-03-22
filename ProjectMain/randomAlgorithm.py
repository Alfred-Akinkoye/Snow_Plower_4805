import random
import math

class RandomAlgorithm:
    def __init__(self, plow):
        self.plow = plow
        self.api = plow.api
        self.movementControl = plow.movementControl
        self.sensors = plow.sensors

        self.wonkyTurnDirection = -1
        self.leaving = "backInside"

    def run(self):

        # Leave Garage
        self.movementControl.move(0.8)

        # Open Plow
        self.plow.unfoldPlow()

        # Go forward
        self.movementControl.setVelocity(0.5)

        # LOOP
        while True:
            # If Object Encountered (Turn random degrees) (Go forward)
            if (self.sensors.objectAhead()):
                print("Object Ahead")
                self.movementControl.stop()
                # Rotate either +/- pi/2 to pi radians
                self.movementControl.rotateRadians((random.random()*0.5 + 0.5) *  math.pi * (-1 if random.random() > 0.5 else 1))
                self.movementControl.setVelocity(0.5)


            # If Edge Encountered (Dump Snow and turn around)
            if (self.sensors.checkAllVisionSensors()):
                print("Vision Sensor Triggered")
                if (self.leaving == "backInside"):
                    self.movementControl.stop()
                    self.wonkyEdgeAlgorithm()
                    self.movementControl.setVelocity(0.5)
                else:
                    if (self.sensors.checkBackVisionSensor() and self.leavingState == "backOutsideLine"):
                        self.leavingState == "backOnLine"
                    if (not self.sensors.checkBackVisionSensor() and self.leavingState == "backOnLine"):
                        self.leavingState == "backInside"

            # Sometimes randomly adjust direction
            if (random.randint(0, 50) == 0):
                print("Random")
                self.movementControl.stop()
                # Rotate between -0.1 pi to 0.1 pi
                self.movementControl.rotateRadians((random.random()*0.1 - 0.2) *  math.pi)
                self.movementControl.setVelocity(0.5)
                   
    def wonkyEdgeAlgorithm(self):
        print("Wonky Edge Algorithm")
        self.movementControl.move(1)
        self.movementControl.rotate(math.pi * self.wonkyTurnDirection)
        self.movementControl.move(1)
        self.movementControl.rotate(math.pi * self.wonkyTurnDirection)
    
        self.wonkyTurnDirection = -self.wonkyTurnDirection
        self.leavingState = "backOutsideLine"
        

        

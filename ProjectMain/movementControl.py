import time
import math

class MovementControl():
    def __init__(self, plow, api):
        self.plow = plow
        self.api = api

        self.LeftWheel = Wheel(api, 'LeftJoint')
        self.RightWheel = Wheel(api, 'RightJoint')
        self.plowerOb = self.api.getObject("Plower")

    # Basic Movement Function
    def setVelocity(self, speed):
        '''
        Set both wheels to rotate at a certain speed
        based on given input
        '''
        self.LeftWheel.setVelocity(speed)
        self.RightWheel.setVelocity(speed)

    def rotate(self, speed):
        '''
        rotate plower in place at a rate based
        on the speed given to the method
        '''
        self.LeftWheel.setVelocity(speed)
        self.RightWheel.setVelocity(-speed)

    def stop(self):
        '''
        Stops all motor movement in the plower
        '''
        self.LeftWheel.stop()
        self.RightWheel.stop()

    # Rotational Methods
    def rotateRadians(self, radians):
        '''
        Rotate a set number of radians
        in diection based on ± input
        '''
        # Rotate a set number of radians
        current = self.getPlowerOrientation()
        target = current + radians
        print("DOING Rotation: radians, From, To")
        print(radians)
        print(current)
        print(target)

        # if new angle greater than pi, readjust angle to fit on -pi to pi axis
        if (target > math.pi):
            target = -(math.pi - (target-math.pi))
        elif (target < -math.pi):
            target = math.pi+(target+math.pi)

        #rotate based on direction given
        if (radians > 0):
            self.rotate(0.25)
        else:
            self.rotate(-0.25)

        #rotate until target equals point
        while (abs(target - self.getPlowerOrientation()) > 0.5):
            print(f"Rotation Difference: {abs(target - self.getPlowerOrientation())}")

        #rotate cw or ccw based on radians given
        if (radians > 0):
            self.rotate(0.05)
        else:
            self.rotate(-0.05)

        while (abs(target - self.getPlowerOrientation()) > 0.025):
            print(f"Rotation Difference: {abs(target - self.getPlowerOrientation())}")

        self.stop()
        print("DONE ROTATION")


    def rotateTo(self, orientation, direction):
        """
        Rotate plower into specific cardinal direction
        Args:
        orientation = enter N,E,S, or W for direction to turn to
        direction = enter True for CW rotation, false for CCW
        """
        od = {"N":0, "E": -math.pi/2, "S": math.pi, "W": math.pi/2}
        target = od[orientation]

        # Initially set rotation speed to 0.3
        if (direction):
            self.rotate(-0.3)
        else:
            self.rotate(0.3)


        difference = abs(target - self.getPlowerOrientation())
        if (direction == "S"):
               difference = abs(abs(target) - abs(self.getPlowerOrientation()))

        # turn until within 3 degrees of target angle
        while(difference > 0.05): 

            if (direction == "S"):
                difference = abs(target - abs(self.getPlowerOrientation()))
            else:
                difference = abs(target - self.getPlowerOrientation())

            # When difference is less than 19 degrees (pi / 9) slow down the rate of rotation
            if (difference < math.pi/9):
                if (direction):
                    self.rotate(-0.08)
                else:
                    self.rotate(0.08)

        print("DONE ROTATION")
        self.stop()


    def turnLeft(self):
        '''
        Rotate the plow 90 degrees CCW
        '''
        curDir = self.getPlowerDirection()
        newDir = ""
        if curDir == "N":
            newDir = "W"
        elif curDir == "E":
            newDir = "N"
        elif curDir == "S":
            newDir = "E"
        elif curDir == "W":
            newDir = "S"

        self.rotateTo(newDir, False)

    def turnRight(self):
        '''
        Rotate the plow 90 degrees Cw
        '''
        curDir = self.getPlowerDirection()
        newDir = ""
        if curDir == "N":
            newDir = "E"
        elif curDir == "E":
            newDir = "S"
        elif curDir == "S":
            newDir = "W"
        elif curDir == "W":
            newDir = "N"

        self.rotateTo(newDir, True)


    def getPlowerOrientation(self):
        '''
        Return the current orientation of plower ranging from
        -pi to pi
        '''
        returnCode, ori = self.api.getObjectOrientation(self.plowerOb)
        #print(ori)
        return ori[2]

    def getPlowerDirection(self):
        """
        Get the current facing of the plow
        """
        orientation = self.getPlowerOrientation()
        orentationDegrees = orientation * 180 / math.pi
        roundedDegrees = round(orentationDegrees/90)*90
        if (roundedDegrees == 0):
            return "N"
        elif (roundedDegrees == 90):
            return "E"
        elif (roundedDegrees == 180):
            return "S"
        elif (roundedDegrees == 270):
            return "W"
        else:
            print("DIRECTION ERROR")
            raise Exception("DIRECTION ERROR")

    def getPlowerAxis(self):
        '''
        get the current axis the plower is moving on
        '''
        direction = self.getPlowerDirection()
        if (direction == "N" or direction == "S"):
            return "y"
        elif (direction == "E" or direction == "W"):
            return "x"
        else:
            print("AXIS ERROR")
            raise Exception("AXIS ERROR")

    def move(self, distance):
        '''
        Have the plower move a specific distance
        '''
        origin = self.getPlowerPosition()
        axis = self.getPlowerAxis()
        target = self.getTargetPosition(origin, distance, axis)

        self.setVelocity(0.25)
        while self.getPlowerPositionDifference(target, axis) > 0.1:
            continue
        self.stop()
        print("DONE MOVE")

    # Positional Methods
    def getPlowerPosition(self):
        '''
        Returns the current location of the plower on the map
        '''
        return self.api.getObjectPosition(self.plowerOb)[1]

    def getPlowerPositionDifference(self, origin, axis):
        '''
        Takes a list of [x,y,z] coordinates origin, and 
        a current axis 'x' or 'y'
        Will return the distance between current position
        and origin on current axis
        '''
        currentPosition = self.getPlowerPosition()
        if (axis == "x"):
            return abs(currentPosition[0]-origin[0])
        elif (axis == "y"):
            return abs(currentPosition[1]-origin[1])
        else:
            print("BAD AXIS")
            raise Exception("BAD AXIS")

    def getTargetPosition(self, origin, distance, axis):
        '''
        adds a value 'distance' to a coordinate origin
        to either x or y axis
        '''
        target = origin[:]
        if (axis == "x"):
            target[0] = target[0] + distance
        elif (axis == "y"):
            target[1] = target[1] + distance
        else:
            print("BAD AXIS")
            raise Exception("BAD AXIS")
        return target


class Wheel():
    def __init__(self, api, handle):
        self.api = api
        self.obj = self.api.getObject(handle) # Use API to get object handle


    def setVelocity(self, speed):
        # We may need to convert "speed" into rads/s? based on wheel size?
        self.api.setJointVelocity(self.obj, speed)

    def stop(self):
        self.api.setJointVelocity(self.obj, 0)
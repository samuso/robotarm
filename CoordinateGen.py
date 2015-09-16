# initialise and then use the
# withAngle method to generate a
# list of 2 tuples with the
# coordinates of the final points
# coordinateGen = CoordinateGen(startPoint, endPoint, rootSectionLength, middleSectionLength, endSectionLength)
# solution = coordinateGen.withAngle(minimumAngle)
from __future__ import division
import math

class CoordinateGen:

    def __init__ (self, startingPoint, objectPoint, rootSectionLength, middleSectionLength, endSectionLength):
        self._startingPoint = startingPoint;
        self._objectPoint = objectPoint;
        self._rootSectionLength = rootSectionLength;
        self._middleSectionLength = middleSectionLength;
        self._endSectionLength = endSectionLength;

    def _isTriangle(self, side1, side2, side3):
        if (side1 + side2 < side3 or
            side1 + side3 < side2 or
            side2 + side3 < side1):
            return False
        return True

    def triangleAngleCalculator(self, side1, side2, side3):
        if not self._isTriangle(side1, side2, side3):
            return False
        temp = (side1 ** 2) + (side2 ** 2) - (side3 ** 2)
        cos = temp / (2 * side1 * side2)
        return math.acos(cos)

    def getPointOnCircle(self, centre, radius, angle):
        x = centre[0] + math.cos(angle) * radius
        y = centre[1] + math.sin(angle) * radius
        return (x,y)

    def distanceBetweenPoints(self, pointOne, pointTwo):
        xDistance = pointOne[0] - pointTwo[0]
        yDistance = pointOne[1] - pointTwo[1]
        return math.sqrt(xDistance**2 + yDistance**2)

    def withAngle(self, angle):
        # first we generate the first point
        pointOne = self.getPointOnCircle(self._startingPoint, self._rootSectionLength, angle)

        # then we need two angles
        # angle one
        l1 = self.distanceBetweenPoints(pointOne, self._objectPoint)
        l2 = self._endSectionLength
        l3 = self._middleSectionLength
        angle1 = self.triangleAngleCalculator(l1, l2, l3)
        # angle2
        l1 = l1
        l2 = self.distanceBetweenPoints(self._objectPoint, self._startingPoint)
        l3 = self._rootSectionLength
        angle2 = self.triangleAngleCalculator(l1, l2, l3)
        # now we get the total of the two angles
        totalAngle = math.pi - (angle1 + angle2)

        # and finally the second point
        pointTwo =  self.getPointOnCircle(self._objectPoint, self._endSectionLength, totalAngle)
        return [pointOne, pointTwo]

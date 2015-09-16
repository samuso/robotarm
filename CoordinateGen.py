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

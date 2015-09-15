from __future__ import division
import math

class CoordinateGen:

    def __init__ (self):
        pass

    def isTriangle(self, side1, side2, side3):
        if (side1 + side2 < side3 or
            side1 + side3 < side2 or
            side2 + side3 < side1):
            return False
        return True

    def triangleAngleCalculator(self, side1, side2, side3):
        if not self.isTriangle(side1, side2, side3):
            return False
        temp = (side1 ** 2) + (side2 ** 2) - (side3 ** 2)
        cos = temp / (2 * side1 * side2)
        return math.acos(cos)

    

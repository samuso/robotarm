# initialise and then use the
# withAngle method to generate a
# list of 2 tuples with the
# coordinates of the final points
# coordinateGen = CoordinateGen()
# solution = coordinateGen.withAngle(startPoint, endPoint, rootSectionLength, middleSectionLength, endSectionLength, minimumAngle)
from __future__ import division
import math
import numpy as np
class CoordinateGen:

	def __init__ (self):
		pass

	def _isTriangle(self, side1, side2, side3):
		if (side1 + side2 < side3 or
			side1 + side3 < side2 or
			side2 + side3 < side1):
			return False
		return True

	def triangleAngleCalculator(self, side1, side2, side3):
		if not self._isTriangle(side1, side2, side3):
			return None
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

	def withAngle(self, startingPoint, objectPoint, rootSectionLength, middleSectionLength, endSectionLength, angle):
		# first we generate the first point

		failedResult = [None,None]
		pointOne = self.getPointOnCircle(startingPoint, rootSectionLength, angle)
		print "startingPoint:{} \n pointOne:{}".format(startingPoint, pointOne)
		# then we need two angles
		# angle one
		l1 = self.distanceBetweenPoints(pointOne, objectPoint)

		# l1 = rootSectionLength
		l2 = endSectionLength
		l3 = middleSectionLength

		if np.linalg.norm(np.subtract(startingPoint, objectPoint)) > (l1+l2+l3):
		# if not self._isTriangle(l1, l2, l3):
			print "1! l1:{}, l2:{}, l3:{}".format(l1,l2,l3)
			return failedResult
		angle1 = self.triangleAngleCalculator(l1, l2, l3)
		if angle1 == None:
			print "2! l1:{}, l2:{}, l3:{}".format(l1,l2,l3)
			return failedResult
		# angle2
		# l1 = l1
		l1 = self.distanceBetweenPoints(pointOne, objectPoint)
		l2 = self.distanceBetweenPoints(objectPoint, startingPoint)
		l3 = rootSectionLength
		if not self._isTriangle(l1, l2, l3):
		    print "3! l1:{}, l2:{}, l3:{}".format(l1,l2,l3)
		    return failedResult
		angle2 = self.triangleAngleCalculator(l1, l2, l3)
		# if angle2 == None:
		# 	return failedResult
		# now we get the total of the two angles
		totalAngle = math.pi - (angle1 + angle2)

		# and finally the second point
		pointTwo =  self.getPointOnCircle(objectPoint, endSectionLength, totalAngle)

		l1 = middleSectionLength
		l2 = rootSectionLength
		l3 = self.distanceBetweenPoints(startingPoint, pointTwo)
		if not self._isTriangle(l1, l2, l3):
		    print "3! l1:{}, l2:{}, l3:{}".format(l1,l2,l3)
		    return failedResult
		finalAngle1 = self.triangleAngleCalculator(l1, l2, l3)

		l1 = endSectionLength
		l2 = middleSectionLength
		l3 = self.distanceBetweenPoints(pointOne, objectPoint)
		if not self._isTriangle(l1, l2, l3):
		    print "4! l1:{}, l2:{}, l3:{}".format(l1,l2,l3)
		    return failedResult
		finalAngle2 = self.triangleAngleCalculator(l1, l2, l3)

		if (angle < 1.8 and finalAngle1<1.5):
			return failedResult

		if (finalAngle2 < 1.8):
		    print "5! finalAngle2:{}".format(finalAngle2)
		    return failedResult
		if finalAngle1 == None or finalAngle2 == None:
			print "6! failure"
			return failedResult
		else:
			return (finalAngle1, finalAngle2)

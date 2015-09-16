from CoordinateGen import CoordinateGen
import math

# Data we need to provide with
minimumAngle = math.pi / 4
startPoint = (0, 0)
endPoint = (7, 0)
rootSectionLength = 5
middleSectionLength = 3.79
endSectionLength = 2

coordinateGen = CoordinateGen(startPoint, endPoint, 5, 3 ,2)

middleSectionPointOne = coordinateGen.getPointOnCircle(startPoint, rootSectionLength, minimumAngle)
print "middle section point one is : " + str(middleSectionPointOne)

l1 = coordinateGen.distanceBetweenPoints(middleSectionPointOne, endPoint)
print "l1 =  " + str(l1)
l2 = endSectionLength
print "l2 = " + str(l2)
l3 = middleSectionLength
print "l3 = " + str(l3)
# First angle
angle1 = coordinateGen.triangleAngleCalculator(l1, l2, l3)
print "angle one in degrees is = " + str(angle1 * 180 / math.pi) + "\n"

# Second angle
l1 = coordinateGen.distanceBetweenPoints(middleSectionPointOne, endPoint)
print "l1 = " + str(l1)
l2 = coordinateGen.distanceBetweenPoints(endPoint, startPoint)
print "l2 = " + str(l2)
l3 = rootSectionLength
print "l3 = " + str(l3)
angle2 = coordinateGen.triangleAngleCalculator(l1, l2, l3)

print "angle two in degrees is = " + str(angle2 * 180 / math.pi) + "\n"
# print angle2

# to find the final angle
totalAngle = math.pi - (angle1 + angle2)
print "total angle = " + str(totalAngle * 180 / math.pi)
middleSectionPointTwo = coordinateGen.getPointOnCircle(endPoint, endSectionLength, totalAngle)
print middleSectionPointTwo

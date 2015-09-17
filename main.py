from CoordinateGen import CoordinateGen
import math

# Data we need to provide with
minimumAngle = math.pi / 8.0
startPoint = (0, 0)
endPoint = (22.32, 0)
rootSectionLength = 10
middleSectionLength = 5
endSectionLength = 10

coordinateGen = CoordinateGen()
solution = coordinateGen.withAngle(startPoint, endPoint, rootSectionLength, middleSectionLength, endSectionLength, minimumAngle)
print "the answer is : \n" + str(solution)

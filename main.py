from CoordinateGen import CoordinateGen
import math

# Data we need to provide with
minimumAngle = math.pi / 4
startPoint = (0, 0)
endPoint = (7, 0)
rootSectionLength = 5
middleSectionLength = 3.79
endSectionLength = 2

coordinateGen = CoordinateGen()
solution = coordinateGen.withAngle(startPoint, endPoint, rootSectionLength, middleSectionLength, endSectionLength, minimumAngle)
print "the answer is : \n" + str(solution)

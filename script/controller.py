
from __future__ import division

import os, sys, pipes,signal

import time
import subprocess
import struct
import numpy as np, math

import serial
from CoordinateGen import CoordinateGen

STARTSEQ = chr(108)
rootSectionLength = 98
middleSectionLength = 122
endSectionLength = 75

class robotarm(object):
	def __init__(self):
		self.coordinateGen = CoordinateGen()

		self.initialiseVision()
		self.ser = serial.Serial(
			port='/dev/tty.usbmodem1411',
			baudrate=115200,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS,
			timeout=0.5
		)

		if self.ser.isOpen() == 0:
			self.ser.open()
		while (1):
			try:
				points = self.getPoints()
			except:
				print "no data.."
			if points:
				# print points
				# get offset 
				offx = points[0]
				offy = points[1]
				offz = points[2]
				base= [points[0], points[1], points[2]]
				joint0 = [points[3], points[4], points[5]]
				joint1 = [points[6], points[7], points[8]]
				joint1_roll = points[12]
				obj = [points[9],points[10],points[11]]
				# if (self.getDistFromPlane (joint0, joint1, base, obj)>-20):
				# 	#move counterclockwise
				# 	print "counterclockwise"
				# 	self.ser.write(STARTSEQ)
				# 	cmd = chr(0B00010000)
				# 	self.ser.write(cmd)
				# elif (self.getDistFromPlane (joint0, joint1, base, obj)<-60):
				# 	#move clockwise
				# 	print "clockwise"
				# 	self.ser.write(STARTSEQ)
				# 	cmd = chr(0B00110000)
				# 	self.ser.write(cmd)

				#tracking ba0
				# a = self.getdist( base, joint0)
				# b = self.getdist(base, obj)
				# c = self.getdist( obj, joint0)
				# ba0 = np.arccos((a**2 + b**2 - c**2)/(2*a*b))

				
				# ba0_target = np.pi*2/3

				# if (ba0_target - ja0) > .2:
				# 	# base_0 go forward
				# 	self.ser.write(STARTSEQ)
				# 	cmd = chr(0B01010000)
				# 	self.ser.write(cmd)
				# elif (ba0_target - ja0) < -.2:
				# 	#base_0 go backward
				# 	self.ser.write(STARTSEQ)
				# 	cmd = chr(0B01110000)
				# 	self.ser.write(cmd)

				#tracking ja0
				# ba0 = np.pi *2/3
				# ja0_target = self.coordinateGen.withAngle(base, obj, rootSectionLength, middleSectionLength, endSectionLength, ba0)
				# print "ja0_target: {}".format(ja0_target)
				# a = self.getdist( base, joint0)
				# b = self.getdist(joint0, joint1)
				# c = self.getdist( base, joint1)
				# ja0 = np.arccos((a**2 + b**2 - c**2)/(2*a*b))


				# if (ja0_target - ja0) > .2:
				# 	# base_0 go forward
				# 	self.ser.write(STARTSEQ)
				# 	cmd = chr(0B10010000)
				# 	self.ser.write(cmd)
				# elif(ja0_target - ja0) < -.2:
				# 	#base_0 go backward
				# 	self.ser.write(STARTSEQ)
				# 	cmd = chr(0B10110000)
				# 	self.ser.write(cmd)

				#Tracking ja1
				l2angle = np.arctan2(joint1[1]-joint0[1], joint1[0]-joint0[0])

				ja1 = np.pi + ( joint1_roll - l2angle)

				ja1_target = np.pi

				print "ja1_target: {}, ja1: {}, l2angle:{}, joint1_roll:{}".format(ja1_target, ja1, l2angle, joint1_roll)
				if (ja1_target - ja1) > .2:
					print "backward"
					# joint1 go backward
					self.ser.write(STARTSEQ)
					cmd = chr(0B11110000)
					self.ser.write(cmd)
				elif(ja1_target - ja1) < -.2:
					print "forward"
					#joint1 go forward
					self.ser.write(STARTSEQ)
					cmd = chr(0B11010000)
					self.ser.write(cmd)

	def getDistFromPlane (self, p1, p2, p3, pt):
		# p1 = np.matrix(point1[0], point1[1], point1[2])
		# p2 = np.matrix(point2[0], point2[1], point2[2])
		# p3 = np.matrix(point3[0], point3[1], point3[2])

		v1 = np.subtract(p1,p2)
		v2 = np.subtract(p1,p3)
		n = np.cross(v1,v2)
		n = n/np.linalg.norm(n)

		sb = np.subtract(pt, p1)
		d = np.dot(n,sb)

		return  d
	def distFromPlane(self, pl, pt):
		dist = (pl[0]*pt[0] + pl[1]*pt[1] + pl[2] *pt[2] + p[3])/np.sqrt(pl[0]**2 + pl[1]**2 + pl[2]**2)

	def getdist(self, pointOne, pointTwo):

		xDistance = pointOne[0] - pointTwo[0]
		yDistance = pointOne[1] - pointTwo[1]
		zDistance = pointOne[2] - pointTwo[2]
		return math.sqrt(xDistance**2 + yDistance**2 + zDistance**2)

	def initialiseVision(self):
		# start vision
		cmd = "./../build/getcoords"
		self.proc = subprocess.Popen(cmd,shell=False, preexec_fn=os.setsid)
		time.sleep(3)
		self.pipein = open("/tmp/autoarm",'r')


	def getPoints(self): # base, join0, joint1, object
		nums = self.pipein.read(52)
		x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,r  = struct.unpack('fffffffffffff',nums)
		# print "x1: {}, y1: {}, z1:{}\n x2:{},y2:{},z2:{}, \n x3:{},y3:{},z3:{} \n x4:{},y4:{},z4:{}, roll: {}".format(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,r)
		return -x1,-y1,z1,-x2,-y2,z2,-x3,-y3,z3,-x4,-y4,z4,r

	def __del__(self):
		try:
			self.pipein.close()
		except:
			pass
		print "close please!"
		try:
			os.killpg(self.proc.pid, signal.SIGTERM)
		except:
			pass
		try:
			ser.close()
		except:
			pass
	def initialiseRoll(self):
		a = np.arctan2(joint1[1]-joint2[1])
if __name__ == "__main__":
	robotarm()

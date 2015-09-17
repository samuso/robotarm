
from __future__ import division

import os, sys, pipes,signal

import time
import subprocess
import struct
import numpy as np, math

import serial
import threading


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
		self.flag = -1
		# intialiase target
		while self.flag is not 2:
			[self.points, self.flag ]= self.getPoints()
			self.prevobj = self.points["obj"]
			self.prevflag= self.flag
		self.resetControl = False
		self.controlWorker = threading.Thread(target=self.controlArm)
		self.controlWorker.start()

		while (1):
			[self.points, self.flag ]= self.getPoints()
			if not np.isnan((self.points["obj"][0])):
				mag = np.linalg.norm(np.subtract(self.prevobj,self.points["obj"]))
				print "mag: {}".format(mag)
				if (mag >40):
					self.resetControl = True
				else:
					self.resetControl = False
		self.controlWorker.join()			
	def controlArm(self):
			# try:
		while(1):
			while not self.resetControl:
				print 
				if self.points and self.flag == 2:
					# print "All objects"
					# search for target points.
					base = self.points["base"]

					obj = self.points["obj"]
					
					# print "base:{}".format(base)
					# print "obj:{}".format(obj)
					startang = np.pi/2
					stopang = np.pi*5/6
					step = 0.2
					ba0_target = startang
					ja0_target = None
					while (ba0_target< stopang and not self.resetControl):

						
	  					[ja0_target, ja1_target] = self.coordinateGen.withAngle(base, obj, rootSectionLength, middleSectionLength, endSectionLength, ba0_target)
	  					ba0_target +=step
	  					if ja0_target:
		  					# print "ba0_target: {}".format(ba0_target)
		  					print "ja0_target: {}".format(ja0_target)
	  					if ja0_target is not None: 
	  						break
				 	# Begin tracking

				 	if ja0_target is not None and not self.resetControl:
				 		# begin tracking. Assume fixed target.
				 		pass
			 			#Track ba0
			 			# self.trackBa0(ba0_target)
				 		self.trackJa0(ja0_target)
				 		# self.trackJa1(ja1_target)

	 		print "END PHASE"






	 		# except:
				# print "error.."
				# print points
				# get offset 
				# offx = points[0]
				# offy = points[1]
				# offz = points[2]
				# base= [points[0], points[1], points[2]]
				# joint0 = [points[3], points[4], points[5]]
				# joint1 = [points[6], points[7], points[8]]
				# joint1_roll = points[12]
				# obj = [points[9],points[10],points[11]]
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


				

				#Tracking ja1
				# l2angle = np.arctan2(joint1[1]-joint0[1], joint1[0]-joint0[0])

				# ja1 = np.pi + ( joint1_roll - l2angle)

				# ja1_target = np.pi

	def drange(self,start, stop, step):
		r = start
		while r < stop:
			yield r
 		r += step			
	def trackBa0(self, ba0_target):
		while (not self.resetControl):

			ba0 = self.estimateBa0()
			if ba0 is not None:	
			 	if (ba0_target - ba0) > .2:
					# base_0 go forward
					self.ser.write(STARTSEQ)
					cmd = chr(0B01010000)
					self.ser.write(cmd)
				elif (ba0_target - ba0) < -.2:
					#base_0 go backward
					self.ser.write(STARTSEQ)
					cmd = chr(0B01110000)
					self.ser.write(cmd)
				elif abs(ba0_target - ba0) < 0.2:
					break
	def trackJa0(self, ja0_target):
		while (not self.resetControl):
			ja0 = self.estimateJa0()
			if (ja0_target - ja0) > .2:
				# base_0 go forward
				self.ser.write(STARTSEQ)
				cmd = chr(0B10010000)
				self.ser.write(cmd)
			elif(ja0_target - ja0) < -.2:
				#base_0 go backward
				self.ser.write(STARTSEQ)
				cmd = chr(0B10110000)
				self.ser.write(cmd)
			elif abs(ja0_target - ja0) < 0.2:
				break
			time.sleep(1)
	def trackJa1(self, ja1_target):
		while (not self.resetControl):
			
			ja1 = self.estimateJa1()
			# print "ja1_target: {}, ja1: {}, l2angle:{}, joint1_roll:{}".format(ja1_target, ja1, l2angle, joint1_roll)
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
			elif abs(ja1_target - ja1) < 0.2:
				break
	def estimateBa0(self):
		points = self.points
		flag = self.flag
	# tracking ba0
		if flag == 2:
			base = points["base"]
			joint0 = points["joint0"]
			joint1 = points["joint1"]
			obj = points["obj"]
			a = self.getdist( base, joint0)
			b = self.getdist(base, obj)
			c = self.getdist( obj, joint0)
			ba0 = np.arccos((a**2 + b**2 - c**2)/(2*a*b))
			return ba0
		else:
			return None
	def estimateJa0(self):
		points = self.points
		flag = self.flag

		base = points["base"]
		joint0 = points["joint0"]
		joint1 = points["joint1"]
		a = self.getdist( base, joint0)
		b = self.getdist(joint0, joint1)
		c = self.getdist( base, joint1)
		ja0 = np.arccos((a**2 + b**2 - c**2)/(2*a*b))
		return ja0
	def estimateJa1(self):
		points = self.points
		flag = self.flag

		base = points["base"]
		joint0 = points["joint0"]
		joint1 = points["joint1"]
		joint1_roll = points["joint1_roll"]
		l2angle = np.arctan2(joint1[1]-joint0[1], joint1[0]-joint0[0])

		ja1 = np.pi + ( joint1_roll - l2angle)
		return ja1

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
		with open(os.devnull, 'w') as devnull:
			self.proc = subprocess.Popen(cmd,preexec_fn=os.setsid)
		time.sleep(3)
		self.pipein = open("/tmp/autoarm",'r')


	def getPoints(self): # base, join0, joint1, object, flag
		# flags: 1 = arm only. 2 = all
		nums = self.pipein.read(52)
		if nums:
			x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,r  = struct.unpack('fffffffffffff',nums)
			# print "x1: {}, y1: {}, z1:{}\n x2:{},y2:{},z2:{}, \n x3:{},y3:{},z3:{} \n x4:{},y4:{},z4:{}, roll: {}".format(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,r)
			

			points = -x1,-y1,z1,-x2,-y2,z2,-x3,-y3,z3,-x4,-y4,z4,r
			base= [points[0], points[1], points[2]]
			joint0 = [points[3], points[4], points[5]]
			joint1 = [points[6], points[7], points[8]]
			joint1_roll = points[12]
			obj = [points[9],points[10],points[11]]
			points = {"base": [points[0], points[1], points[2]], "joint0":[points[3], points[4], points[5]], "joint1":[points[6], points[7], points[8]], "joint1_roll": points[12], "obj": [points[9],points[10],points[11]]}
			if np.isnan(obj[0]):
				flag = 1
			else:
				flag = 2
			return [points, flag]
		else:
			return {}

	def __del__(self):
		
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

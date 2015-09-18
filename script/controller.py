
from __future__ import division

import os, sys, pipes,signal

import time
import subprocess
import struct
import numpy as np, math

import serial
from CoordinateGen import CoordinateGen

STARTSEQ = chr(108)
rootSectionLength = 90
middleSectionLength = 122
endSectionLength = 100

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

		self.ser2 = serial.Serial(
			port='/dev/tty.usbmodem14221',
			baudrate=115200,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS,
			timeout=0.5
		)

		if self.ser.isOpen() == 0:
			self.ser.open()
		if self.ser2.isOpen() == 0:
			self.ser2.open()
		self.objChanged = False
		self.first = 1
		self.initialiseJoints()
		while (1):
			# try:
			[points, flag ]= self.getPoints()
			
			if points and flag == 2:
				print "All objects"
				# search for target points.
				base = points["base"]
				obj = points["obj"]
				if self.first:
					self.first = False
					self.prevObj = obj
				
				print "base:{}".format(base)
				print "obj:{}".format(obj)
				startang = np.pi*8/9
				stopang = np.pi/2
				step = 0.2
				ba0_target = startang
				ja0_target = None
				# print "Ja1 : {}".format(self.estimateJa1())
				if not self.objChanged:
					self.trackBasez()
				else:
					self.initialiseJoints()
	 				self.ba0Tracked = False
		 			self.ja0Tracked = False
		 			self.ja1Tracked = False
				while (ba0_target> stopang):

					print "ba0_target: {}".format(ba0_target)
  					[ja0_target, ja1_target] = self.coordinateGen.withAngle(base, obj, rootSectionLength, middleSectionLength, endSectionLength, ba0_target)
  					ba0_target -=step
  					# 
  					if ja0_target is not None: 
  						print "solution found!"
  						ba0_target +=step
  						break
			 	# Begin tracking
			 	if ja0_target is not None:
			 		# begin tracking. Assume fixed target.
			 		print "begin tracking!"
			 		print "ba0_target: {}".format(ba0_target)
			 		print "ja0_target: {}".format(ja0_target)
			 		print "ja1_target: {}".format(ja1_target)
		 			#Track ba0
		 			
		 			#Check if task completed
		 			self.ba0Tracked = False
		 			self.ja0Tracked = False
		 			self.ja1Tracked = False

		 			if not self.objChanged:
		 				self.trackJa0(ja0_target)
		 				pass
		 			if not self.objChanged:
		 				self.trackBa0(ba0_target)

		 			if not self.objChanged:
		 				self.trackJa1(ja1_target)
		 				pass
		 			else:
		 				print "initialising Joints"
		 				self.initialiseJoints()
		 				self.ba0Tracked = False
			 			self.ja0Tracked = False
			 			self.ja1Tracked = False
		 				pass
			 		
			 		if self.ba0Tracked and 	self.ja0Tracked and	self.ja1Tracked :
			 			self.gripClose()
			 			self.ba0Tracked = False
			 			self.ja0Tracked = False
			 			self.ja1Tracked = False
			 		
		 		else:
		 			print "No solution"



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
	def gripClose(self):
		count = 5
		while count > 0:
			count-=1 
			self.ser2.write(STARTSEQ)
			val = 0B11110000
			val |= 1
			cmd = chr(val)
			self.ser2.write(cmd)
			time.sleep(.1)

	def gripOpen(self):
		count = 3
		while count > 0:
			count-=1 
			self.ser2.write(STARTSEQ)
			val = 0B11110000
			val |= (1<<1)
			cmd = chr(val)
			self.ser2.write(cmd)
			time.sleep(.1)

	def trackBasez (self):
		while(not self.objChanged):
			print "Tracking base z"
			[points, flag]= self.getPoints()
			base = points["base"]
			joint0 = points["joint0"]
			joint1 = points["joint1"]
			obj = points["obj"]
			if flag ==2:
				if (self.getDistFromPlane (joint0, joint1, base, obj)>-40):
					# 	#move counterclockwise
					print "counterclockwise"
					self.ser.write(STARTSEQ)
					cmd = chr(0B00010000)
					self.ser.write(cmd)
					time.sleep(.1)
				elif (self.getDistFromPlane (joint0, joint1, base, obj)<-80):
					#move clockwise
					print "clockwise"
					self.ser.write(STARTSEQ)
					cmd = chr(0B00110000)
					self.ser.write(cmd)
					time.sleep(.1)
				else:
					print "Finished tracking base z"
					break
	def trackBa0(self, ba0_target):
		while (not self.objChanged):
			print "Tracking Ba0"
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
					time.sleep(.1)
				elif abs(ba0_target - ba0) < 0.2:
					print "end ba0 tracking"
					self.ba0Tracked = True
					break
			else:
				break
			print "trackBa0:{}, ba0:{}".format(ba0_target, ba0)					
	def trackJa0(self, ja0_target):
		while (not self.objChanged):
			print "Tracking Ja0"
			ja0 = self.estimateJa0()
			if (ja0_target - ja0) > .2:
				# base_0 go forward
				self.ser.write(STARTSEQ)
				cmd = chr(0B10010000)
				self.ser.write(cmd)
				time.sleep(.1)
			elif(ja0_target - ja0) < -.2:
				#base_0 go backward
				self.ser.write(STARTSEQ)
				cmd = chr(0B10110000)
				self.ser.write(cmd)
				time.sleep(.1)
			elif abs(ja0_target - ja0) < 0.2:
				print "end ja0 tracking"
	 			self.ja0Tracked = True
				break
			print "ja0_target:{}, ja0:{}".format(ja0_target,ja0)
	def trackJa1(self, ja1_target):
		while (not self.objChanged):
			print "Tracking Ja1"
			ja1 = self.estimateJa1()
			print "trackJa1:{}, ja1:{}".format(ja1_target, ja1)
			# print "ja1_target: {}, ja1: {}, l2angle:{}, joint1_roll:{}".format(ja1_target, ja1, l2angle, joint1_roll)
			if (ja1_target - ja1) > .2:
				print "backward"
				# joint1 go backward
				self.ser.write(STARTSEQ)
				cmd = chr(0B11110000)
				self.ser.write(cmd)
				time.sleep(.1)
			elif(ja1_target - ja1) < -.2:
				print "forward"
				#joint1 go forward
				self.ser.write(STARTSEQ)
				cmd = chr(0B11010000)
				self.ser.write(cmd)
				time.sleep(.1)
			elif abs(ja1_target - ja1) < 0.2:
				print "end ja1 tracking"
	 			self.ja1Tracked = True
				break
			
	def estimateBa0(self):
		loopagain = False
		while not loopagain:
			[points, flag]= self.getPoints()
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
				self.ba0 = ba0
				return ba0
			else:
				loopagain = True
				continue
	def estimateJa0(self):
		loopagain = False
		while not loopagain:
			[points, flag]= self.getPoints()
			base = points["base"]
			joint0 = points["joint0"]
			joint1 = points["joint1"]
			a = self.getdist( base, joint0)
			b = self.getdist(joint0, joint1)
			c = self.getdist( base, joint1)
			if a == 0 or b == 0:
				loopagain = True
				continue
			ja0 = np.arccos((a**2 + b**2 - c**2)/(2*a*b))
			self.ja0 = ja0
			return ja0
	def estimateJa1(self):
		loopagain = False
		while not loopagain:
			[points, flag]= self.getPoints()
			base = points["base"]
			joint0 = points["joint0"]
			joint1 = points["joint1"]
			joint1_roll = points["joint1_roll"]
			l2angle = np.arctan2(joint1[1]-joint0[1], joint1[0]-joint0[0])

			ja1 = np.pi + ( joint1_roll - l2angle)
			self.ja1 = ja1
			if not ja1:
				loopagain = True
				continue
			return ja1
	def transtoPlane(self, n, pt):
		alpha = np.dot([0,0,-1], n)
		print "plane alpha: {}".format(alpha)
		x = pt[0]*np.cos(alpha)- pt[2]*np.sin(alpha)
		return x

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

	def initialiseJoints(self):
		self.gripOpen()		
		self.trackBa0(np.pi*150/180)
		self.trackJa0(np.pi/2)
		self.trackJa1(2.5)
		print "Joints initialised"

	def getPoints(self): # base, join0, joint1, object, flag
		# flags: 1 = arm only. 2 = all
		loopagain = False
		while not loopagain:
			nums = self.pipein.read(52)

			if nums:
				x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,r  = struct.unpack('fffffffffffff',nums)
				# print "x1: {}, y1: {}, z1:{}\n x2:{},y2:{},z2:{}, \n x3:{},y3:{},z3:{} \n x4:{},y4:{},z4:{}, roll: {}".format(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,r)
				

				points = -x1,-y1,z1,-x2,-y2,z2,-x3,-y3,z3,-x4,-y4,z4,r
				for l in range(len(points)-1):
					if points[l] == 0:
						loopagain == True
						continue
				base= [points[0], points[1], points[2]]
				joint0 = [points[3], points[4], points[5]]
				joint1 = [points[6], points[7], points[8]]
				joint1_roll = points[12]
				obj = [points[9],points[10],points[11]]
				points = {"base": [points[0], points[1], points[2]], "joint0":[points[3], points[4], points[5]], "joint1":[points[6], points[7], points[8]], "joint1_roll": points[12], "obj": [points[9],points[10],points[11]]}

				if not self.first:
					mag = np.linalg.norm(np.subtract(obj, self.prevObj))
					if mag > 60:
						self.objChanged = True
						
						self.prevObj = obj
						
					else:
						self.objChanged = False
				print "objChanged:{}".format(self.objChanged)
				if np.isnan(obj[0]):
					flag = 1
				else:
					flag = 2
				return [points, flag]
			else:
				loopagain == True
				continue

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
			self.ser.close()
			self.ser2.close()
		except:
			pass
		sys.exit()
	def initialiseRoll(self):
		a = np.arctan2(joint1[1]-joint2[1])
if __name__ == "__main__":
	robotarm()

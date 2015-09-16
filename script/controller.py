


import os, sys, pipes,signal
import atexit
import time
import subprocess
import struct




class robotarm(object):
	def __init__(self):
		self.initialiseVision()
		atexit.register(self.exit_fn)	
		while (1):
			try:
				points = self.getPoints()
				# get offset 
				offx = points[0]
				offy = points[1]
				offz = points[2]

				angleOff = atan2()
				alpha = 
				time.sleep(1)
				print offx
			except:
				print "no data.."
	def initialiseVision(self):
		# start vision
		cmd = "./../build/getcoords"
		self.proc = subprocess.Popen(cmd, shell=True,preexec_fn=os.setsid)
		time.sleep(3)
		self.pipein = open("/tmp/autoarm",'r')


	def getPoints(self): # base, join0, joint1, object
		nums = self.pipein.read(52)
		x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,r  = struct.unpack('fffffffffffff',nums)
		print "x1: {}, y1: {}, z1:{}\n x2:{},y2:{},z2:{}, \n x3:{},y3:{},z3:{} \n x4:{},y4:{},z4:{}, roll: {}".format(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,r)
		return -x1,-y1,-z1,-x2,-y2,-z2,-x3,-y3,-z3,-x4,-y4,-z4,r

	def exit_fn(self):
		os.killpg(self.proc.pid, signal.SIGTERM)
		self.pipein.close()
if __name__ == "__main__":
	robotarm()

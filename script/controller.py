import os, sys, pipes,signal
import atexit
import time
import subprocess
import struct


def exit_fn():
	os.killpg(proc.pid, signal.SIGTERM)
	pipein.close()
def read():
	nums = pipein.read(36)
	x1,y1,z1,x2,y2,z2,x3,y3,z3 = struct.unpack('fffffffff',nums)
	print "x1: {}, y1: {}, z1:{}\n x2:{},y2:{},z2:{}, \n x3:{},y3:{},z3:{}".format(x1,y1,z1,x2,y2,z2,x3,y3,z3)
	return x1,y1,z1,x2,y2,z2,x3,y3,z3


atexit.register(exit_fn)
cmd = "./../build/getcoords"
proc = subprocess.Popen(cmd, shell=True,preexec_fn=os.setsid)

time.sleep(3)
pipein = open("/tmp/autoarm",'r')
while(1):
	time.sleep(1)
	try:
		read()
	except:
		print "no data.."
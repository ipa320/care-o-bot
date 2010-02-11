#!/usr/bin/python

import roslib; roslib.load_manifest('cob3_dashboard')

from home import *
from folded import *
from simple_trajectory import *

class arm:
	def Home(self):
		home()

	def Folded(self):
		folded() 
	   
	def SimpleTrajectory(self):
		simple_trajectory()

class sdh:
	def Home(self):
		print "sdh.Home"
   
	def Close(self):
		print "sdh.Close"
		
class cob3:
	arm=arm()
	sdh=sdh()

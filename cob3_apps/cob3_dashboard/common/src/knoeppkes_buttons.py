#!/usr/bin/python

from knoeppkes_actions import *

panels = [  
  ( "arm", [ 
	 ( "home", cob3.arm.Home),
	 ( "folded", cob3.arm.Folded),
	 ( "simple_trajectory", cob3.arm.SimpleTrajectory),
	 ]),

  ( "sdh", [
	( "home",  cob3.sdh.Home),
	( "close",  cob3.sdh.Close),
   ]),

   ]

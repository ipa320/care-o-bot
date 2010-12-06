#!/bin/bash

# Author: 
# Florian Weisshardt, mail:florian.weisshardt@ipa.fhg.de
#
# Description:
# This file sets the ros environment variables. Executed 
# manually by makeconfig or automatically through bashrc

# add path to $ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$1:$ROS_PACKAGE_PATH

# define amount of ros prozesses during build for multi-prozessor machines
COUNT=$(cat /proc/cpuinfo | grep 'processor' | wc -l)
COUNT=$(echo "$COUNT*2" | bc)
export ROS_PARALLEL_JOBS=-j$COUNT

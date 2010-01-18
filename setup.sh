#!/bin/bash

# Author: 
# Florian Weisshardt, mail:florian.weisshardt@ipa.fhg.de
#
# Description:
# This file sets the ros environment variables. Executed 
# manually by makeconfig or automatically through bashrc

# add path to $ROS_PACKAGE_PATH and $ROS_STACK_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$1
export ROS_STACK_PATH=$ROS_STACK_PATH:$1

# define amount of ros prozesses during build for multi-prozessor machines
export ROS_PARALLEL_JOBS=-j2

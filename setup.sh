#!/bin/bash

# Author: 
# Florian Weisshardt, mail:florian.weisshardt@ipa.fhg.de
#
# Description:
# This file sets the ros environment variables

# add path for cob-ros-pkg to $ROS_PACKAGE_PATH
export ROOT=$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROOT/.
export ROS_STACK_PATH=$ROS_STACK_PATH:$ROOT/.
export PYTHONPATH=$PYTHONPATH:$ROS_ROOT/core/roslib/src

# define amount of ros prozesses during build for multi-prozessor machines
export ROS_PARALLEL_JOBS=-j2

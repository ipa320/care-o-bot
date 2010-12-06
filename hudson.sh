#!/bin/bash

STACKS="
cob_apps
cob_common
cob_driver
cob_extern
cob_simulation
"

# checking for ROS release
if [ $# != 1 ]; then
	echo "ERROR: no ROS release specified"
	exit 1
elif [ $1 = "boxturtle" ]; then
	RELEASE=boxturtle
elif [ $1 = "latest" ]; then
	RELEASE=latest
elif [ $1 = "cturtle" ]; then
	RELEASE=cturtle
else
	echo "ERROR: no valid ROS release specified"
	exit 1
fi

echo "==> RELEASE =" $RELEASE
echo "==> WORKSPACE =" $WORKSPACE

# installing ROS release
sudo apt-get install ros-$RELEASE-pr2all -y

# perform rosinstall
rm $WORKSPACE/../.rosinstall
rosinstall $WORKSPACE/.. $WORKSPACE/./$RELEASE.rosinstall $WORKSPACE
. $WORKSPACE/../setup.sh

# define amount of ros prozesses during build for multi-prozessor machines
COUNT=$(cat /proc/cpuinfo | grep 'processor' | wc -l)
COUNT=$(echo "$COUNT*2" | bc)
export ROS_PARALLEL_JOBS=-j$COUNT

#build farm stuff
#export PATH=/usr/lib/ccache/:$PATH
#export DISTCC_HOSTS='localhost distcc@hektor.ipa.fhg.de/8 distcc@chaos.ipa.fhg.de/4'
#export CCACHE_PREFIX=distcc
#export ROS_PARALLEL_JOBS=-j28

echo "==> ROS_ROOT =" $ROS_ROOT
echo "==> ROS_PACKAGE_PATH =" $ROS_PACKAGE_PATH

# installing dependencies and building
rosdep install $STACKS
rosmake $STACKS --skip-blacklist

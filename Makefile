# Author: 
# Florian Weisshardt, mail:florian.weisshardt@ipa.fhg.de
#
# Description:
# This Makefile helps to build the cob3_driver repository. Define all 
# packages you want to build in the variable PACKAGES_TO_BUILD.
#
# Usage:
# run 'make or make all' to build all packages listed below
# run 'make ros' to build all packages listed below with the recursive build system of ros
# run 'make ros-ignore-errors' to build all packages listed below with the recursive build system of ros ignoring errored builds
# run 'make clean' to clean up all packages listed below

#--------------------------------------------------------------------
# list all packages here
PACKAGES_TO_BUILD=\
	libpcan\
	libwm4\
	libmesasr\
	cob3_msgs\
	cob3_srvs\
	sickS300\
	sdh\
	cob3_cameraSensors\
	powercube_chain\
	cob3_arm\
	cob3_platform\
	cob3_teleop\
#--------------------------------------------------------------------

all:
	@for dir in $(PACKAGES_TO_BUILD); do \
		$(MAKE) -C $$(rospack find $$dir); \
		done

ros:
	rosmake $(PACKAGES_TO_BUILD)

ros-ignore-errors:
	rosmake $(PACKAGES_TO_BUILD) -r -v

ros-pre-clean:
	rosmake $(PACKAGES_TO_BUILD) -s --pre-clean

clean:
	@for dir in $(PACKAGES_TO_BUILD); do \
		$(MAKE) -C $$(rospack find $$dir) clean; \
		done

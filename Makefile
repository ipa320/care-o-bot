# Author: 
# Florian Weisshardt, mail:florian.weisshardt@ipa.fhg.de
#
# Description:
# This Makefile helps to build the cob3_driver repository. Define all 
# packages you want to build in the variable PACKAGES_TO_BUILD.
#
# Usage:
# run 'make or make all' to build all packages listed above
# run 'make ros' to build all packages listed above with the recursive build system of ros
# run 'make clean' to clean up all packages listed above

#--------------------------------------------------------------------
# list all packages here
PACKAGES_TO_BUILD=\
	cob3_msgs\
	cob3_srvs\
	sickS300\
	cob3_cameraSensors\
	powercube_chain\
	cob3_arm\
#	cob3_platform\
#--------------------------------------------------------------------

all:
	@for dir in $(PACKAGES_TO_BUILD); do \
		$(MAKE) -C $$(rospack find $$dir); \
		done

ros:
	rosmake $(PACKAGES_TO_BUILD)

clean:
	@for dir in $(PACKAGES_TO_BUILD); do \
		$(MAKE) -C $$(rospack find $$dir) clean; \
		done

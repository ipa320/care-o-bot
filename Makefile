#/****************************************************************
# *
# * Copyright (c) 2010
# *
# * Fraunhofer Institute for Manufacturing Engineering	
# * and Automation (IPA)
# *
# * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# *
# * Project name: care-o-bot
# * ROS stack name:
# * ROS package name:
# * Description: This Makefile helps to build the care-o-bot repository. Define
# *     all packages you want to build in the variable PACKAGES_TO_BUILD.
# * Usage:
# *     run 'make or make all' to build all packages listed below
# *     run 'make ros' to build all packages listed below with the recursive build system of ros
# *     run 'make ros-ignore-errors' to build all packages listed below with the recursive build system of ros ignoring errored builds
# *     run 'make ros-preclean' to build all packages listed below with the recursive build system of ros with the pre-clean option
# *     run 'make ros-deps' to install all system dependencies for the packages listed below
# *     run 'make clean' to clean up all packages listed below
# *
# * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# *			
# * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# * Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# *
# * Date of creation: Jan 2010
# * ToDo:
# *
# * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# *     * Redistributions of source code must retain the above copyright
# *       notice, this list of conditions and the following disclaimer.
# *     * Redistributions in binary form must reproduce the above copyright
# *       notice, this list of conditions and the following disclaimer in the
# *       documentation and/or other materials provided with the distribution.
# *     * Neither the name of the Fraunhofer Institute for Manufacturing 
# *       Engineering and Automation (IPA) nor the names of its
# *       contributors may be used to endorse or promote products derived from
# *       this software without specific prior written permission.
# *
# * This program is free software: you can redistribute it and/or modify
# * it under the terms of the GNU Lesser General Public License LGPL as 
# * published by the Free Software Foundation, either version 3 of the 
# * License, or (at your option) any later version.
# * 
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU Lesser General Public License LGPL for more details.
# * 
# * You should have received a copy of the GNU Lesser General Public 
# * License LGPL along with this program. 
# * If not, see <http://www.gnu.org/licenses/>.
# *
# ****************************************************************/

#--------------------------------------------------------------------
# list all packages here
PACKAGES_TO_BUILD=\
	libpcan\
	libwm4\
	libm5api\
	libntcan\
	cob3_msgs\
	cob3_srvs\
	sick_s300\
	cob3_hokuyo\
	sdh\
	cob3_camera_sensors\
	powercube_chain\
	cob3_arm\
	cob3_arm_ik\
	cob3_platform\
	cob3_tf_broadcaster\
	cob3_teleop\
	cob3_test\
	cob3_defs\
	cob3_ogre\
	cob3_gazebo\
#--------------------------------------------------------------------

all:
	make ros
ros:
	rosmake $(PACKAGES_TO_BUILD)

ros-ignore-errors:
	rosmake $(PACKAGES_TO_BUILD) -r -v

ros-pre-clean:
	rosmake $(PACKAGES_TO_BUILD) -s --pre-clean
	
ros-deps:
	rosdep install $(PACKAGES_TO_BUILD)

clean:
	@for dir in $(PACKAGES_TO_BUILD); do \
		$(MAKE) -C $$(rospack find $$dir) clean; \
		done

/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob3_driver
 * ROS package name: cob3_camera_sensors
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
 *
 * Date of creation: Jan 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/PointCloud.h>

// external includes
#include <cob_camera_sensors/AbstractRangeImagingSensor.h>

using namespace ipa_CameraSensors;

//##########################
//#### global variables ####
//--

//##################################
//#### topic callback functions ####
// function will be called when a new message arrives on a topic
//--

//####################################
//#### service callback functions ####
// function will be called when a service is querried
/*bool srvCallback_GetCameraInfo(cob3_srvs::GetCameraInfo::Request &req,
                               cob3_srvs::GetCameraInfo::Response &res )
{
    ROS_INFO("get camera info");
    sensor_msgs::CameraInfo cameraInfo;
    //TODO: get real camera Info
    res.cameraInfo = cameraInfo;
    res.success = 0; // 0 = true, else = false
    return true;
}*/

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "swiss_ranger");

	// create a handle for this node, initialize node
	ros::NodeHandle n;
	
    // topics to publish
    ros::Publisher topicPub_PointCloud = n.advertise<sensor_msgs::PointCloud>("PointCloud", 1);
    
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //ros::ServiceServer srvServer_GetCameraInfo = n.advertiseService("GetCameraInfo", srvCallback_GetCameraInfo);
        
    // service clients
    //--
    
    // external code 	 	 		
	IplImage* image = 0;
	std::string directory = "../files/";

	AbstractRangeImagingSensor* rangeImagingSensor = 0;
	rangeImagingSensor = ipa_CameraSensors::CreateRangeImagingSensor_SR3000();

	if (rangeImagingSensor->Init(directory) & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - CameraDataViewerControlFlow::Init:" << std::endl;
		std::cerr << "\t ... Error while initializing range imaging sensor.\n";
		return ipa_Utils::RET_FAILED;
	}
	
	if (rangeImagingSensor->Open() & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - CameraDataViewerControlFlow::Init:" << std::endl;
		std::cerr << "\t ... Error while opening range imaging sensor.\n";
		return ipa_Utils::RET_FAILED;
	}

    // main loop
    ros::Rate loop_rate(5); // Hz
	while (n.ok())
	{
		if (image != 0)
		{
			cvReleaseImage(&image);
			image = 0;
		}

		if(rangeImagingSensor->AcquireImages2(0, 0, &image, false, false) & ipa_Utils::RET_FAILED)
		{	
			std::cerr << "ERROR - CameraDataViewerControlFlow::ShowSharedImage:" << std::endl;
			std::cerr << "\t ... Range image acquisition failed" << std::endl;
			return ipa_Utils::RET_FAILED;	
		}

        // create message
		int numPoints = image->width*image->height;
        sensor_msgs::PointCloud msg;
		msg.header.stamp = ros::Time::now(); 
        msg.points.resize(numPoints);
 		for (int i = 0; i < numPoints; i++)
		{
			msg.points[i].x = ((float*)image->imageData)[3*i + 0];
			msg.points[i].y = ((float*)image->imageData)[3*i + 1];
			msg.points[i].z = ((float*)image->imageData)[3*i + 2];
		}

        // publish message
        ROS_INFO("published PointCloud from swiss_ranger");

        topicPub_PointCloud.publish(msg);

        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }

	rangeImagingSensor->Close();
	ipa_CameraSensors::ReleaseRangeImagingSensor(rangeImagingSensor);
    
    return 0;
}

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

// ROS service includes
//--

// external includes
#include <cob3_camera_sensors/AbstractRangeImagingSensor.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // topics to publish
		//--
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_PointCloud;
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        //--

        // Constructor
        NodeClass()
        {
            //topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
            topicSub_PointCloud= n.subscribe("PointCloud", 1, &NodeClass::topicCallback_PointCloud, this);

			// uncomment to display image
			cvNamedWindow("PointCloud", CV_WINDOW_AUTOSIZE);
        }
        
        // Destructor
        ~NodeClass() 
        {
//			cvDestroyAllWindows();
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_PointCloud(const sensor_msgs::PointCloud::ConstPtr& msg)
        {
        	ROS_INFO("received PointCloud from swiss_ranger");
			// display PointCloud
			IplImage* image = cvCreateImage(cvSize(176, 144), IPL_DEPTH_32F, 3);
			for (unsigned int i=0; i<msg->points.size(); i++) 
			{
				((float*)image->imageData)[3*i+0] = (msg->points[i]).x;
				((float*)image->imageData)[3*i+1] = (msg->points[i]).y;
				((float*)image->imageData)[3*i+2] = (msg->points[i]).z;
			}

			IplImage* image_show = cvCreateImage(cvSize(176, 144), IPL_DEPTH_8U, 3);
			ipa_Utils::ConvertToShowImage(image, image_show, 3);

			// uncomment to display image
			cvShowImage("PointCloud", image_show);
			cvWaitKey(10);
        }

        // service callback functions
        // function will be called when a service is querried
		//--
        
        // other function declarations
        //--
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "point_cloud_show");
    
    NodeClass nodeClass;
 
    while(nodeClass.n.ok())
    {
        ros::spinOnce();
    }
    
//    ros::spin();

    return 0;
}

//##################################
//#### function implementations ####
//--

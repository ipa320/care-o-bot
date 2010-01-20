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
 * ROS package name: sdh
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
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
#include <cob3_msgs/CmdPos.h>
#include <sensor_msgs/JointState.h>

// ROS service includes
//--

// external includes
#include <include/sdh.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // declaration of topics to publish
        ros::Publisher topicPub_JointState;
        ros::Publisher topicPub_ActuatorState;
        
	    // declaration of topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_CmdPos;
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        cSDH *sdh; 

        // Constructor
        NodeClass()
        {
            // implementation of topics to publish
            topicPub_JointState = n.advertise<sensor_msgs::JointState>("JointState", 1);
            
            // implementation of topics to subscribe
            topicSub_CmdPos = n.subscribe("CmdPos", 1, &NodeClass::topicCallback_CmdPos, this);
            
            // pointer to sdh
            sdh = new cSDH();
        }
        
        // Destructor
        ~NodeClass() 
        {
        	sdh->Close();
        	delete sdh;
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_CmdPos(const cob3_msgs::CmdPos::ConstPtr& msg)
        {
            ROS_INFO("Received new CmdPos [%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f]", msg->cmdPos[0], msg->cmdPos[1], msg->cmdPos[2], msg->cmdPos[3], msg->cmdPos[4], msg->cmdPos[5], msg->cmdPos[6]);
            
            //TODO: send msg data to hardware
          	std::vector<int> axes;
		    for(int i=0; i<7; i++)
		    {
				axes.push_back(i);
			}
		
			try
			{
				sdh->SetAxisTargetAngle( axes, msg->cmdPos );
			}
			catch (cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}
			
			try
			{
				sdh->MoveHand(true);
			}
			catch (cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
		   	 }
        }

        // service callback functions
        // function will be called when a service is querried
        //--
        
        // other function declarations
        void initSdh()
        {
       	    // open can for sdh 
       	    //TODO: read from parameter
			int _net=0;
			unsigned long _baudrate=1000000;
			double _timeout=-1.0;
			unsigned int _id_read=43;
			unsigned int _id_write=42;
	
		    try
			{
				sdh->OpenCAN_ESD( _net, _baudrate, _timeout, _id_read, _id_write );
			}
			catch (cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}
	   	}

        void updateJointState()
        {
        	ROS_INFO("updateJointState");
            //get actual joint positions 
            //TODO: get from hardware
			std::vector<double> actualAngles;
            std::vector<int> axes;

		    for(int i=0; i<7; i++)
		    {
				axes.push_back(i);
			}

            try
            {
				actualAngles = sdh->GetAxisActualAngle( axes );
			}
			catch (cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
		   	}

			//fill message
			sensor_msgs::JointState msg;
			msg.position.resize(7);
			for(int i=0; i<7; i++)
		    {
				msg.position[i] = actualAngles[i];
			}

            //publish the message
            topicPub_JointState.publish(msg);

            ROS_INFO("published JointState: [%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f]", msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]);
        }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "sdh");
    
    NodeClass nodeClass;
    
	// initialize sdh	
	nodeClass.initSdh();
 
 	ros::Rate loop_rate(5); // Hz
    while(nodeClass.n.ok())
    {
        // publish JointState
        nodeClass.updateJointState();
    
        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
//    ros::spin();

    return 0;
}

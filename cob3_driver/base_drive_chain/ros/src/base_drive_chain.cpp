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
 * ROS stack name: cob3_drivers
 * ROS package name: base_drive_chain
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2009:
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
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

// ROS service includes
#include <std_srvs/Empty.h>

// external includes
#include <CanCtrlPltfCoB3.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // topics to publish
        ros::Publisher topicPub_JointState;
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_JointStateCmd;
        
        // service servers
        ros::ServiceServer srvServer_Init;
        ros::ServiceServer srvServer_Reset;
        ros::ServiceServer srvServer_Shutdown;
        ros::ServiceServer srvServer_SetMotionType;
        ros::ServiceServer srvServer_GetJointState;
            
        // service clients
        //--
        
        // global variables
		// generate can-node handle -> should this realy be public?
		CanCtrlPltfCoB3 m_CanCtrlPltf;
		bool isInitialized;

        // Constructor
        NodeClass()
        {
			// initialization of variables
			isInitialized = false;

			// implementation of topics
            topicPub_JointState = n.advertise<sensor_msgs::JointState>("JointState", 1);
            topicSub_JointStateCmd = n.subscribe("JointStateCmd", 1, &NodeClass::topicCallback_JointStateCmd, this);

            // implementation of service servers
            srvServer_Init = n.advertiseService("Init", &NodeClass::srvCallback_Init, this);
            srvServer_Reset = n.advertiseService("Reset", &NodeClass::srvCallback_Reset, this);
            srvServer_Shutdown = n.advertiseService("Shutdown", &NodeClass::srvCallback_Shutdown, this);
            srvServer_isPltfError = n.advertiseService("isPltfError", &NodeClass::srvCallback_isPltfError, this);
            srvServer_GetJointState = n.advertiseService("GetJointState", &NodeClass::srvCallback_GetJointState, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_JointStateCmd(const sensor_msgs::JointState::ConstPtr& msg)
        {
            ROS_INFO("this is topicCallback_demoSubscribe");
        }

        // service callback functions
        // function will be called when a service is querried

		// Init Can-Configuration
        bool srvCallback_Init(cob3_srvs::Switch::Request &req,
                              cob3_srvs::Switch::Response &res )
        {
            if(isInitialized == false)
            {
                ROS_INFO("...initializing can-nodes...");
				isInitialized = m_CanCtrlPltf.initPltf();
				res.success = isInitialized;
				if(isInitialized)
				{
                	ROS_INFO("Can-Node initialized");
				}
				else
				{
                	res.errorMessage.data = "platform already initialized";
                	ROS_INFO("Initialization FAILED");
				}
            }
            else
            {
                ROS_ERROR("...platform already initialized...");
                res.success = false;
                res.errorMessage.data = "platform already initialized";
            }            
            return true;
        }
		
		// reset Can-Configuration
        bool srvCallback_Reset(cob3_srvs::Switch::Request &req,
                                     cob3_srvs::Switch::Response &res )
        {
			res.success = m_CanCtrlPltf.resetPltf();
			if (res.success)
            	ROS_INFO("Can-Node resetted");
			else
            	ROS_INFO("Reset of Can-Node FAILED");

            return true;
        }
		
		// shutdown Drivers and Can-Node
        bool srvCallback_Shutdown(cob3_srvs::Switch::Request &req,
                                     cob3_srvs::Switch::Response &res )
        {
			res.success = m_CanCtrlPltf.shutdownPltf();
			if (res.success)
            	ROS_INFO("Drives shut down");
			else
            	ROS_INFO("Shutdown of Drives FAILED");

            return true;
        }


        bool srvCallback_GetJointState(cob3_srvs::GetJointState::Request &req,
                                     cob3_srvs::GetJointState::Response &res )
        {
            ROS_INFO("This is srvCallback_demoService");
            return true;
        }
        
        // other function declarations
        void demoFunction();
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "base_drive_chain");
    
    NodeClass nodeClass;
 	
	// currently only waits for callbacks -> if it should run cyclical
	// -> specify looprate
 	// ros::Rate loop_rate(10); // Hz 
    while(nodeClass.n.ok())
    {

        ros::spinOnce();
		// -> let it sleep for a while
        //loop_rate.sleep();
    }
    
//    ros::spin();

    return 0;
}

//##################################
//#### function implementations ####
void NodeClass::demoFunction()
{
    ROS_INFO("This is the demoFunction");
}

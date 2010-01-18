/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Inistitute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * All rights reserved.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name:
 * ROS package name:
 * Function name:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author:
 * Supervised by:
 *
 * Date of creation:
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
 *     * Neither the name of the Fraunhofer Inistitute for Manufacturing 
 *		 Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

// ROS service includes
#include <std_srvs/Empty.h>

// external includes
//--

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // topics to publish
        ros::Publisher topicPub_demoPublish;
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_demoSubscribe;
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        //--

        // Constructor
        NodeClass()
        {
            topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
            topicSub_demoSubscribe = n.subscribe("demoSubscribe", 1, &NodeClass::topicCallback_demoSubscribe, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_demoSubscribe(const std_msgs::String::ConstPtr& msg)
        {
            ROS_INFO("this is topicCallback_demoSubscribe");
        }

        // service callback functions
        // function will be called when a service is querried
        bool srvCallback_demoService(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res )
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
    ros::init(argc, argv, "node_name");
    
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
void NodeClass::demoFunction()
{
    ROS_INFO("This is the demoFunction");
}

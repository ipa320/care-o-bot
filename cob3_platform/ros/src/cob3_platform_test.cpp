//##################
//#### includes ####

// standard includes
//#include <string>
//#include <sstream>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

// ROS service includes
#include <cob3_srvs/Init.h>
#include <cob3_srvs/Stop.h>
#include <cob3_srvs/Shutdown.h>

// external includes
//--

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
//--

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob3_driver_platform_test");

	// create a handle for this node, initialize node
	ros::NodeHandle n;

    // topics to publish
    ros::Publisher topicPub_CmdVel = n.advertise<geometry_msgs::Twist>("cob3/platform/CmdVel", 1);
        
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    ros::ServiceClient srvClient_Init = n.serviceClient<cob3_srvs::Init>("cob3/platform/Init");
    ros::ServiceClient srvClient_Stop = n.serviceClient<cob3_srvs::Stop>("cob3/platform/Stop");
    ros::ServiceClient srvClient_Shutdown = n.serviceClient<cob3_srvs::Shutdown>("cob3/platform/Shutdown");
    
    // external code
	bool srv_querry = false;
	int srv_execute = 1;
    
    char c;
         
    // main loop
    while(n.ok())
    {
        // process user inputs
        std::cout << "Choose service to test ([s]top, [i]nit, shut[d]own, send[C]ommand, [e]xit): ";
        
        std::cin >> c;

        switch(c)
        {
            case 's':
            {
                //ROS_INFO("querry service [cob3/platform/Stop]");
                cob3_srvs::Stop srv;
                srv_querry = srvClient_Stop.call(srv);
                srv_execute = srv.response.success;
              	break;
            }

            case 'd':
            {
                //ROS_INFO("querry service [cob3/platform/Shutdown]");
                cob3_srvs::Shutdown srv;
                srv_querry = srvClient_Shutdown.call(srv);
                srv_execute = srv.response.success;
              	break;
            }
            
            case 'i':
            {
            	//ROS_INFO("querry service [cob3/platform/Init]");
                cob3_srvs::Init srv;
                srv_querry = srvClient_Init.call(srv);
                srv_execute = srv.response.success;
              	break;
            }
            
            case 'C':
            {
                // create message
                geometry_msgs::Twist msg;
                
                std::cout << "Choose preset target velocity ([0] = , [1] = , [2] = ): ";
                std::cin >> c;
                if (c == '0')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = 0;
                }
                else if (c == '1')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = 1;
                }
                else if (c == '2')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = -1;
                }
                else
                {
                    ROS_ERROR("invalid target");
                }
                
                topicPub_CmdVel.publish(msg);
                
                std::cout << std::endl;
                srv_querry = true;
                srv_execute = 0;
                break;
            }
            
            case 'e':
            {
                ROS_INFO("exit. Shutting down node");
                std::cout << std::endl;
                return 0;
                break;
            }
            
            default:
            {
                std::cout << "ERROR: invalid input, try again..." << std::endl << std::endl;
            }
        } //switch
        
		if (!srv_querry)
		{
			ROS_ERROR("Failed to call service");
		}
		else
		{
			ROS_INFO("Service call succesfull");
			
			if (srv_execute != 0)
			{
				ROS_ERROR("Service execution failed");
			}
		}
		
	} //while

    return 0;
} //main

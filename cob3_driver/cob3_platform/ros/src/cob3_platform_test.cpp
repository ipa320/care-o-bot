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
    ros::Publisher topicPub_CmdVel = n.advertise<geometry_msgs::Twist>("cmdVel", 1);
        
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    ros::ServiceClient srvClient_Init = n.serviceClient<cob3_srvs::Init>("Init");
    ros::ServiceClient srvClient_Stop = n.serviceClient<cob3_srvs::Stop>("Stop");
    ros::ServiceClient srvClient_Shutdown = n.serviceClient<cob3_srvs::Shutdown>("Shutdown");
    
    // external code
	bool srv_querry = false;
	int srv_execute = 1;
	std::string srv_errorMessage = "no error";
    
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
                //ROS_INFO("querry service [Stop]");
                cob3_srvs::Stop srv;
                srv_querry = srvClient_Stop.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
                std::cout << srv.response.errorMessage.data.c_str() << std::endl;
              	break;
            }

            case 'd':
            {
                //ROS_INFO("querry service [Shutdown]");
                cob3_srvs::Shutdown srv;
                srv_querry = srvClient_Shutdown.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'i':
            {
            	//ROS_INFO("querry service [Init]");
                cob3_srvs::Init srv;
                srv_querry = srvClient_Init.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'C':
            {
                // create message
                geometry_msgs::Twist msg;
                
                std::cout << "Choose preset target velocity ([0,1,2,3,4,5,6]): ";
                std::cin >> c;
                if (c == '0')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = 0;
                }
                else if (c == '1')
                {
                    msg.linear.x = -0.02;
                    msg.linear.y = 0.02;
                    msg.angular.z = 0;
                }
                else if (c == '2')
                {
                    msg.linear.x = -0.02;
                    msg.linear.y = 0;
                    msg.angular.z = 0;
                }
                else if (c == '3')
                {
                    msg.linear.x = -0.02;
                    msg.linear.y = -0.02;
                    msg.angular.z = 0;
                }
                else if (c == '4')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0.02;
                    msg.angular.z = 0;
                }
                else if (c == '5')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = 0;
                }
                else if (c == '6')
                {
                    msg.linear.x = 0;
                    msg.linear.y = -0.02;
                    msg.angular.z = 0;
                }
                else if (c == '7')
                {
                    msg.linear.x = 0.02;
                    msg.linear.y = 0.02;
                    msg.angular.z = 0;
                }
                else if (c == '8')
                {
                    msg.linear.x = 0.02;
                    msg.linear.y = 0;
                    msg.angular.z = 0;
                }
                else if (c == '9')
                {
                    msg.linear.x = 0.02;
                    msg.linear.y = -0.02;
                    msg.angular.z = 0;
                }
                else if (c == '+')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = 0.02;
                }
                else if (c == '-')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = -0.02;
                }
                else
                {
                    ROS_ERROR("invalid target");
                }
                
                topicPub_CmdVel.publish(msg);
                
                std::cout << std::endl;
                srv_querry = true;
                srv_execute = 0;
            	srv_errorMessage = "no error";
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
				ROS_ERROR("Service execution failed, errorMessage: %s", srv_errorMessage.c_str());
			}
		}
		
	} //while

    return 0;
} //main

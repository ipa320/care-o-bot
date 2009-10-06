//##################
//#### includes ####

// standard includes
//#include <string>
//#include <sstream>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <cob3_msgs/CmdVel.h>
#include <cob3_msgs/CmdPos.h>

// ROS service includes
#include <cob3_srvs/Init.h>
#include <cob3_srvs/Home.h>
#include <cob3_srvs/Stop.h>
#include <cob3_srvs/SetOperationMode.h>

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
	ros::init(argc, argv, "cob3_driver_arm_test");

	// create a handle for this node, initialize node
	ros::NodeHandle n;

    // topics to publish
    ros::Publisher topicPub_Vel = n.advertise<cob3_msgs::CmdVel>("cob3/arm/CmdVel", 1);
    ros::Publisher topicPub_Pos = n.advertise<cob3_msgs::CmdPos>("cob3/arm/CmdPos", 1);
        
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    ros::ServiceClient srvClient_Init = n.serviceClient<cob3_srvs::Init>("cob3/arm/Init");
    ros::ServiceClient srvClient_Home = n.serviceClient<cob3_srvs::Home>("cob3/arm/Home");
    ros::ServiceClient srvClient_Stop = n.serviceClient<cob3_srvs::Stop>("cob3/arm/Stop");
    ros::ServiceClient srvClient_SetOperationMode = n.serviceClient<cob3_srvs::SetOperationMode>("cob3/arm/SetOperationMode");
    
    // external code
	bool srv_querry = false;
	int srv_execute = 1;
	std::string srv_errorMessage = "no error";
    
    char c;
         
    // main loop
    while(n.ok())
    {
        // process user inputs
        std::cout << "Choose service to test ([s]top, [i]nit, [h]ome, setOperation[M]ode, send[C]ommand, [e]xit): ";
        
        
        std::cin >> c;

        switch(c)
        {
            case 's':
            {
                //ROS_INFO("querry service [cob3/arm/Stop]");
                cob3_srvs::Stop srv;
                srv_querry = srvClient_Stop.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'i':
            {
            	//ROS_INFO("querry service [cob3/arm/Stop]");
                cob3_srvs::Init srv;
                srv_querry = srvClient_Init.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'h':
            {
            	//ROS_INFO("querry service [cob3/arm/Home]");
                cob3_srvs::Home srv;
                srv_querry = srvClient_Home.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'M':
            {
                cob3_srvs::SetOperationMode srv;
                
                std::cout << "Choose operation mode ([v] = velocity controll, [p] = position controll): ";
                std::cin >> c;
                if (c == 'v')
                {
                    srv.request.operationMode.data = "velocity";
                }
                else if (c == 'p')
                {
                    srv.request.operationMode.data = "position";
                }
                else
                {
                    srv.request.operationMode.data = "none";
                }
                ROS_INFO("changing operation mode to: %s controll", srv.request.operationMode.data.c_str());
                
                //ROS_INFO("querry service [cob3/arm/SetOperationMode]");
                srv_querry = srvClient_SetOperationMode.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
                break;
            }
            
            case 'C':
            {
                std::cout << "Choose command type ([v] = velocity command, [p] = position command): ";
                std::cin >> c;
                if (c == 'v')
                {
                    // create message
                    int DOF = 7;
                    std::vector<double> vel;
                    vel.resize(DOF);
                    cob3_msgs::CmdVel msg;
                    msg.set_cmdVel_size(DOF);
                    
                    std::cout << "Choose preset target velocity ([0] = , [1] = , [2] = ): ";
                    std::cin >> c;
                    if (c == '0')
                    {
                        msg.cmdVel[0] = 0;
                        msg.cmdVel[1] = 0;
                        msg.cmdVel[2] = 0;
                        msg.cmdVel[3] = 0;
                        msg.cmdVel[4] = 0;
                        msg.cmdVel[5] = 0;
                        msg.cmdVel[6] = 0;
                    }
                    else if (c == '1')
                    {
                        msg.cmdVel[0] = 1;
                        msg.cmdVel[1] = 1;
                        msg.cmdVel[2] = 1;
                        msg.cmdVel[3] = 1;
                        msg.cmdVel[4] = 1;
                        msg.cmdVel[5] = 1;
                        msg.cmdVel[6] = 1;
                    }
                    else if (c == '2')
                    {
                        msg.cmdVel[0] = 2;
                        msg.cmdVel[1] = 2;
                        msg.cmdVel[2] = 2;
                        msg.cmdVel[3] = 2;
                        msg.cmdVel[4] = 2;
                        msg.cmdVel[5] = 2;
                        msg.cmdVel[6] = 2;
                    }
                    else
                    {
                        ROS_ERROR("invalid target");
                    }
                    
                    topicPub_Vel.publish(msg);
                }
                else if (c == 'p')
                {
                    // create message
                    int DOF = 7;
                    std::vector<double> pos;
                    pos.resize(DOF);
                    cob3_msgs::CmdPos msg;
                    msg.set_cmdPos_size(DOF);
                    
                    std::cout << "Choose preset target position ([0] = , [1] = , [2] = ): ";
                    std::cin >> c;
                    if (c == '0')
                    {
                        msg.cmdPos[0] = 0;
                        msg.cmdPos[1] = 0;
                        msg.cmdPos[2] = 0;
                        msg.cmdPos[3] = 0;
                        msg.cmdPos[4] = 0;
                        msg.cmdPos[5] = 0;
                        msg.cmdPos[6] = 0;
                    }
                    else if (c == '1')
                    {
                        msg.cmdPos[0] = 1;
                        msg.cmdPos[1] = 1;
                        msg.cmdPos[2] = 1;
                        msg.cmdPos[3] = 1;
                        msg.cmdPos[4] = 1;
                        msg.cmdPos[5] = 1;
                        msg.cmdPos[6] = 1;
                    }
                    else if (c == '2')
                    {
                        msg.cmdPos[0] = 1;
                        msg.cmdPos[1] = 2;
                        msg.cmdPos[2] = 2;
                        msg.cmdPos[3] = 2;
                        msg.cmdPos[4] = 2;
                        msg.cmdPos[5] = 2;
                        msg.cmdPos[6] = 2;
                    }
                    else
                    {
                        ROS_ERROR("invalid target");
                    }
                    
                    topicPub_Pos.publish(msg);
                    
                }
                else
                {
                    ROS_ERROR("invalid command type");
                }

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
				ROS_ERROR("Service execution failed. Error message: %s", srv_errorMessage.c_str());
			}
		}
		
	} //while

    return 0;
} //main

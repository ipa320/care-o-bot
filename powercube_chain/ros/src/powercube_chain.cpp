//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <cob3_msgs/CmdPos.h>
#include <cob3_msgs/CmdVel.h>
#include <cob3_msgs/ActuatorState.h>

// ROS service includes
#include <cob3_srvs/Init.h>
#include <cob3_srvs/Home.h>
#include <cob3_srvs/Stop.h>
#include <cob3_srvs/SetOperationMode.h>

// external includes
#include <PowerCubeCtrl.h>

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
        ros::Subscriber topicSub_CmdVel;
        
        // declaration of service servers
        ros::ServiceServer srvServer_Init;
        ros::ServiceServer srvServer_Home;
        ros::ServiceServer srvServer_Stop;
        ros::ServiceServer srvServer_SetOperationMode;
            
        // declaration of service clients
        //--
        
        // global variables
        PowerCubeCtrl* PCube;

        // Constructor
        NodeClass()
        {
        	PCube = new PowerCubeCtrl();
        
            // implementation of topics to publish
            topicPub_JointState = n.advertise<sensor_msgs::JointState>("cob3/arm/JointState", 1);
            topicPub_ActuatorState = n.advertise<cob3_msgs::ActuatorState>("cob3/arm/ActuatorState", 1);
            
            // implementation of topics to subscribe
            topicSub_CmdPos = n.subscribe("cob3/arm/CmdPos", 1, &NodeClass::topicCallback_CmdPos, this);
            topicSub_CmdVel = n.subscribe("cob3/arm/CmdVel", 1, &NodeClass::topicCallback_CmdVel, this);
            
            // implementation of service servers
            srvServer_Init = n.advertiseService("cob3/arm/Init", &NodeClass::srvCallback_Init, this);
            srvServer_Home = n.advertiseService("cob3/arm/Home", &NodeClass::srvCallback_Home, this);
            srvServer_Stop = n.advertiseService("cob3/arm/Stop", &NodeClass::srvCallback_Stop, this);
            srvServer_SetOperationMode = n.advertiseService("cob3/arm/SetOperationMode", &NodeClass::srvCallback_SetOperationMode, this);
            
            // implementation of service clients
            //--
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_CmdPos(const cob3_msgs::CmdPos::ConstPtr& msg)
        {
            std::string operationMode;
            n.getParam("cob3/arm/OperationMode", operationMode);
            if (operationMode == "position")
            {
                ROS_INFO("received new position command [%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f]", 
                     msg->cmdPos[0], msg->cmdPos[1], msg->cmdPos[2], msg->cmdPos[3], 
                     msg->cmdPos[4], msg->cmdPos[5], msg->cmdPos[6]);
                //TODO PowerCubeCtrl
                PCube->MoveJointSpaceSync(msg->cmdPos);
            }
            else
            {
                ROS_ERROR("received new position command, but node running with OperationMode [%s]", operationMode.c_str());
            }
        }

        void topicCallback_CmdVel(const cob3_msgs::CmdVel::ConstPtr& msg)
        {          
            std::string operationMode;
            n.getParam("cob3/arm/OperationMode", operationMode);
            if (operationMode == "velocity")
            {
                ROS_INFO("received new velocity command [%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f]", 
                     msg->cmdVel[0], msg->cmdVel[1], msg->cmdVel[2], msg->cmdVel[3], 
                     msg->cmdVel[4], msg->cmdVel[5], msg->cmdVel[6]);
                //TODO PowerCubeCtrl
                PCube->MoveVel(msg->cmdVel);                
            }
            else
            {
                ROS_ERROR("received new velocity command, but node running with OperationMode [%s]", operationMode.c_str());
            }
        }

        // service callback functions
        // function will be called when a service is querried
        bool srvCallback_Init(cob3_srvs::Init::Request &req,
                              cob3_srvs::Init::Response &res )
        {
        	ROS_INFO("Initializing arm");
          	
          	// init powercubes 
          	//TODO: make iniFilepath as an argument
          	//TODO: read iniFile into ros prarameters
            if (PCube->Init("include/iniFile.txt")) 
            {
            	ROS_INFO("Initializing succesfull");
            	res.success = 0; // 0 = true, else = false
            }
            else
            {
            	ROS_ERROR("Initializing arm not succesfull. error: %s", PCube->getErrorMessage().c_str());
            	res.success = 1; // 0 = true, else = false
            	res.errorMessage.data = PCube->getErrorMessage();
            }
            return true;
        }

        bool srvCallback_Home(cob3_srvs::Home::Request &req,
                              cob3_srvs::Home::Response &res )
        {
	        ROS_INFO("Homing arm");
        
            // homing arm
            if (PCube->doHoming())
            {
            	ROS_INFO("Homing arm succesfull");
            	res.success = 0; // 0 = true, else = false
            }
            else
            {
            	ROS_ERROR("Homing arm not succesfull. error: %s", PCube->getErrorMessage().c_str());
            	res.success = 1; // 0 = true, else = false
            	res.errorMessage.data = PCube->getErrorMessage();
            }
            return true;
        }

        bool srvCallback_Stop(cob3_srvs::Stop::Request &req,
                              cob3_srvs::Stop::Response &res )
        {
       	    ROS_INFO("Stopping arm");
        
            // stopping all arm movements
            if (PCube->Stop())
            {
            	ROS_INFO("Stopping arm succesfull");
            	res.success = 0; // 0 = true, else = false
            }
            else
            {
            	ROS_ERROR("Stopping arm not succesfull. error: %s", PCube->getErrorMessage().c_str());
            	res.success = 1; // 0 = true, else = false
            	res.errorMessage.data = PCube->getErrorMessage();
            }
            return true;
        }

        bool srvCallback_SetOperationMode(cob3_srvs::SetOperationMode::Request &req,
                                          cob3_srvs::SetOperationMode::Response &res )
        {
        	ROS_INFO("Set operation mode to [%s]", req.operationMode.data.c_str());
            n.setParam("cob3/arm/OperationMode", req.operationMode.data.c_str());
            res.success = 0; // 0 = true, else = false
            return true;
        }
                
        // other function declarations
        void publishJointState()
        {
            // create message
            int DOF = 7;
            std::vector<double> ActualPos;
            std::vector<double> ActualVel;
            ActualPos.resize(DOF);
            ActualVel.resize(DOF);
            //TODO
            //PCube->getConfig(ActualPos);
            //get velocities
            sensor_msgs::JointState msg;
            msg.set_position_size(DOF);
            msg.set_velocity_size(DOF);
            for (int i = 0; i<DOF; i++ )
            {
                msg.position[i] = ActualPos[i];
                msg.velocity[i] = ActualVel[i];
            }
                
            // publish message
            ROS_INFO("published ActualPos [%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f]", 
                     msg.position[0], msg.position[1], msg.position[2], msg.position[3], 
                     msg.position[4], msg.position[5], msg.position[6]);
            topicPub_JointState.publish(msg);
        }

        void publishActuatorState()
        {
            // create message
            cob3_msgs::ActuatorState msg;
            
            // publish message
            ROS_INFO("published ActuatorState");
            topicPub_ActuatorState.publish(msg);
        }
}; //NodeClass

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "cob3_driver_arm");
    
    // create nodeClass
    NodeClass nodeClass;
 
    // main loop
 	ros::Rate loop_rate(1); // Hz
    while(nodeClass.n.ok())
    {
        // publish JointState
        nodeClass.publishJointState();
        
        // publish ActuatorState
        nodeClass.publishActuatorState();
        
        // read parameter
        std::string operationMode;
        nodeClass.n.getParam("cob3/arm/OperationMode", operationMode);
        ROS_INFO("running with OperationMode [%s]", operationMode.c_str());

        // sleep and waiting for messages, callbacks 
        ros::spinOnce();
        loop_rate.sleep();
    }
    
//    ros::spin();

    return 0;
}

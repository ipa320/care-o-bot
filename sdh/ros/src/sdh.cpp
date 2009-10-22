//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <cob3_msgs/CmdPos.h>
#include <cob3_msgs/ActuatorState.h>
#include <sensor_msgs/JointState.h>

// ROS service includes
//--

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
                
        // declaration of topics to publish
        ros::Publisher topicPub_JointState;
        ros::Publisher topicPub_ActuatorState;
        
	    // declaration of topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_CmdPos;
        
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
            // implementation of topics to publish
            topicPub_JointState = n.advertise<sensor_msgs::JointState>("JointState", 1);
            topicPub_ActuatorState = n.advertise<cob3_msgs::ActuatorState>("ActuatorState", 1);
            
            // implementation of topics to subscribe
            topicSub_CmdPos = n.subscribe("CmdPos", 1, &NodeClass::topicCallback_CmdPos, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_CmdPos(const cob3_msgs::CmdPos::ConstPtr& msg)
        {
            ROS_INFO("Received new CmdPos [%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f]", msg->cmdPos[0], msg->cmdPos[1], msg->cmdPos[2], msg->cmdPos[3], msg->cmdPos[4], msg->cmdPos[5], msg->cmdPos[6]);
            
            //TODO: send msg data to hardware
        }

        // service callback functions
        // function will be called when a service is querried
        //--
        
        // other function declarations
        void updateCmdPos()
        {
            // send target pos to hardware
            //TODO
        }

        void updateJointState()
        {
            //get actual joint positions 
            //TODO: get from hardware

			//fill message
			sensor_msgs::JointState msg;
			msg.position.resize(7);
			//TODO: fill with hardware data
			msg.position[0] = 0;
			msg.position[1] = 0;
			msg.position[2] = 0;
			msg.position[3] = 0;
			msg.position[4] = 0;
			msg.position[5] = 0;
			msg.position[6] = 0;
			

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
 
 	ros::Rate loop_rate(5); // Hz
    while(nodeClass.n.ok())
    {
        // publish JointState
        nodeClass.updateJointState();

        // update target commands
        nodeClass.updateCmdPos();
    
        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
//    ros::spin();

    return 0;
}

//##################################
//#### function implementations ####
//--

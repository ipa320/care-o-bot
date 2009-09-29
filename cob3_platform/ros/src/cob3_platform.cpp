//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

// ROS service includes
#include <cob3_srvs/Init.h>
#include <cob3_srvs/Stop.h>

// external includes
#include <../include/PlatformHardware.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // topics to publish
        ros::Publisher topicPub_Pose2D;
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_CmdVel;
        
        // service servers
        ros::ServiceServer srvServer_Init;
        ros::ServiceServer srvServer_Home;
        ros::ServiceServer srvServer_Stop;
            
        // service clients
        //--
        
        // global variables
        //--

        // Constructor
        NodeClass()
        {
        	PlatformHardware* pltf;
        	pltf = new PlatformHardware();
        
	        // implementation of topics to publish
            topicPub_Pose2D = n.advertise<geometry_msgs::Pose2D>("cob3/platform/Pose2D", 1);
            
            // implementation of topics to subscribe
            topicSub_CmdVel = n.subscribe("cob3/platform/CmdVel", 1, &NodeClass::topicCallback_CmdVel, this);
            
            // implementation of service servers
            srvServer_Init = n.advertiseService("cob3/arm/Init", &NodeClass::srvCallback_Init, this);
            srvServer_Stop = n.advertiseService("cob3/arm/Stop", &NodeClass::srvCallback_Stop, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_CmdVel(const geometry_msgs::Twist::ConstPtr& msg)
        {
            ROS_INFO("received new velocity command [linX=%3.2f,linY=%3.2f,angZ=%3.2f]", 
                     msg->linear.x, msg->linear.y, msg->angular.z);
        }

        // service callback functions
        // function will be called when a service is querried
        bool srvCallback_Init(cob3_srvs::Init::Request &req,
                              cob3_srvs::Init::Response &res )
        {
            ROS_INFO("This is srvCallback_Init");
            res.success = 0; // 0 = true, else = false
            return true;
        }
        
        bool srvCallback_Stop(cob3_srvs::Stop::Request &req,
                              cob3_srvs::Stop::Response &res )
        {
            ROS_INFO("This is srvCallback_Stop");
            res.success = 0; // 0 = true, else = false
            return true;
        }
        
        // other function declarations
        void publishPose2D()
        {
            // create message
            geometry_msgs::Pose2D msg;
            //TODO fill message
            
            // publish message
            ROS_INFO("published Pose2D");
            topicPub_Pose2D.publish(msg);
        }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "cob3_driver_platform");
    
    NodeClass nodeClass;

    // main loop
 	ros::Rate loop_rate(1); // Hz 
    while(nodeClass.n.ok())
    {
        // publish Pose2D
        nodeClass.publishPose2D();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
//    ros::spin();

    return 0;
}

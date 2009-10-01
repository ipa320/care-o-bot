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
#include <cob3_srvs/Shutdown.h>

// external includes
#include <PlatformHardware.h>

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
        ros::ServiceServer srvServer_Shutdown;
            
        // service clients
        //--
        
        // global variables
        PlatformHardware* pltf;
        bool isInitialized;
        double velX,velY,angZ;

        // Constructor
        NodeClass()
        {        	
            isInitialized = false;
            velX = velY = angZ = 0;

        	// implementation of topics to publish
            topicPub_Pose2D = n.advertise<geometry_msgs::Pose2D>("cob3/platform/Pose2D", 1);
            
            // implementation of topics to subscribe
            topicSub_CmdVel = n.subscribe("cob3/platform/CmdVel", 1, &NodeClass::topicCallback_CmdVel, this);
            
            // implementation of service servers
            srvServer_Init = n.advertiseService("cob3/platform/Init", &NodeClass::srvCallback_Init, this);
            srvServer_Stop = n.advertiseService("cob3/platform/Stop", &NodeClass::srvCallback_Stop, this);
            srvServer_Shutdown = n.advertiseService("cob3/platform/Shutdown", &NodeClass::srvCallback_Shutdown, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
            pltf->setVelPltf(0, 0, 0, 0);
            pltf->shutdownPltf();
            isInitialized = false;
            delete pltf;
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_CmdVel(const geometry_msgs::Twist::ConstPtr& msg)
        {
            ROS_INFO("received new velocity command [linX=%3.5f,linY=%3.5f,angZ=%3.5f]", 
                     msg->linear.x, msg->linear.y, msg->angular.z);

            velX = msg->linear.x;
            velY = msg->linear.y;
            angZ = msg->angular.z;
        }

        // service callback functions
        // function will be called when a service is querried
        bool srvCallback_Init(cob3_srvs::Init::Request &req,
                              cob3_srvs::Init::Response &res )
        {
            ROS_INFO("This is srvCallback_Init");
            pltf = new PlatformHardware();
            pltf->initPltf();
            isInitialized = true;
            res.success = 0; // 0 = true, else = false
            return true;
        }
        
        bool srvCallback_Stop(cob3_srvs::Stop::Request &req,
                              cob3_srvs::Stop::Response &res )
        {
            ROS_INFO("This is srvCallback_Stop");
            velX = 0;
            velY = 0;
            angZ = 0;
            res.success = 0; // 0 = true, else = false
            return true;
        }

        bool srvCallback_Shutdown(cob3_srvs::Shutdown::Request &req,
                                  cob3_srvs::Shutdown::Response &res )
        {
            ROS_INFO("This is srvCallback_Shutdown");
            pltf->shutdownPltf();
            isInitialized = false;
            res.success = 0; // 0 = true, else = false
            return true;
        }
        
        // other function declarations
        void updateVelPltf()
        {
            // send vel if platform is initialized
            if(isInitialized == true)
            {
                ROS_INFO("update vel");
                pltf->setVelPltf(velX, velY, angZ, 0);
            }
        }

        void publishPose2D()
        {
            // create message
            geometry_msgs::Pose2D msg;
            //TODO fill message
            if(isInitialized == true)
            {
                // publish message
                ROS_INFO("published Pose2D");
            }
            
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
 	ros::Rate loop_rate(10); // Hz 
    while(nodeClass.n.ok())
    {
        // publish Pose2D
        nodeClass.publishPose2D();

        // update velocity of platform
        nodeClass.updateVelPltf();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
//    ros::spin();

    return 0;
}

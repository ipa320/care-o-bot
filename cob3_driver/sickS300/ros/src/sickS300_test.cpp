//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/LaserScan.h>

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
                
        // topics to publish
		//--
		        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_LaserScan;
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        //--

        // Constructor
        NodeClass()
        {
            topicSub_LaserScan = n.subscribe("LaserScan", 1, &NodeClass::topicCallback_LaserScan, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_LaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
            ROS_INFO("this is topicCallback_LaserScan");
            ROS_INFO("...received new scan. ranges[1]=%f",msg->ranges[1]);
        }

        // service callback functions
        // function will be called when a service is querried
        //--
        
        // other function declarations
		//--
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "sickS300_test");
    
    NodeClass nodeClass;
    
    ROS_INFO("...waiting for scans...");
 
    while(nodeClass.n.ok())
    {

        ros::spinOnce();
    }
    
//    ros::spin();

    return 0;
}

//##################################
//#### function implementations ####
//--

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

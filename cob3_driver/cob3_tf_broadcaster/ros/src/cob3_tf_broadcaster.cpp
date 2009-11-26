//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

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
		tf::TransformBroadcaster br;
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_Odometry;
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
		tf::Transform transform;

        // Constructor
        NodeClass()
        {
            topicSub_Odometry = n.subscribe("odometry", 10, &NodeClass::topicCallback_Odometry, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_Odometry(const nav_msgs::Odometry::ConstPtr& msg)
        {
			// publish tf for world --> base_footprint
			ROS_DEBUG("received new odometry message --> publishing tf");
			tf::poseMsgToTF(msg->pose.pose,transform);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_footprint"));
        }

        // service callback functions
        // function will be called when a service is querried
		//--
        
        // other function declarations
		void updateTf()
        {
            ROS_DEBUG("update static tf");

			// publish tf for base_footprint --> base_link
			transform.setOrigin(tf::Vector3(0.0, 0.0, 0.2));
			transform.setRotation(tf::Quaternion(0, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));

			// publish tf for base_link --> base_laser_front
			transform.setOrigin(tf::Vector3(0.3, 0.0, 0.2));
			transform.setRotation(tf::Quaternion(0, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser_front"));

			// publish tf for base_link --> base_laser_rear
			transform.setOrigin(tf::Vector3(-0.3, 0.0, 0.2));
			transform.setRotation(tf::Quaternion(0, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser_rear"));

			// publish tf for base_link --> base_hokuyo
			transform.setOrigin(tf::Vector3(0.2, 0.0, 0.2));
			transform.setRotation(tf::Quaternion(0, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_hokuyo"));

			// publish ...

        }

};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "cob3_tf_broadcaster");
    
    NodeClass nodeClass;

    // main loop
 	ros::Rate loop_rate(1); // Hz
	ROS_INFO("publishing tf with 1 Hz");
    while(nodeClass.n.ok())
    {
        // update static tf
        nodeClass.updateTf();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

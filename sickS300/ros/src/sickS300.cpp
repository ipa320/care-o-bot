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
        ros::Publisher topicPub_LaserScan;
        
	    // topics to subscribe, callback is called for new messages arriving
		//--
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        //--

        // Constructor
        NodeClass()
        {
            topicPub_LaserScan = n.advertise<sensor_msgs::LaserScan>("LaserScan", 1);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
		//--

        // service callback functions
        // function will be called when a service is querried
        //--
        
        // other function declarations
		void publishLaserScan()
        {
        	// fill message
        	int num_readings = 100; //TODO
			double laser_frequency = 40; //TODO
			
        	sensor_msgs::LaserScan laserScan;
			laserScan.header.stamp = ros::Time::now();
			laserScan.header.frame_id = "base_link";
			laserScan.angle_min = -1.57; //TODO
			laserScan.angle_max = 1.57; //TODO
			laserScan.angle_increment = 3.14 / num_readings; //TODO
			laserScan.time_increment = (1 / laser_frequency) / (num_readings); //TODO
			laserScan.range_min = 0.0; //TODO
			laserScan.range_max = 100.0; //TODO

   			laserScan.set_ranges_size(num_readings);
    		laserScan.set_intensities_size(num_readings);
			for(int i = 0; i < num_readings; ++i)
			{
			    laserScan.ranges[i] = (rand() % 100 + 1) / 100.0 + 1;
			    laserScan.intensities[i] = i;
			}
        
        	// publish message
        	ROS_INFO("published new LaserScan message");
            topicPub_LaserScan.publish(laserScan);
        }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "sickS300");
    
    NodeClass nodeClass;
 
	ros::Rate loop_rate(1); // Hz
    while(nodeClass.n.ok())
    {
    	// publish LaserScan
        nodeClass.publishLaserScan();

        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

//##################################
//#### function implementations ####
//--

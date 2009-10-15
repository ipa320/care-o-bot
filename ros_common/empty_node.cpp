/*-----------------------------------------------\
| This is a template for the cob-ros-pkg to have |
| a unique coding standard for developing nodes. |
|                                                |
| !!! IMPORTANT: change node_name !!!            |
|                                                |
| fmw 01.09.2009                                 |
\_______________________________________________*/

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
//--

// ROS service includes
//--

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
	ros::init(argc, argv, "node_name");

	// create a handle for this node, initialize node
	ros::NodeHandle n;
	
    // topics to publish
    //--
    
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    //--
    
    // external code
  	//--
  	
	// main loop
    ros::Rate loop_rate(10); // Hz
	while (n.ok())
	{
        // create message
        //--
        
        // publish message
        //--

        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

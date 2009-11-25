//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/PointCloud.h>

// ROS service includes
//--

// external includes
#include <include/AbstractRangeImagingSensor.h>

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
        ros::Subscriber topicSub_PointCloud;
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        //--

        // Constructor
        NodeClass()
        {
            //topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
            topicSub_PointCloud= n.subscribe("cob3/swissRanger/PointCloud", 1, &NodeClass::topicCallback_PointCloud, this);

			// uncomment to display image
			cvNamedWindow("PointCloud", CV_WINDOW_AUTOSIZE);
        }
        
        // Destructor
        ~NodeClass() 
        {
//			cvDestroyAllWindows();
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_PointCloud(const sensor_msgs::PointCloud::ConstPtr& msg)
        {
        	ROS_INFO("received pointCloud from swissRanger");
			// display PointCloud
			IplImage* image = cvCreateImage(cvSize(176, 144), IPL_DEPTH_32F, 3);
			for (unsigned int i=0; i<msg->points.size(); i++) 
			{
				((float*)image->imageData)[3*i+0] = (msg->points[i]).x;
				((float*)image->imageData)[3*i+1] = (msg->points[i]).y;
				((float*)image->imageData)[3*i+2] = (msg->points[i]).z;
			}

			IplImage* image_show = cvCreateImage(cvSize(176, 144), IPL_DEPTH_8U, 3);
			ipa_Utils::ConvertToShowImage(image, image_show, 3);

			// uncomment to display image
			cvShowImage("PointCloud", image_show);
			cvWaitKey(10);
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
    ros::init(argc, argv, "cob3_driver_swissRanger_displayPointCloud");
    
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
//--

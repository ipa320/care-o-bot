//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/Image.h>

// ROS service includes
//--

// external includes
#include <AbstractColorCamera.h>

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
        ros::Subscriber topicSub_Image;
        
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
            topicSub_Image= n.subscribe("cob3/colorCamera/left/Image", 1, &NodeClass::topicCallback_Image, this);

			// uncomment to display image
			cvNamedWindow("Image_left", CV_WINDOW_AUTOSIZE);
        }
        
        // Destructor
        ~NodeClass() 
        {
			cvDestroyAllWindows();
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_Image(const sensor_msgs::Image::ConstPtr& msg)
        {
        	ROS_INFO("received image from colorCamera_left");

			IplImage* image = cvCreateImage(cvSize(msg->width, msg->height), IPL_DEPTH_8U, 3);
			for (unsigned int i=0; i<msg->width*msg->height*3; i++) 
			{
				image->imageData[i] = (msg->data)[i];
			}

			double scaleFactor = 0.2;
			IplImage* image_show = cvCreateImage(cvSize(msg->width*scaleFactor, msg->height*scaleFactor), IPL_DEPTH_8U, 3);
			cvResize(image, image_show);

			// uncomment to display image
			cvShowImage("Image_left", image_show);
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
    ros::init(argc, argv, "cob3_driver_colorCamera_left_displayImage");
    
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

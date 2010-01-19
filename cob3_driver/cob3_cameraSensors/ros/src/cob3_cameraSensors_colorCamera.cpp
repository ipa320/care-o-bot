//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <cob3_msgs/SensorState.h>

// ROS service includes
#include <cob3_srvs/Init.h>
#include <cob3_srvs/Stop.h>
#include <cob3_srvs/GetCameraInfo.h>

// external includes
#include <AbstractColorCamera.h>
#include <GlobalDefines.h>

using namespace ipa_CameraSensors;

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

bool srvCallback_Init(cob3_srvs::Init::Request &req,
                      cob3_srvs::Init::Response &res )
{
    ROS_INFO("Initializing camera");
    res.success = 0; // 0 = true, else = false
    return true;
}

bool srvCallback_Stop(cob3_srvs::Stop::Request &req,
                      cob3_srvs::Stop::Response &res )
{
    ROS_INFO("Stopping camera");
    res.success = 0; // 0 = true, else = false
    return true;
}

bool srvCallback_GetCameraInfo(cob3_srvs::GetCameraInfo::Request &req,
                               cob3_srvs::GetCameraInfo::Response &res )
{
    ROS_INFO("get camera info");
    sensor_msgs::CameraInfo cameraInfo;
    //TODO: get real camera Info
    res.cameraInfo = cameraInfo;
    res.success = 0; // 0 = true, else = false
    return true;
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob3_driver_colorCamera_left");

	// create a handle for this node, initialize node
	ros::NodeHandle n;
	
    // topics to publish
    ros::Publisher topicPub_Image = n.advertise<sensor_msgs::Image>("cob3/colorCamera/left/Image", 1);
    ros::Publisher topicPub_SensorState = n.advertise<cob3_msgs::SensorState>("cob3/colorCamera/left/SensorState", 1);
    
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    ros::ServiceServer srvServer_Stop = n.advertiseService("cob3/colorCamera/left/Stop", srvCallback_Stop);
    ros::ServiceServer srvServer_Init = n.advertiseService("cob3/colorCamera/left/Init", srvCallback_Init);
    ros::ServiceServer srvServer_GetCameraInfo = n.advertiseService("cob3/colorCamera/left/GetCameraInfo", srvCallback_GetCameraInfo);
        
    // service clients
    //--
    
    // external code

	/// Camera index ranges from 0 (right) to 1 (left)
	int cameraIndex = 1;
	IplImage* image = 0;
	std::string directory = "../files/";

  	AbstractColorCamera* colorCamera = 0;
	colorCamera = ipa_CameraSensors::CreateColorCamera_AVTPikeCam();

	if (colorCamera->Init(directory, cameraIndex) & ipa_CameraSensors::RET_FAILED)
    {
            std::cerr << "ERROR - CameraDataViewerControlFlow::Init:" << std::endl;
            std::cerr << "\t ... Error while initializing color sensor 0.\n";
            colorCamera = 0;
    }

    if (colorCamera && (colorCamera->Open() & ipa_CameraSensors::RET_FAILED))
    {
            std::cerr << "ERROR - CameraDataViewerControlFlow::Init:" << std::endl;
            std::cerr << "\t ... Error while opening color sensor 0.\n";
            colorCamera = 0;
    }

	// display image
	//cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);

    // main loop
    ros::Rate loop_rate(3); // Hz
	while (n.ok())
	{
		if (image != 0)
		{
			cvReleaseImage(&image);
			image = 0;
		}

        if (colorCamera->GetColorImage2(&image) & ipa_Utils::RET_FAILED)
        {
                std::cerr << "ERROR - CameraDataViewerControlFlow::ShowSharedImage:" << std::endl;
                std::cerr << "\t ... Color image acquisition from camera 0 failed." << std::endl;
                return ipa_Utils::RET_FAILED;
        }

        // create message
        sensor_msgs::Image msg;
		fillImage(msg, "bgr8", image->height, image->width, image->widthStep, image->imageData);
		msg.header.stamp = ros::Time::now();    
  
        // publish message
        ROS_INFO("published image from colorCamera_left");
		
		// uncomment to display image
		//cvShowImage("Image", image);
		//cvWaitKey(10);

        topicPub_Image.publish(msg);

        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }

	colorCamera->Close();
	ipa_CameraSensors::ReleaseColorCamera(colorCamera);
    
    return 0;
}

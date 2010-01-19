//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/PointCloud.h>
#include <cob3_msgs/SensorState.h>

// ROS service includes
#include <cob3_srvs/Init.h>
#include <cob3_srvs/Stop.h>
#include <cob3_srvs/GetCameraInfo.h>

// external includes
#include <AbstractRangeImagingSensor.h>

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
    ROS_INFO("Initializing SwissRanger");
    res.success = 0; // 0 = true, else = false
    return true;
}

bool srvCallback_Stop(cob3_srvs::Stop::Request &req,
                      cob3_srvs::Stop::Response &res )
{
    ROS_INFO("Stopping SwissRanger");
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
	ros::init(argc, argv, "cob3_driver_swissRanger");

	// create a handle for this node, initialize node
	ros::NodeHandle n;
	
    // topics to publish
    ros::Publisher topicPub_PointCloud = n.advertise<sensor_msgs::PointCloud>("cob3/swissRanger/PointCloud", 1);
    ros::Publisher topicPub_SensorState = n.advertise<cob3_msgs::SensorState>("cob3/swissRanger/SensorState", 1);
    
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    ros::ServiceServer srvServer_Stop = n.advertiseService("cob3/swissRanger/Stop", srvCallback_Stop);
    ros::ServiceServer srvServer_Init = n.advertiseService("cob3/swissRanger/Init", srvCallback_Init);
    ros::ServiceServer srvServer_GetCameraInfo = n.advertiseService("cob3/swissRanger/GetCameraInfo", srvCallback_GetCameraInfo);
        
    // service clients
    //--
    
    // external code 	 	 		
	IplImage* image = 0;
	std::string directory = "../files/";

	AbstractRangeImagingSensor* rangeImagingSensor = 0;
	rangeImagingSensor = ipa_CameraSensors::CreateRangeImagingSensor_SR3000();

	if (rangeImagingSensor->Init(directory) & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - CameraDataViewerControlFlow::Init:" << std::endl;
		std::cerr << "\t ... Error while initializing range imaging sensor.\n";
		return ipa_Utils::RET_FAILED;
	}
	
	if (rangeImagingSensor->Open() & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - CameraDataViewerControlFlow::Init:" << std::endl;
		std::cerr << "\t ... Error while opening range imaging sensor.\n";
		return ipa_Utils::RET_FAILED;
	}

    // main loop
    ros::Rate loop_rate(5); // Hz
	while (n.ok())
	{
		if (image != 0)
		{
			cvReleaseImage(&image);
			image = 0;
		}

		if(rangeImagingSensor->AcquireImages2(0, 0, &image, false, false) & ipa_Utils::RET_FAILED)
		{	
			std::cerr << "ERROR - CameraDataViewerControlFlow::ShowSharedImage:" << std::endl;
			std::cerr << "\t ... Range image acquisition failed" << std::endl;
			return ipa_Utils::RET_FAILED;	
		}

        // create message
		int numPoints = image->width*image->height;
        sensor_msgs::PointCloud msg;
		msg.header.stamp = ros::Time::now(); 
        msg.points.resize(numPoints);
 		for (int i = 0; i < numPoints; i++)
		{
			//TODO: produces segmentation faults!
//			std::cout << "numPoint: " << i << "/" << numPoints << std::endl;
			msg.points[i].x = ((float*)image->imageData)[3*i + 0];
			msg.points[i].y = ((float*)image->imageData)[3*i + 1];
			msg.points[i].z = ((float*)image->imageData)[3*i + 2];
		}

        // publish message
        ROS_INFO("published pointCloud from swissRanger");

        topicPub_PointCloud.publish(msg);

        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }

	rangeImagingSensor->Close();
	ipa_CameraSensors::ReleaseRangeImagingSensor(rangeImagingSensor);
    
    return 0;
}

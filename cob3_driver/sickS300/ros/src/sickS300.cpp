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
#include <include/ScannerSickS300.h>

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
        std::string port;
		int baud;
		bool inverted; // TODO not used at the moment
		std::string frame_id;

        // Constructor
        NodeClass()
        {
            // initialize global variables
			n.param("port", port, std::string("/dev/ttyUSB0"));
			n.param("baud", baud, 500000);
			n.param("inverted", inverted, false);
			n.param("frame_id", frame_id, std::string("base_laser"));

        	// implementation of topics to publish
            topicPub_LaserScan = n.advertise<sensor_msgs::LaserScan>("scan", 1);

            // implementation of topics to subscribe
			//--
            
            // implementation of service servers
			//--
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
		void publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, std::vector<double> vdIntensAU)
        {
        	// fill message
        	int num_readings = vdDistM.size();
			double laser_frequency = 10; //TODO: read from Ini-file
			
			// create LaserScan Data-Container
        	sensor_msgs::LaserScan laserScan;

			// get time stamp for header
			laserScan.header.stamp = ros::Time::now();
            
			// set scan parameters
//			laserScan.header.frame_id = "base_laser_front"; // TODO read from parameter
			laserScan.header.frame_id = frame_id;
			laserScan.angle_min = vdAngRAD[0]; // first ScanAngle
			laserScan.angle_max = vdAngRAD[num_readings - 1]; // last ScanAngle
			laserScan.angle_increment = vdAngRAD[1] - vdAngRAD[0];
			laserScan.range_min = 0.0; //TODO read from ini-file/parameter-file
			laserScan.range_max = 100.0; //TODO read from ini-file/parameter-file

			laserScan.time_increment = (1 / laser_frequency) / (num_readings); //TODO read from ini-file/parameter-file

   			laserScan.set_ranges_size(num_readings);
    		laserScan.set_intensities_size(num_readings);
			for(int i = 0; i < num_readings; i++)
			{
			    laserScan.ranges[i] = vdDistM[i];
			    laserScan.intensities[i] = vdIntensAU[i];
			}
        
        	// publish message
            topicPub_LaserScan.publish(laserScan);
        	ROS_DEBUG("published new LaserScan message");
        }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "sickS300");
    
    NodeClass nodeClass;
	ScannerSickS300 SickS300;

	//char *pcPort = new char();
//	const char pcPort[] = "/dev/ttyUSB1"; //TODO replace with parameter port
	const char pcPort[] = "/dev/ttyUSB1";
//	int iBaudRate = 500000;
	int iBaudRate = nodeClass.baud;
	bool bOpenScan, bRecScan;
	std::vector<double> vdDistM, vdAngRAD, vdIntensAU;
 
	bOpenScan = SickS300.open(pcPort, iBaudRate);
	if(bOpenScan)
		ROS_INFO("Scanner opened successfully");
	else
	{
		ROS_ERROR("Scanner not available");
		return 0;
	}

	// main loop
	ros::Rate loop_rate(5); // Hz
    while(nodeClass.n.ok())
    {
		// read scan
		ROS_DEBUG("Read Scanner!");
		bRecScan = SickS300.getScan(vdDistM, vdAngRAD, vdIntensAU);
		ROS_DEBUG("Scanner read successfully");
    	// publish LaserScan
        if(bRecScan)
        {
		    ROS_DEBUG("publish LaserScan message!");
            nodeClass.publishLaserScan(vdDistM, vdAngRAD, vdIntensAU);
        }
        else
        {
		    ROS_WARN("No Scan available");
        }

        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

//##################################
//#### function implementations ####
//--

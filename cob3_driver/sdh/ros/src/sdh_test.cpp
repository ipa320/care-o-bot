/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob3_driver
 * ROS package name: sdh
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Jan 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
//#include <string>
//#include <sstream>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <cob3_msgs/CmdPos.h>

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
	ros::init(argc, argv, "sdh_test");

	// create a handle for this node, initialize node
	ros::NodeHandle n;

    // topics to publish
    ros::Publisher topicPub_CmdPos = n.advertise<cob3_msgs::CmdPos>("CmdPos", 1);
        
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
	//--
    
    // external code
	bool srv_querry = false;
	int srv_execute = 1;
	std::string srv_errorMessage = "no error";
    
    char c;
         
    // main loop
    while(n.ok())
    {
        // process user inputs
        std::cout << "Choose to test (send[C]ommand, [e]xit): ";
        
        std::cin >> c;

        switch(c)
        {
            case 'C':
            {
                // create message
                cob3_msgs::CmdPos msg;
                msg.cmdPos.resize(7);
                
                std::cout << "Choose preset target position ([0,1,2,3,4,5,6]): ";
                std::cin >> c;
                if (c == '0')
                {
                    msg.cmdPos[0] = 0;
                    msg.cmdPos[1] = 0;
                    msg.cmdPos[2] = 0;
                    msg.cmdPos[3] = 0;
                    msg.cmdPos[4] = 0;
                    msg.cmdPos[5] = 0;
                    msg.cmdPos[6] = 0;
                }
                else if (c == '1')
                {
                    msg.cmdPos[0] = 1;
                    msg.cmdPos[1] = 1;
                    msg.cmdPos[2] = 1;
                    msg.cmdPos[3] = 1;
                    msg.cmdPos[4] = 1;
                    msg.cmdPos[5] = 1;
                    msg.cmdPos[6] = 1;
                }
                else if (c == '2')
                {
                    msg.cmdPos[0] = 2;
                    msg.cmdPos[1] = 2;
                    msg.cmdPos[2] = 2;
                    msg.cmdPos[3] = 2;
                    msg.cmdPos[4] = 2;
                    msg.cmdPos[5] = 2;
                    msg.cmdPos[6] = 2;
                }
                else if (c == '3')
                {
                    msg.cmdPos[0] = 0;
                    msg.cmdPos[1] = 0;
                    msg.cmdPos[2] = 0;
                    msg.cmdPos[3] = 0;
                    msg.cmdPos[4] = 0;
                    msg.cmdPos[5] = 0;
                    msg.cmdPos[6] = 0;
                }
                else if (c == '4')
                {
                    msg.cmdPos[0] = 0;
                    msg.cmdPos[1] = 0;
                    msg.cmdPos[2] = 0;
                    msg.cmdPos[3] = 0;
                    msg.cmdPos[4] = 0;
                    msg.cmdPos[5] = 0;
                    msg.cmdPos[6] = 0;
                }
                else if (c == '5')
                {
                    msg.cmdPos[0] = 0;
                    msg.cmdPos[1] = 0;
                    msg.cmdPos[2] = 0;
                    msg.cmdPos[3] = 0;
                    msg.cmdPos[4] = 0;
                    msg.cmdPos[5] = 0;
                    msg.cmdPos[6] = 0;
                }
                else if (c == '6')
                {
                    msg.cmdPos[0] = 0;
                    msg.cmdPos[1] = 0;
                    msg.cmdPos[2] = 0;
                    msg.cmdPos[3] = 0;
                    msg.cmdPos[4] = 0;
                    msg.cmdPos[5] = 0;
                    msg.cmdPos[6] = 0;
                }
                else
                {
                    ROS_ERROR("invalid target");
                }
                
                topicPub_CmdPos.publish(msg);
                
                std::cout << "ende" << std::endl;
                srv_querry = true;
                srv_execute = 0;
            	srv_errorMessage = "no error";
                break;
            }
            
            case 'e':
            {
                ROS_INFO("exit. Shutting down node");
                std::cout << std::endl;
                return 0;
                break;
            }
            
            default:
            {
                std::cout << "ERROR: invalid input, try again..." << std::endl << std::endl;
            }
        } //switch
        
		if (!srv_querry)
		{
			ROS_ERROR("Failed to call service");
		}
		else
		{
			ROS_INFO("Service call succesfull");
			
			if (srv_execute != 0)
			{
				ROS_ERROR("Service execution failed, errorMessage: %s", srv_errorMessage.c_str());
			}
		}
		
	} //while

    return 0;
} //main

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
 * ROS package name: cob3_arm_ik
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * Date of creation: Feb 2010
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

#ifndef COB3_ARM_IK_NODE_H
#define COB3_ARM_IK_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <angles/angles.h>
#include <cob3_arm_ik_solver.h>
#include <tf_conversions/tf_kdl.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicTreeInfo.h>
#include <kinematics_msgs/GetCollisionFreePositionIK.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <boost/shared_ptr.hpp>

namespace cob3_arm_ik
{
  class COB3ArmIKNode
  {
    public:

    /** @class
     *  @brief ROS/KDL based interface for the inverse kinematics of the COB3 arm
     *  @author Sachin Chitta <sachinc@willowgarage.com>
     *
     *  This class provides a ROS/KDL based interface to the inverse kinematics of the COB3 arm. It inherits from the KDL::ChainIkSolverPos class
     *  but also exposes additional functionality to return the multiple solutions from an inverse kinematics computation. It uses an instance of
     *  a ros::NodeHandle to find the robot description. It can thus be used only if the robot description is available on a ROS param server.
     *
     *  To use this wrapper, you must have a roscore running with a robot description available from the ROS param server. 
     */
    COB3ArmIKNode();

    virtual ~COB3ArmIKNode(){};

    /** 
     *  @brief Specifies if the node is active or not
     *  @return True if the node is active, false otherwise.
     */
    bool isActive();

    protected:

    urdf::Model robot_model_;

    bool active_;

    int free_angle_;

    double search_discretization_;

    double cost_multiplier_;

    std::string ik_service_name_;

    std::string fk_service_name_;

    std::string ik_query_name_;

    ros::NodeHandle node_handle_, root_handle_;

    ros::ServiceServer ik_service_;

    ros::ServiceServer ik_collision_service_;

    ros::ServiceServer ik_service_with_cost_;

    ros::ServiceServer fk_service_;

    ros::ServiceServer ik_query_;

    ros::ServiceClient check_state_validity_client_;

    ros::ServiceClient get_state_cost_client_;

    boost::shared_ptr<cob3_arm_ik::COB3ArmIKSolver> cob3_arm_ik_solver_;

    tf::TransformListener tf_;

    std::string control_topic_name_;

    KDL::Frame pose_desired_;

    std::string root_name_;

    int dimension_;

    bool ikService(kinematics_msgs::GetPositionIK::Request &request, 
                   kinematics_msgs::GetPositionIK::Response &response);

    bool ikQuery(kinematics_msgs::GetKinematicTreeInfo::Request &request, 
                 kinematics_msgs::GetKinematicTreeInfo::Response &response);

    bool fkService(kinematics_msgs::GetPositionFK::Request &request, 
                   kinematics_msgs::GetPositionFK::Response &response);

    /*    bool ikServiceWithCollision(kinematics_msgs::GetCollisionFreePositionIK::Request &request, 
                                kinematics_msgs::GetCollisionFreePositionIK::Response &response);

    bool ikServiceMinimumCost(kinematics_msgs::GetCollisionFreePositionIK::Request &request, kinematics_msgs::GetCollisionFreePositionIK::Response &response);

    void collisionCheck(const KDL::JntArray &jnt_array, const std::vector<std::string> &joint_names, bool &check);

    void getCollisionCost(const KDL::JntArray &jnt_array, const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, bool &valid, double &cost);
    */
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;

    KDL::Chain kdl_chain_;

    kinematics_msgs::KinematicTreeInfo chain_info_;
  };
}

#endif

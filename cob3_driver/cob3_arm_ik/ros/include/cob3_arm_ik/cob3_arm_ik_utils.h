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

#ifndef COB3_ARM_IK_UTILS_H
#define COB3_ARM_IK_UTILS_H

#include <ros/ros.h>
#include <vector>
#include <angles/angles.h>
#include <Eigen/Array>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetCollisionFreePositionIK.h>
#include <kinematics_msgs/GetKinematicTreeInfo.h>


using namespace angles;

namespace cob3_arm_ik
{
  Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p);

  double computeEuclideanDistance(const std::vector<double> &array_1, 
                                  const KDL::JntArray &array_2);

  double distance(const urdf::Pose &transform);

  bool solveQuadratic(const double &a, 
                      const double &b, 
                      const double &c, 
                      double *x1, 
                      double *x2);

  Eigen::Matrix4f matrixInverse(const Eigen::Matrix4f &g);

  bool solveCosineEqn(const double &a, 
                      const double &b, 
                      const double &c, 
                      double &soln1, 
                      double &soln2);

  bool loadRobotModel(ros::NodeHandle node_handle, 
                      urdf::Model &robot_model, 
                      std::string &root_name, 
                      std::string &tip_name, 
                      std::string &xml_string);

  bool getKDLChain(const std::string &xml_string, 
                   const std::string &root_name, 
                   const std::string &tip_name, 
                   KDL::Chain &kdl_chain);

  bool getKDLTree(const std::string &xml_string, 
                   const std::string &root_name, 
                   const std::string &tip_name, 
                   KDL::Tree &kdl_chain);

  bool checkJointNames(const std::vector<std::string> &joint_names, 
                       const kinematics_msgs::KinematicTreeInfo &chain_info);

  bool checkLinkNames(const std::vector<std::string> &link_names,
                      const kinematics_msgs::KinematicTreeInfo &chain_info);

  bool checkLinkName(const std::string &link_name, 
                     const kinematics_msgs::KinematicTreeInfo &chain_info);
 
  bool checkRobotState(motion_planning_msgs::RobotState &robot_state,
                       const kinematics_msgs::KinematicTreeInfo &chain_info);

  bool checkFKService(kinematics_msgs::GetPositionFK::Request &request, 
                      kinematics_msgs::GetPositionFK::Response &response, 
                      const kinematics_msgs::KinematicTreeInfo &chain_info);
 
  bool checkIKService(kinematics_msgs::GetPositionIK::Request &request, 
                      kinematics_msgs::GetPositionIK::Response &response,
                      const kinematics_msgs::KinematicTreeInfo &chain_info);
 
  bool checkCollisionFreeIKService(kinematics_msgs::GetCollisionFreePositionIK::Request &request, 
                                   kinematics_msgs::GetCollisionFreePositionIK::Response &response,
                                   const kinematics_msgs::KinematicTreeInfo &chain_info);

  int getJointIndex(const std::string &name,
                    const kinematics_msgs::KinematicTreeInfo &chain_info);

  bool convertPoseToRootFrame(const geometry_msgs::PoseStamped &pose_msg, 
                              KDL::Frame &pose_kdl, 
                              const std::string &root_frame, 
                              const tf::TransformListener &tf);

  bool convertPoseToRootFrame(const geometry_msgs::PoseStamped &pose_msg, 
                              geometry_msgs::PoseStamped &pose_msg_out, 
                              const std::string &root_frame, 
                              const tf::TransformListener &tf);

  int getKDLSegmentIndex(const KDL::Chain &chain, 
                         const std::string &name);


}

#endif// COB3_ARM_IK_UTILS_H

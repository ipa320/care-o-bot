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

#include <cob3_arm_ik/cob3_arm_ik_node.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include "ros/ros.h"
#include <algorithm>
#include <numeric>

#include <planning_environment_msgs/GetStateCost.h>
#include <planning_environment_msgs/GetStateValidity.h>
#include <planning_environment_msgs/GetJointTrajectoryValidity.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace cob3_arm_ik {

/// the string used internally to access control starting service; this should be remaped in the launch file
//  static const std::string COLLISION_CHECK_SERVICE = "get_state_validity";
//  static const std::string GET_STATE_COST_SERVICE = "get_state_cost";

//  static const std::string IK_WITH_COLLISION_SERVICE = "get_collision_free_ik";
//  static const std::string IK_WITH_COST_SERVICE = "get_minimum_cost_ik";
  static const std::string IK_SERVICE = "get_ik";
  static const std::string FK_SERVICE = "get_fk";
  static const std::string QUERY_SERVICE = "get_kinematic_tree_info";

  COB3ArmIKNode::COB3ArmIKNode():  node_handle_("~"),dimension_(7)
  {
    urdf::Model robot_model;
    std::string tip_name, xml_string;

    while(!loadRobotModel(node_handle_,robot_model,root_name_,tip_name,xml_string) && node_handle_.ok())
    {
      ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
      ros::Duration(0.5).sleep();
    }

    ROS_INFO("Loading KDL Tree");
    if(!getKDLChain(xml_string,root_name_,tip_name,kdl_chain_))
    {
      active_ = false;
      ROS_ERROR("Could not load kdl tree");
    }

    node_handle_.param<double>("cost_multiplier",cost_multiplier_,1.0e8);
    ROS_INFO("Advertising services");
    fk_service_ = node_handle_.advertiseService(FK_SERVICE,&COB3ArmIKNode::fkService,this);
    ik_service_ = node_handle_.advertiseService(IK_SERVICE,&COB3ArmIKNode::ikService,this);
    ik_query_ = node_handle_.advertiseService(QUERY_SERVICE,&COB3ArmIKNode::ikQuery,this);
    //    ik_collision_service_ = root_handle_.advertiseService(IK_WITH_COLLISION_SERVICE,&COB3ArmIKNode::ikServiceWithCollision,this);
    //    ik_service_with_cost_ = root_handle_.advertiseService(IK_WITH_COST_SERVICE,&COB3ArmIKNode::ikServiceMinimumCost,this);

    //    check_state_validity_client_ = root_handle_.serviceClient<planning_environment_msgs::GetStateValidity>(COLLISION_CHECK_SERVICE);
    //    get_state_cost_client_ = root_handle_.serviceClient<planning_environment_msgs::GetStateCost>(GET_STATE_COST_SERVICE);
    
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    node_handle_.param<int>("free_angle",free_angle_,2);
    node_handle_.param<double>("search_discretization",search_discretization_,0.01);
    cob3_arm_ik_solver_.reset(new cob3_arm_ik::COB3ArmIKSolver(robot_model,root_name_,tip_name, search_discretization_,free_angle_));
    if(!cob3_arm_ik_solver_->active_)
    {
      ROS_ERROR("Could not load ik");
      active_ = false;
    }
    else
    {
      active_ = true;
      cob3_arm_ik_solver_->getChainInfo(chain_info_);
      for(unsigned int i=0; i < chain_info_.joint_names.size(); i++)
      {
        ROS_INFO("cob3_arm_ik:: joint name: %s",chain_info_.joint_names[i].c_str());
      }
      for(unsigned int i=0; i < chain_info_.link_names.size(); i++)
      {
        ROS_INFO("cob3_arm_ik:: link name: %s",chain_info_.link_names[i].c_str());
      }
      ROS_INFO("cob3_arm_ik:: active");
    }
  }

  bool COB3ArmIKNode::isActive()
  {
    if(active_)
      return true;
    return false;
  }

  bool COB3ArmIKNode::ikService(kinematics_msgs::GetPositionIK::Request &request, 
                               kinematics_msgs::GetPositionIK::Response &response)
  {
    if(!active_)
    {
      ROS_ERROR("IK service not active");
      return true;
    }

    if(!checkIKService(request,response,chain_info_))
      return true;

    geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
    if(!convertPoseToRootFrame(pose_msg_in,pose_desired_,root_name_,tf_))
    {
      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      int tmp_index = getJointIndex(request.ik_request.ik_seed_state.joint_state.name[i],chain_info_);
      if(tmp_index >=0)
      {
        jnt_pos_in(tmp_index) = request.ik_request.ik_seed_state.joint_state.position[i];
      }
      else
      {
        ROS_ERROR("i: %d, No joint index for %s",i,request.ik_request.ik_seed_state.joint_state.name[i].c_str());
      }
    }

    bool ik_valid = (cob3_arm_ik_solver_->CartToJntSearch(jnt_pos_in,pose_desired_,jnt_pos_out,request.timeout.toSec()) >= 0);

    if(ik_valid)
    {
      response.solution.joint_state.name = chain_info_.joint_names;
      response.solution.joint_state.position.resize(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        response.solution.joint_state.position[i] = jnt_pos_out(i);
        ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,jnt_pos_out(i));
      }
      response.error_code.val = response.error_code.SUCCESS;
      return true;
    }
    else
    {
      response.error_code.val = response.error_code.NO_IK_SOLUTION;
      ROS_DEBUG("An IK solution could not be found");   
      return true;
    }
  }
/*
  bool COB3ArmIKNode::ikServiceWithCollision(kinematics_msgs::GetCollisionFreePositionIK::Request &request, kinematics_msgs::GetCollisionFreePositionIK::Response &response)
  {
    if(!active_)
      return false;

    if(!checkCollisionFreeIKService(request,response,chain_info_))
      return false;

    if(!ros::service::exists(COLLISION_CHECK_SERVICE,true))
    {
      ROS_ERROR("Could not find collision checking service: %s",COLLISION_CHECK_SERVICE.c_str());
      response.error_code.val = response.error_code.COLLISION_CHECKING_UNAVAILABLE;
      return false;
    }

    geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
    if(!convertPoseToRootFrame(pose_msg_in,pose_desired_,root_name_,tf_))
    {
      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
      return false;
    }
    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      int tmp_index = getJointIndex(request.ik_request.ik_seed_state.joint_state.name[i],chain_info_);
      if(tmp_index >=0)
        jnt_pos_in(tmp_index) = request.ik_request.ik_seed_state.joint_state.position[i];
    }

    bool ik_valid = (cob3_arm_ik_solver_->CartToJntSearchWithCollision(jnt_pos_in,
                                                                      pose_desired_,
                                                                      jnt_pos_out,
                                                                      request.timeout.toSec(),chain_info_.joint_names,
                                                                      boost::bind(&COB3ArmIKNode::collisionCheck, this, _1, _2, _3)) >= 0);
    if(ik_valid)
    {
      response.solution.joint_state.name = chain_info_.joint_names;
      response.solution.joint_state.position.resize(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        response.solution.joint_state.position[i] = jnt_pos_out(i);
        ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,jnt_pos_out(i));
      }
      response.error_code.val = response.error_code.SUCCESS;
      return true;
    }
    else
    {
      ROS_DEBUG("An IK solution could not be found");   
      response.error_code.val = response.error_code.NO_IK_SOLUTION;
      return false;
    }
  }
*/

/*  bool COB3ArmIKNode::ikServiceMinimumCost(kinematics_msgs::GetCollisionFreePositionIK::Request &request, kinematics_msgs::GetCollisionFreePositionIK::Response &response)
  {
    if(!active_)
      return false;

    if(!checkCollisionFreeIKService(request,response,chain_info_))
      return false;

    if(!ros::service::exists(GET_STATE_COST_SERVICE,true))
    {
      ROS_ERROR("Could not find collision cost service: %s",GET_STATE_COST_SERVICE.c_str());
      response.error_code.val = response.error_code.COLLISION_CHECKING_UNAVAILABLE;
      return false;
    }
    geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
    if(!convertPoseToRootFrame(pose_msg_in,pose_desired_,root_name_,tf_))
    {
      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
      return false;
    }

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      int tmp_index = getJointIndex(request.ik_request.ik_seed_state.joint_state.name[i],chain_info_);
      if(tmp_index >=0)
        jnt_pos_in(tmp_index) = request.ik_request.ik_seed_state.joint_state.position[i];
    }

    bool ik_valid = (cob3_arm_ik_solver_->CartToJntSearchMinimumCost(jnt_pos_in,
                                                                    pose_desired_,
                                                                    jnt_pos_out,
                                                                    10,
                                                                    chain_info_.joint_names,
                                                                    chain_info_.link_names,
                                                                    boost::bind(&COB3ArmIKNode::getCollisionCost, this, _1, _2, _3, _4, _5)) >= 0);


    if(ik_valid)
    {
      response.solution.joint_state.name = chain_info_.joint_names;
      response.solution.joint_state.position.resize(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        response.solution.joint_state.position[i] = jnt_pos_out(i);
        ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,jnt_pos_out(i));
      }
      response.error_code.val = response.error_code.SUCCESS;
      return true;
    }
    else
    {
      ROS_DEBUG("An IK solution could not be found");   
      response.error_code.val = response.error_code.NO_IK_SOLUTION;
      return false;
    }
  }

  void COB3ArmIKNode::collisionCheck(const KDL::JntArray &jnt_array, const std::vector<string> &joint_names, bool &check)
  {
    planning_environment_msgs::GetStateValidity::Request req;
    planning_environment_msgs::GetStateValidity::Response res;
    req.robot_state.joint_state.header.stamp = ros::Time::now();
    req.robot_state.joint_state.position.resize(7);
    req.robot_state.joint_state.name = joint_names;
    for(int i = 0; i < 7; i++)
    {
      req.robot_state.joint_state.position[i] = jnt_array(i);
    }

    req.flag = planning_environment_msgs::GetStateValidity::Request::COLLISION_TEST;

    if(check_state_validity_client_.call(req,res))
    {
      ROS_DEBUG("Service call to check state validity succeeded");
      check = res.error_code.val;
      if(check == res.error_code.SUCCESS)
        ROS_INFO("IK succeeded");
      return;
    }
    else
    {
      check = false;
      ROS_ERROR("Service call to check state validity failed");
      return;
    }
  }

  void COB3ArmIKNode::getCollisionCost(const KDL::JntArray &jnt_array, 
                                      const std::vector<std::string> &joint_names, 
                                      const std::vector<std::string> &link_names, 
                                      bool &valid, 
                                      double &cost)
  {
    planning_environment_msgs::GetStateCost::Request req;
    planning_environment_msgs::GetStateCost::Response res;
    req.robot_state.joint_state.header.stamp = ros::Time::now();
    req.robot_state.joint_state.position.resize(7);
    req.robot_state.joint_state.name = joint_names;
    for(int i = 0; i < 7; i++)
    {
      req.robot_state.joint_state.position[i] = jnt_array(i);
    }
    req.link_names = link_names;
    if(get_state_cost_client_.call(req,res))
    {
      ROS_DEBUG("Service call to find cost succeeded");
      std::vector<double> tmp;
      tmp = res.costs;
      for(unsigned int i =0; i < tmp.size(); i++)
        tmp[i] *= cost_multiplier_;
      cost = std::accumulate(tmp.begin(),tmp.end(),0);
      valid = res.valid;
      return;
    }
    else
    {
      valid = false;
      ROS_ERROR("Service call to check plan validity failed");
      return;
    }
  }
*/
  bool COB3ArmIKNode::ikQuery(kinematics_msgs::GetKinematicTreeInfo::Request &request, 
                             kinematics_msgs::GetKinematicTreeInfo::Response &response)
  {
    if(!active_)
    {
      ROS_ERROR("IK node not active");
      return true;
    }
    cob3_arm_ik_solver_->getChainInfo(response.kinematic_tree_info);
    return true;
  }

  bool COB3ArmIKNode::fkService(kinematics_msgs::GetPositionFK::Request &request, 
                               kinematics_msgs::GetPositionFK::Response &response)
  {
    if(!active_)
    {
      ROS_ERROR("FK service not active");
      return true;
    }

    if(!checkFKService(request,response,chain_info_))
      return true;

    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      int tmp_index = getJointIndex(request.robot_state.joint_state.name[i],chain_info_);
      if(tmp_index >=0)
        jnt_pos_in(tmp_index) = request.robot_state.joint_state.position[i];
    }

    response.pose_stamped.resize(request.fk_link_names.size());
    response.fk_link_names.resize(request.fk_link_names.size());

    bool valid = true;
    for(unsigned int i=0; i < request.fk_link_names.size(); i++)
    {
      ROS_DEBUG("End effector index: %d",cob3_arm_ik::getKDLSegmentIndex(kdl_chain_,request.fk_link_names[i]));
      ROS_DEBUG("Chain indices: %d",kdl_chain_.getNrOfSegments());
      if(jnt_to_pose_solver_->JntToCart(jnt_pos_in,p_out,cob3_arm_ik::getKDLSegmentIndex(kdl_chain_,request.fk_link_names[i])) >=0)
      {
        tf_pose.frame_id_ = root_name_;
        tf_pose.stamp_ = ros::Time();
        tf::PoseKDLToTF(p_out,tf_pose);
        try{
          tf_.transformPose(request.header.frame_id,tf_pose,tf_pose);
        }
        catch(...)
        {
          ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
          response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
          return false;
        }
        tf::poseStampedTFToMsg(tf_pose,pose);
        response.pose_stamped[i] = pose;
        response.fk_link_names[i] = request.fk_link_names[i];
        response.error_code.val = response.error_code.SUCCESS;
      }
      else
      {
        ROS_ERROR("Could not compute FK for %s",request.fk_link_names[i].c_str());
        response.error_code.val = response.error_code.NO_FK_SOLUTION;
        valid = false;
      }
    }
    return true;
  }
} // namespace

/*int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob3_arm_ik_node");
  cob3_arm_ik::COB3ArmIKNode cob3_arm_ik_node;
  ros::spin();
  return(0);
}
*/

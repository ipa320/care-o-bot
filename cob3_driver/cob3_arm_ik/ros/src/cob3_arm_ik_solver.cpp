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

#include <cob3_arm_ik_solver.h>

using namespace Eigen;
using namespace cob3_arm_ik;

COB3ArmIKSolver::COB3ArmIKSolver(urdf::Model robot_model, 
                               std::string root_frame_name,
                               std::string tip_frame_name,
                               double search_discretization_angle, 
                               int free_angle):ChainIkSolverPos()
{
  search_discretization_angle_ = search_discretization_angle;
  free_angle_ = free_angle;
  if(!cob3_arm_ik_.init(robot_model,root_frame_name,tip_frame_name))
    active_ = false;
  else
    active_ = true;
}

void COB3ArmIKSolver::getChainInfo(kinematics_msgs::KinematicTreeInfo &response)
{
  cob3_arm_ik_.getChainInfo(response);
}

int COB3ArmIKSolver::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  if(free_angle_ == 0)
  {
    ROS_DEBUG("Solving with %f",q_init(0)); 
    cob3_arm_ik_.computeIKShoulderPan(b,q_init(0));
  }
  else
  {
    cob3_arm_ik_.computeIKShoulderRoll(b,q_init(2));
  }
  
  if(cob3_arm_ik_.solution_ik_.empty())
    return -1;

  double min_distance = 1e6;
  int min_index = -1;

  for(int i=0; i< (int) cob3_arm_ik_.solution_ik_.size(); i++)
  {     
    ROS_DEBUG("Solution : %d",(int)cob3_arm_ik_.solution_ik_.size());

    for(int j=0; j < (int)cob3_arm_ik_.solution_ik_[i].size(); j++)
    {   
      ROS_DEBUG("%d: %f",j,cob3_arm_ik_.solution_ik_[i][j]);
    }
    ROS_DEBUG(" ");
    ROS_DEBUG(" ");

    double tmp_distance = computeEuclideanDistance(cob3_arm_ik_.solution_ik_[i],q_init);
    if(tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min_index = i;
    }
  }

  if(min_index > -1)
  {
    q_out.resize((int)cob3_arm_ik_.solution_ik_[min_index].size());
    for(int i=0; i < (int)cob3_arm_ik_.solution_ik_[min_index].size(); i++)
    {   
      q_out(i) = cob3_arm_ik_.solution_ik_[min_index][i];
    }
    return 1;
  }
  else
    return -1;
}

int COB3ArmIKSolver::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  KDL::JntArray q;

  if(free_angle_ == 0)
  {
    cob3_arm_ik_.computeIKShoulderPan(b,q_init(0));
  }
  else
  {
    cob3_arm_ik_.computeIKShoulderRoll(b,q_init(2));
  }
  
  if(cob3_arm_ik_.solution_ik_.empty())
    return -1;

  q.resize(7);
  q_out.clear();
  for(int i=0; i< (int) cob3_arm_ik_.solution_ik_.size(); i++)
  {     
    for(int j=0; j < 7; j++)
    {   
      q(j) = cob3_arm_ik_.solution_ik_[i][j];
    }
    q_out.push_back(q);
  }
  return 1;
}

bool COB3ArmIKSolver::getCount(int &count, int max_count, int min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {   
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}

int COB3ArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out, const double &timeout)
{
  KDL::JntArray q_init = q_in;
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((cob3_arm_ik_.chain_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-cob3_arm_ik_.chain_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,cob3_arm_ik_.chain_info_.limits[free_angle_].max_position,cob3_arm_ik_.chain_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  return -1;
}

int COB3ArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, KDL::JntArray &q_out, const double &timeout)
{
  KDL::JntArray q_init = q_in;
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((cob3_arm_ik_.chain_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-cob3_arm_ik_.chain_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,cob3_arm_ik_.chain_info_.limits[free_angle_].max_position,cob3_arm_ik_.chain_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  return -1;
}

int COB3ArmIKSolver::CartToJntSearchWithCollision(const KDL::JntArray& q_in, 
                                                 const KDL::Frame& p_in, 
                                                 KDL::JntArray &q_out, 
                                                 const double &timeout, 
                                                 const std::vector<std::string> &joint_names, 
                                                 const boost::function<void(const KDL::JntArray&, const std::vector<std::string> &joint_names, bool&)> &callback)
{
  KDL::JntArray q_init = q_in;
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((cob3_arm_ik_.chain_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-cob3_arm_ik_.chain_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,cob3_arm_ik_.chain_info_.limits[free_angle_].max_position,cob3_arm_ik_.chain_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
    {
      bool valid = false;
      callback(q_out,joint_names,valid);
      if(valid)
        return 1;
    }
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  return -1;
}

int COB3ArmIKSolver::CartToJntSearchMinimumCost(const KDL::JntArray& q_in, 
                                               const KDL::Frame& p_in, 
                                               KDL::JntArray &q_out, 
                                               const double &timeout, 
                                               const std::vector<std::string> &joint_names, 
                                               const std::vector<std::string> &link_names, 
                                               const boost::function<void(const KDL::JntArray&, const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, bool&, double&)> &callback)
{
  std::vector<cost_pair> cost_angle_pair;
  KDL::JntArray q_init = q_in;
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((cob3_arm_ik_.chain_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-cob3_arm_ik_.chain_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,cob3_arm_ik_.chain_info_.limits[free_angle_].max_position,cob3_arm_ik_.chain_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
    {
      if(CartToJnt(q_init,p_in,q_out) > 0)
        {
          bool valid = false;
          double cost = 0.0;
          callback(q_out,joint_names,link_names,valid,cost);
          cost_angle_pair.push_back(cost_pair(cost,q_out(free_angle_)));
	  ROS_DEBUG("Solution found: Cost: %f, Free angle: %f",cost,q_out(free_angle_));
        }
      if(!getCount(count,num_positive_increments,-num_negative_increments))
        {
          if(cost_angle_pair.empty())
            {
              ROS_WARN("No ik solutions found");
              return false;
            }
          std::sort(cost_angle_pair.begin(),cost_angle_pair.end());
          double min_angle = cost_angle_pair[0].second;
          q_init(free_angle_) = angles::normalize_angle(min_angle);
          if(CartToJnt(q_init,p_in,q_out) > 0)
            {            
              return 1;
            }
          return -1;
        }
      q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
      ROS_DEBUG("%d, %f",count,q_init(free_angle_));
      loop_time = (ros::Time::now()-start_time).toSec();
    }
  return -1;
}

int COB3ArmIKSolver::CartToJntSearchWithCallbacks(const kinematics_msgs::PositionIKRequest &ik_request, 
                                                 const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                                                 const motion_planning_msgs::RobotState &robot_state,
                                                 const double &timeout,
                                                 KDL::JntArray &q_out, 
                                                 kinematics_msgs::KinematicsErrorCode &error_code,
                                                 const boost::function<void(const kinematics_msgs::PositionIKRequest&, 
                                                                            const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                                                                            const motion_planning_msgs::RobotState &, kinematics_msgs::KinematicsErrorCode &error_code)> &desired_pose_callback,
                                                 const boost::function<void(const KDL::JntArray&, 
                                                                            const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                                                                            const motion_planning_msgs::RobotState &, kinematics_msgs::KinematicsErrorCode &error_code)> &solution_callback)
{
  geometry_msgs::PoseStamped pose_stamped_in = ik_request.pose_stamped;
  KDL::JntArray q_init;
  q_init.resize(cob3_arm_ik_.chain_info_.joint_names.size());
 
  for(unsigned int i=0; i < cob3_arm_ik_.chain_info_.joint_names.size(); i++)
  {
    int tmp_index = getJointIndex(ik_request.ik_seed_state.joint_state.name[i],cob3_arm_ik_.chain_info_);
    if(tmp_index >=0)
      q_init(tmp_index) = ik_request.ik_seed_state.joint_state.position[i];
  }

  KDL::Frame p_in;
  tf::PoseMsgToKDL(pose_stamped_in.pose, p_in);
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((cob3_arm_ik_.chain_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-cob3_arm_ik_.chain_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,cob3_arm_ik_.chain_info_.limits[free_angle_].max_position,cob3_arm_ik_.chain_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);

  if(!desired_pose_callback.empty())
    desired_pose_callback(ik_request,collision_operations,robot_state,error_code);
  if(!(error_code.val == error_code.SUCCESS))
  {
    return -1;
  }
  bool callback_check = true;
  if(solution_callback.empty())
    callback_check = false;

  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
    {
      if(callback_check)
      {
        solution_callback(q_out,collision_operations,robot_state,error_code);
        if(error_code.val == error_code.SUCCESS)
        {
          return 1;
        }
      }
      else
      {
        error_code.val = error_code.SUCCESS;
        return 1;
      }
    }
    if(!getCount(count,num_positive_increments,-num_negative_increments))
    {
      error_code.val = error_code.NO_IK_SOLUTION;
      return -1;
    }
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    //ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time > timeout)
    error_code.val = error_code.TIMED_OUT;
  else
    error_code.val = error_code.NO_IK_SOLUTION;

  return -1;
}


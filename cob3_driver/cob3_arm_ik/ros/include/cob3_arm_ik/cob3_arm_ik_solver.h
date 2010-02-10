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

#ifndef COB3_ARM_IK_SOLVER_H
#define COB3_ARM_IK_SOLVER_H

#include <urdf/model.h>
#include <Eigen/Array>
#include <kdl/chainiksolver.hpp>
#include <cob3_arm_ik.h>
#include <cob3_arm_ik_utils.h>
#include <kinematics_msgs/GetKinematicTreeInfo.h>
#include <kinematics_msgs/PositionIKRequest.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_kdl.h>
#include <motion_planning_msgs/RobotState.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>
#include <kinematics_msgs/KinematicsErrorCode.h>

namespace cob3_arm_ik
{

  typedef std::pair<double, double> cost_pair;

  class COB3ArmIKSolver : public KDL::ChainIkSolverPos
  {
    public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /** @class
     *  @brief ROS/KDL based interface for the inverse kinematics of the COB3 arm
     *  @author Sachin Chitta <sachinc@willowgarage.com>
     *
     *  This class provides a ROS/KDL based interface to the inverse kinematics of the COB3 arm. It inherits from the KDL::ChainIkSolverPos class
     *  but also exposes additional functionality to return the multiple solutions from an inverse kinematics computation. It uses an instance of
     *  a ros::Node to find the robot description. It can thus be used only if the robot description is available on a ROS param server.
     *
     *  To use this wrapper, you must have a roscore running with a robot description available from the ROS param server. 
     */
    COB3ArmIKSolver(urdf::Model robot_model, 
                   std::string root_frame_name,
                   std::string tip_frame_name,
                   double search_discretization_angle, 
                   int free_angle);

    ~COB3ArmIKSolver(){};

    /** 
     * @brief A pointer to the inverse kinematics solver 
     */ 
    COB3ArmIK cob3_arm_ik_;

    /**
     * @brief Indicates whether the solver has been successfully initialized
     */
    bool active_;

    /**
     * @brief The KDL solver interface that is required to be implemented. NOTE: This method only returns a solution
     * if it exists for the free parameter value passed in. To search for a solution in the entire workspace use the CartToJntSearch 
     * method detailed below.
     *
     * @return < 0 if no solution is found
     * @param q_init The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(cob3_ik_->free_angle_) as 
     * as an input to the inverse kinematics. cob3_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A single inverse kinematic solution (if it exists).  
     */
    int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out);

    /**
     * @brief An extension of the KDL solver interface to return all solutions found. NOTE: This method only returns a solution 
     * if it exists for the free parameter value passed in. To search for a solution in the entire workspace use the CartToJntSearch 
     * method detailed below.
     *
     * @return < 0 if no solution is found
     * @param q_init The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(cob3_ik_->free_angle_) as 
     * as an input to the inverse kinematics. cob3_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     */
    int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out);
  
     /**
     * @brief This method searches for and returns the first set of solutions it finds. 
     *
     * @return < 0 if no solution is found
     * @param q_in The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(cob3_ik_->free_angle_) as 
     * as an input to the inverse kinematics. cob3_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     * @param timeout The amount of time (in seconds) to spend looking for a solution.
     */
    int CartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out, const double &timeout);

     /**
     * @brief This method searches for and returns the closest solution to the initial guess in the first set of solutions it finds. 
     *
     * @return < 0 if no solution is found
     * @param q_in The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(cob3_ik_->free_angle_) as 
     * as an input to the inverse kinematics. cob3_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     * @param timeout The amount of time (in seconds) to spend looking for a solution.
     */
    int CartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, KDL::JntArray &q_out, const double &timeout);

     /**
     * @brief This method searches for and returns the first collision free solution it finds.
     *
     * @return < 0 if no solution is found
     * @param q_in The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(cob3_ik_->free_angle_) as 
     * as an input to the inverse kinematics. cob3_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     * @param timeout The amount of time (in seconds) to spend looking for a solution.
     * @param joint_names A vector of joint names for the IK computation.
     * @param callback A callback function that determines whether the given robot state is in collision or not
     */
    int CartToJntSearchWithCollision(const KDL::JntArray& q_in, const KDL::Frame& p_in, KDL::JntArray &q_out, const double &timeout, const std::vector<std::string> &joint_names, const boost::function<void(const KDL::JntArray&, const std::vector<std::string> &joint_names, bool&)> &callback);

     /**
     * @brief This method searches for and returns the least cost IK solution it finds.
     * @return < 0 if no solution is found
     * @param q_in The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(cob3_ik_->free_angle_) as 
     * as an input to the inverse kinematics. cob3_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     * @param timeout The amount of time (in seconds) to spend looking for a solution.
     * @param link_names A vector of link names for which the costs are computed and then added to determine the net cost.
     * @param joint_names A vector of joint names for the serial chain used for IK computation.
     * @param callback A callback function that determines whether the given robot state is in collision or not
     */
    int CartToJntSearchMinimumCost(const KDL::JntArray& q_in, const KDL::Frame& p_in, KDL::JntArray &q_out, const double &timeout, const std::vector<std::string> &joint_names,  const std::vector<std::string> &link_names, const boost::function<void(const KDL::JntArray&, const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, bool&, double&)> &callback);

    /**
       @brief A method to get chain information about the serial chain that the IK operates on 
       @param response This class gets populated with information about the joints that IK operates on, including joint names and limits.
    */
    void getChainInfo(kinematics_msgs::KinematicTreeInfo &response);

     /**
     * @brief This method searches for and returns the first solution it finds that also satisifies both user defined callbacks.
     *
     * @return < 0 if no solution is found
     * @param The kinematics request in kinematics_msgs::PositionIKRequest form
     * @param The set of links to check for collisions
     * @param The set of links to disable collision checks for
     * @param The robot state (use this to fill out state information for other joints)
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     * @param desired_pose_callback A callback function to which the desired pose is passed in
     * @param solution_callback A callback function to which IK solutions are passed in
     */
    int CartToJntSearchWithCallbacks(const kinematics_msgs::PositionIKRequest &ik_request, 
                                     const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                                     const motion_planning_msgs::RobotState &robot_state,
                                     const double &timeout,
                                     KDL::JntArray &q_out, 
                                     kinematics_msgs::KinematicsErrorCode &error_code,
                                     const boost::function<void(const kinematics_msgs::PositionIKRequest&, 
                                                                const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                                                                const motion_planning_msgs::RobotState &, kinematics_msgs::KinematicsErrorCode &)> &desired_pose_callback,
                                     const boost::function<void(const KDL::JntArray&, 
                                                                const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                                                                const motion_planning_msgs::RobotState &, kinematics_msgs::KinematicsErrorCode &)> &solution_callback);

    private:

    bool getCount(int &count, int max_count, int min_count);

    double search_discretization_angle_;

    int free_angle_;
  };
}
#endif// COB3_ARM_IK_SOLVER_H

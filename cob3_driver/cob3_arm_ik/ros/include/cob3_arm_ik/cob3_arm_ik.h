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

#ifndef COB3_ARM_IK_H
#define COB3_ARM_IK_H

#include <urdf/model.h>
#include <Eigen/Array>
#include <Eigen/LU>// provides LU decomposition
#include <kdl/chainiksolver.hpp>
#include <cob3_arm_ik_utils.h>
#include <cob3_arm_ik_constants.h>


namespace cob3_arm_ik
{
  class COB3ArmIK
  {
    public:
    
    /** @class
     *  @brief Inverse kinematics for the COB3 arm.
     *  @author Sachin Chitta <sachinc@willowgarage.com>
     *
     */
    COB3ArmIK();    
    ~COB3ArmIK(){};

    /** 
	@brief Initialize the solver by providing a urdf::Model and a root and tip name.
	@param A urdf::Model representation of the COB3 robot model
	@param The root joint name of the arm 
	@param The tip joint name of the arm
	@return true if initialization was successful, false otherwise.
    */
    bool init(urdf::Model robot_model, std::string root_name, std::string tip_name);

    /**
       @brief compute IK based on an initial guess for the shoulder pan angle.
       @param Input pose for end-effector
       @param Initial guess for shoulder pan angle
     */
    void computeIKShoulderPan(const Eigen::Matrix4f &g_in, const double &shoulder_pan_initial_guess);

    /**
       @brief compute IK based on an initial guess for the shoulder roll angle.
h       @param Input pose for end-effector
       @param Initial guess for shoulder roll angle
     */
    void computeIKShoulderRoll(const Eigen::Matrix4f &g_in, const double &shoulder_roll_initial_guess);

    
    std::vector<std::vector<double> > solution_ik_;/// a vector of ik solutions

    /**
       @brief get chain information about the arm. This populates the IK query response, filling in joint level information including names and joint limits. 
       @param The response structure to be filled in.
    */
    void getChainInfo(kinematics_msgs::KinematicTreeInfo &info);

    /**
       @brief get chain information about the arm.
    */
    kinematics_msgs::KinematicTreeInfo chain_info_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    void addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint,kinematics_msgs::KinematicTreeInfo &info);

    bool checkJointLimits(const std::vector<double> &joint_values);
 
    bool checkJointLimits(const double &joint_value, const int &joint_num);

    Eigen::Matrix4f grhs_, gf_, home_inv_, home_;

    std::vector<double> angle_multipliers_;

    std::vector<double> solution_;

    double shoulder_upperarm_offset_, upperarm_elbow_offset_, elbow_wrist_offset_, shoulder_wrist_offset_, shoulder_elbow_offset_, torso_shoulder_offset_x_, torso_shoulder_offset_y_, torso_shoulder_offset_z_;

    std::vector<double> min_angles_;

    std::vector<double> max_angles_;

  };
}
#endif// COB3_ARM_IK_H

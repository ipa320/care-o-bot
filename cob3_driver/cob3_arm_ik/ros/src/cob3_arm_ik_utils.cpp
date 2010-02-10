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

#include <cob3_arm_ik/cob3_arm_ik_utils.h>
#include <cob3_arm_ik/cob3_arm_ik_constants.h>

namespace cob3_arm_ik
{
  static const double IK_DEFAULT_TIMEOUT = 10.0;
  bool loadRobotModel(ros::NodeHandle node_handle, urdf::Model &robot_model, std::string &root_name, std::string &tip_name, std::string &xml_string)
  {
    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
    node_handle.searchParam(urdf_xml,full_urdf_xml);
    TiXmlDocument xml;
    ROS_DEBUG("Reading xml file from parameter server\n");
    std::string result;
    if (node_handle.getParam(full_urdf_xml, result))
      xml.Parse(result.c_str());
    else
    {
      ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
      return false;
    }
    xml_string = result;
    TiXmlElement *root_element = xml.RootElement();
    TiXmlElement *root = xml.FirstChildElement("robot");
    if (!root || !root_element)
    {
      ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
      exit(1);
    }
    robot_model.initXml(root);
    if (!node_handle.getParam("root_name", root_name)){
      ROS_FATAL("COB3IK: No root name found on parameter server");
      return false;
    }
    if (!node_handle.getParam("tip_name", tip_name)){
      ROS_FATAL("COB3IK: No tip name found on parameter server");
      return false;
    }
    return true;
  }

  bool getKDLChain(const std::string &xml_string, const std::string &root_name, const std::string &tip_name, KDL::Chain &kdl_chain)
  {
    // create robot chain from root to tip
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(xml_string, tree))
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    if (!tree.getChain(root_name, tip_name, kdl_chain))
    {
      ROS_ERROR("Could not initialize chain object");
      return false;
    }
    return true;
  }

  bool getKDLTree(const std::string &xml_string, const std::string &root_name, const std::string &tip_name, KDL::Tree &kdl_tree)
  {
    // create robot chain from root to tip
    if (!kdl_parser::treeFromString(xml_string, kdl_tree))
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    return true;
  }


  Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p)
  {
    Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
    for(int i=0; i < 3; i++)
    {
      for(int j=0; j<3; j++)
      {
        b(i,j) = p.M(i,j);
      }
      b(i,3) = p.p(i);
    }
    return b;
  }

  double computeEuclideanDistance(const std::vector<double> &array_1, const KDL::JntArray &array_2)
  {
    double distance = 0.0;
    for(int i=0; i< (int) array_1.size(); i++)
    {
      distance += (array_1[i] - array_2(i))*(array_1[i] - array_2(i));
    }
    return sqrt(distance);
  }


  double distance(const urdf::Pose &transform)
  {
    return sqrt(transform.position.x*transform.position.x+transform.position.y*transform.position.y+transform.position.z*transform.position.z);
  }


  bool solveQuadratic(const double &a, const double &b, const double &c, double *x1, double *x2)
  {
    double discriminant = b*b-4*a*c;
    if(fabs(a) < IK_EPS)
    {
      *x1 = -c/b;
      *x2 = *x1;
      return true;
    }
    //ROS_DEBUG("Discriminant: %f\n",discriminant);
    if (discriminant >= 0)
    {      
      *x1 = (-b + sqrt(discriminant))/(2*a); 
      *x2 = (-b - sqrt(discriminant))/(2*a);
      return true;
    } 
    else if(fabs(discriminant) < IK_EPS)
    {
      *x1 = -b/(2*a);
      *x2 = -b/(2*a);
      return true;
    }
    else
    {
      *x1 = -b/(2*a);
      *x2 = -b/(2*a);
      return false;
    }
  }

  Eigen::Matrix4f matrixInverse(const Eigen::Matrix4f &g)
  {
    Eigen::Matrix4f result = g;
    Eigen::Matrix3f Rt = Eigen::Matrix3f::Identity();

    Eigen::Vector3f p = Eigen::Vector3f::Zero(3);
    Eigen::Vector3f pinv = Eigen::Vector3f::Zero(3);

    Rt(0,0) = g(0,0);
    Rt(1,1) = g(1,1);
    Rt(2,2) = g(2,2);

    Rt(0,1) = g(1,0);
    Rt(1,0) = g(0,1);

    Rt(0,2) = g(2,0);
    Rt(2,0) = g(0,2);

    Rt(1,2) = g(2,1);
    Rt(2,1) = g(1,2);

    p(0) = g(0,3);
    p(1) = g(1,3);
    p(2) = g(2,3);

    pinv = -Rt*p;

    result(0,0) = g(0,0);
    result(1,1) = g(1,1);
    result(2,2) = g(2,2);

    result(0,1) = g(1,0);
    result(1,0) = g(0,1);

    result(0,2) = g(2,0);
    result(2,0) = g(0,2);

    result(1,2) = g(2,1);
    result(2,1) = g(1,2);

    result(0,3) = pinv(0);
    result(1,3) = pinv(1);
    result(2,3) = pinv(2);
  
    return result;
  }


  bool solveCosineEqn(const double &a, const double &b, const double &c, double &soln1, double &soln2)
  {
    double theta1 = atan2(b,a);
    double denom  = sqrt(a*a+b*b);

    if(fabs(denom) < IK_EPS) // should never happen, wouldn't make sense but make sure it is checked nonetheless
    {
#ifdef DEBUG
      std::cout << "denom: " << denom << std::endl;
#endif
      return false;
    }
    double rhs_ratio = c/denom;
    if(rhs_ratio < -1 || rhs_ratio > 1)
    {
#ifdef DEBUG
      std::cout << "rhs_ratio: " << rhs_ratio << std::endl;
#endif
      return false;
    }
    double acos_term = acos(rhs_ratio);
    soln1 = theta1 + acos_term;
    soln2 = theta1 - acos_term;

    return true;
  }


bool checkJointNames(const std::vector<std::string> &joint_names,
                       const kinematics_msgs::KinematicTreeInfo &chain_info)
  {    
    for(unsigned int i=0; i < chain_info.joint_names.size(); i++)
    {
      int index = -1;
      for(unsigned int j=0; j < joint_names.size(); j++)
      {
        if(chain_info.joint_names[i] == joint_names[j])
        {
          index = j;
          break;
        }
      }
      if(index < 0)
      {
        ROS_ERROR("Joint state does not contain joint state for %s.",chain_info.joint_names[i].c_str());
        return false;
      }
    }
    return true;
  }

  bool checkLinkNames(const std::vector<std::string> &link_names,
                      const kinematics_msgs::KinematicTreeInfo &chain_info)
  {
    if(link_names.empty())
      return false;
    for(unsigned int i=0; i < link_names.size(); i++)
    {
      if(!checkLinkName(link_names[i],chain_info))
        return false;
    }
    return true;   
  }

  bool checkLinkName(const std::string &link_name,
                   const kinematics_msgs::KinematicTreeInfo &chain_info)
  {
    for(unsigned int i=0; i < chain_info.link_names.size(); i++)
    {
		std::cout << "utils:checkLinkName:link_name = " << link_name << std::endl;
		std::cout << "utils:checkLinkName:chain_info.link_names[i] = " << chain_info.link_names[i] << std::endl;
      if(link_name == chain_info.link_names[i])
        return true;
    }
    return false;   
  }

  bool checkRobotState(motion_planning_msgs::RobotState &robot_state,
                     const kinematics_msgs::KinematicTreeInfo &chain_info)
  {
    if((int) robot_state.joint_state.position.size() != (int) robot_state.joint_state.name.size())
    {
      ROS_ERROR("Number of joints in robot_state.joint_state does not match number of positions in robot_state.joint_state");
      return false;
    }    
    if(!checkJointNames(robot_state.joint_state.name,chain_info))
    {
      ROS_ERROR("Robot state must contain joint state for every joint in the kinematic chain");
      return false;
    }
    return true;
  }

  bool checkFKService(kinematics_msgs::GetPositionFK::Request &request, 
                      kinematics_msgs::GetPositionFK::Response &response, 
                      const kinematics_msgs::KinematicTreeInfo &chain_info)
  {
    if(!checkLinkNames(request.fk_link_names,chain_info))
    {
      ROS_ERROR("Link name in service request does not match links that kinematics can provide solutions for1.");
      response.error_code.val = response.error_code.INVALID_LINK_NAME;
      return false;
    }
    if(!checkRobotState(request.robot_state,chain_info))
    {
      response.error_code.val = response.error_code.INVALID_ROBOT_STATE;
      return false;
    }
    return true;
  }

  bool checkIKService(kinematics_msgs::GetPositionIK::Request &request, 
                      kinematics_msgs::GetPositionIK::Response &response,
                      const kinematics_msgs::KinematicTreeInfo &chain_info)
  {
    if(!checkLinkName(request.ik_request.ik_link_name,chain_info))
    {
      ROS_ERROR("Link name in service request does not match links that kinematics can provide solutions for2.");      
      response.error_code.val = response.error_code.INVALID_LINK_NAME;
      return false;
    }
    if(!checkRobotState(request.ik_request.ik_seed_state,chain_info))
    {
      response.error_code.val = response.error_code.INVALID_ROBOT_STATE;
      return false;
    }
    if(request.timeout <= ros::Duration(0.0))
      request.timeout = ros::Duration(IK_DEFAULT_TIMEOUT);
    return true;
  }

  bool checkCollisionFreeIKService(kinematics_msgs::GetCollisionFreePositionIK::Request &request, 
                      kinematics_msgs::GetCollisionFreePositionIK::Response &response,
                      const kinematics_msgs::KinematicTreeInfo &chain_info)
  {
    if(!checkLinkName(request.ik_request.ik_link_name,chain_info))
    {
      ROS_ERROR("Link name in service request does not match links that kinematics can provide solutions for3.");      
      response.error_code.val = response.error_code.INVALID_LINK_NAME;
      return false;
    }
    if(!checkRobotState(request.ik_request.ik_seed_state,chain_info))
    {
      response.error_code.val = response.error_code.INVALID_ROBOT_STATE;
      return false;
    }
    if(request.timeout <= ros::Duration(0.0))
      request.timeout = ros::Duration(IK_DEFAULT_TIMEOUT);
    return true;
  }

  bool convertPoseToRootFrame(const geometry_msgs::PoseStamped &pose_msg, 
                              KDL::Frame &pose_kdl, 
                              const std::string &root_frame, 
                              const tf::TransformListener &tf)
  {
    geometry_msgs::PoseStamped pose_stamped;
    if(!convertPoseToRootFrame(pose_msg, pose_stamped, root_frame,tf))
      return false;
    tf::PoseMsgToKDL(pose_stamped.pose, pose_kdl);
    return true;
  }


  bool convertPoseToRootFrame(const geometry_msgs::PoseStamped &pose_msg, 
                              geometry_msgs::PoseStamped &pose_msg_out, 
                              const std::string &root_frame, 
                              const tf::TransformListener &tf)
  {
    geometry_msgs::PoseStamped pose_msg_in = pose_msg;
    ROS_DEBUG("Request:\nframe_id: %s\nPosition: %f %f %f\n:Orientation: %f %f %f %f\n",
              pose_msg_in.header.frame_id.c_str(),
              pose_msg_in.pose.position.x,
              pose_msg_in.pose.position.y,
              pose_msg_in.pose.position.z,
              pose_msg_in.pose.orientation.x,
              pose_msg_in.pose.orientation.y,
              pose_msg_in.pose.orientation.z,
              pose_msg_in.pose.orientation.w);
    tf::Stamped<tf::Pose> pose_stamped;
    poseStampedMsgToTF(pose_msg_in, pose_stamped);
    
    if (!tf.canTransform(root_frame, pose_stamped.frame_id_, pose_stamped.stamp_))
    {
      std::string err;    
      if (tf.getLatestCommonTime(pose_stamped.frame_id_, root_frame, pose_stamped.stamp_, &err) != tf::NO_ERROR)
      {
        ROS_ERROR("cob3_arm_ik:: Cannot transform from '%s' to '%s'. TF said: %s",pose_stamped.frame_id_.c_str(),root_frame.c_str(), err.c_str());
        return false;
      }
    }    
    try
    {
      tf.transformPose(root_frame, pose_stamped, pose_stamped);
    }
    catch(...)
    {
      ROS_ERROR("cob3_arm_ik:: Cannot transform from '%s' to '%s'",pose_stamped.frame_id_.c_str(),root_frame.c_str());
      return false;
    } 
    tf::poseStampedTFToMsg(pose_stamped,pose_msg_out);   
    return true;
  }



  int getJointIndex(const std::string &name,
                  const kinematics_msgs::KinematicTreeInfo &chain_info)
  {
    for(unsigned int i=0; i < chain_info.joint_names.size(); i++)
    {
      if(chain_info.joint_names[i] == name)
      {
          return i;
      }
    }
    return -1;
  }

  int getKDLSegmentIndex(const KDL::Chain &chain, 
                                    const std::string &name)
  {
    int i=0; // segment number
    while(i < (int)chain.getNrOfSegments())
    {
      if(chain.getSegment(i).getName() == name)
      {
        return i+1;
      }
      i++;
    }
    return -1;   
  }


}

/**
 * @file   sr_ros_wrapper.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
*
* Copyright 2015 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/

#pragma once

#include <string>
#include <vector>
#include <map>

#include "sr_standalone/shadow_hand.hpp"
#include <boost/scoped_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sr_hand/hand_commander.hpp>
#include <sr_robot_msgs/ControlType.h>
#include <sr_robot_msgs/ChangeControlType.h>
#include <sr_robot_msgs/ChangeControlTypeRequest.h>
#include <sr_robot_msgs/ChangeControlTypeResponse.h>
#include <sr_robot_msgs/BiotacAll.h>
#include <sr_robot_msgs/joint.h>

namespace shadow_robot_standalone
{

class ShadowHand::SrRosWrapper
{
  friend class ShadowHand;
public:
  SrRosWrapper();

  bool get_control_type(ControlType *current_ctrl_type);
  bool set_control_type(const ControlType &new_ctrl_type);

  void send_position(const std::string &joint_name, double target);
  void send_all_positions(const std::vector<double> &targets);
  void send_torque(const std::string &joint_name, double target);
  void send_all_torques(const std::vector<double> &targets);
  void spin(void);

  void joint_state_cb(const sensor_msgs::JointStateConstPtr &msg);
  void joint0_state_cb(const sensor_msgs::JointStateConstPtr &msg);
  void tactile_cb(const sr_robot_msgs::BiotacAllConstPtr &msg);

protected:
  std::map<std::string, JointState> joint_states_;
  std::vector<Tactile> tactiles_;

  boost::scoped_ptr<ros::NodeHandle> nh_;
  boost::scoped_ptr<ros::NodeHandle> n_tilde_;

  boost::scoped_ptr<shadowrobot::HandCommander> hand_commander_;

  ros::Subscriber joint_states_sub_, joint0_states_sub_;
  ros::Subscriber tactile_sub_;
  boost::unordered_map<std::string, ros::Publisher> torque_pubs_;
};

}  // namespace shadow_robot_standalone

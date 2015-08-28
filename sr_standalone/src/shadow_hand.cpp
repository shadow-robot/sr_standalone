/**
 * @file   shadow_hand.cpp
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

#include <vector>
#include <string>

#include "sr_standalone/shadow_hand.hpp"
#include "sr_standalone/sr_ros_wrapper.hpp"

using std::vector;
using std::string;

namespace shadow_robot_standalone
{

ShadowHand::ShadowHand()
  : wrapper_(new SrRosWrapper()) {}

ShadowHand::~ShadowHand()
{
  delete wrapper_;
}

bool ShadowHand::get_control_type(ControlType* control_type)
{
  return wrapper_->get_control_type(control_type);
}

bool ShadowHand::set_control_type(ControlType control_type)
{
  return wrapper_->set_control_type(control_type);
}

void ShadowHand::send_position(const string &joint_name, double target)
{
  wrapper_->send_position(joint_name, target);
}

void ShadowHand::send_all_positions(const vector<double> &targets)
{
  wrapper_->send_all_positions(targets);
}

void ShadowHand::send_torque(const string &joint_name, double target)
{
  wrapper_->send_torque(joint_name, target);
}

void ShadowHand::send_all_torques(const vector<double> &targets)
{
  wrapper_->send_all_torques(targets);
}

map<string, JointState> & ShadowHand::get_joint_states() const
{
  wrapper_->spin();
  return wrapper_->joint_states_;
}

vector<Tactile> & ShadowHand::get_tactiles() const
{
  wrapper_->spin();
  return wrapper_->tactiles_;
}

vector<string> ShadowHand::get_joints_with_state() const
{
  vector<string> joints_with_state;
  map<string, JointState>::const_iterator it = wrapper_->joint_states_.begin();
  while (it != wrapper_->joint_states_.end())
  {
    joints_with_state.push_back(it->first);
    ++it;
  }
  return joints_with_state;
}

vector<string> ShadowHand::get_controlled_joints() const
{
  vector<string> controlled_joints;
  boost::unordered_map<string, ros::Publisher>::const_iterator it = wrapper_->torque_pubs_.begin();
  while (it != wrapper_->torque_pubs_.end())
  {
    controlled_joints.push_back(it->first);
    ++it;
  }
  return controlled_joints;
}

}  // namespace shadow_robot_standalone

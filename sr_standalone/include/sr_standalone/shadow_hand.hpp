/**
 * @file   shadow_hand.hpp
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

namespace shadow_robot_standalone
{

struct JointState
{
  JointState() :
    position(), velocity(), effort() {}
  JointState(double _position, double _velocity, double _effort) :
    position(_position), velocity(_velocity), effort(_effort) {}
  double position;
  double velocity;
  double effort;
};

struct Tactile
{
  int pac0;
  int pac1;
  int pdc;

  int tac;
  int tdc;

  static const int no_of_electrodes = 19;
  int electrodes[no_of_electrodes];
};

enum ControlType
{
  POSITION_PWM,
  EFFORT_TORQUE
};

class ShadowHand
{
public:
  ShadowHand();
  ~ShadowHand();

  /**
   * Get the control type currently used on the hand.
   *
   * @param control_type Either position control over PWM, or
   *        effort control over Torque.
   *
   * @return true if success.
   */
  bool get_control_type(ControlType* control_type);

  /**
   * Set the control type to be used on the hand.
   *
   * @param control_type Either position control over PWM, or
   *        effort control over Torque.
   *
   * @return true if success.
   */
  bool set_control_type(ControlType control_type);

  /**
   * Send a position target, in radians, to the given joint's
   * position controller.
   *
   * @param joint_name Name of the joint to control
   * @param a position in radians
   */
  void send_position(const std::string &joint_name, double target);

  /**
   * Send position targets, in radians, to position controllers
   * of all joints. The order in targets is the same
   * as in response from get_controlled_joints()
   *
   * @param positions in radians for all joints
   */
  void send_all_positions(const std::vector<double> &targets);

  /**
   * Send a torque target, to the given joint.
   *
   * @param joint_name Name of the joint to control
   * @param a torque target.
   */
  void send_torque(const std::string &joint_name, double target);

  /**
   * Send torque targets, to all joints. The order in targets is the same
   * as in response from get_controlled_joints()
   *
   * @param torque targets.
   */
  void send_all_torques(const std::vector<double> &targets);

  /**
   * Retrieves the latest information about the joints.
   * will be empty if nothing has been published
   *
   * @return map from joint names to struct containing
   *         joint's position, velocity and effort.
   */
  std::map<std::string, JointState> & get_joint_states() const;
  /**
   * Retrieves the tactile data from the biotacs.
   *
   * @return A vector of tactiles in the following order:
   *         FF, MF, RF, LF, TH
   */
  std::vector<Tactile> & get_tactiles() const;

  /**
   * Get a list of joint names in the hand
   * that can be controlled
   *
   * @return vector containing joint names
   */
  std::vector<std::string> get_controlled_joints() const;

  /**
   * Get a list of joint names in the hand
   * for which state is reported
   *
   * @return vector containing joint names
   */
  std::vector<std::string> get_joints_with_state() const;

private:
  /*
   * Pimpl idiom for hiding implementation details in the header file
   */
  class SrRosWrapper;  // fwd declaration
  SrRosWrapper *wrapper_;

  ShadowHand(const ShadowHand& other)
  {
  }

  ShadowHand& operator=(const ShadowHand& other)
  {
  }
};

}  // namespace shadow_robot_standalone

#include "sr_standalone/shadow_hand.hpp"
#include "sr_standalone/sr_ros_wrapper.hpp"
#include <algorithm>

using namespace std;

namespace shadow_robot_standalone
{

ShadowHand::ShadowHand(int argc, char** argv)
  : wrapper_(new SrRosWrapper(argc, argv))
{
}

ShadowHand::~ShadowHand()
{
}

bool ShadowHand::set_control_type(ControlType control_type)
{
  wrapper_->control_type_ = control_type;
  return true;  // TODO check if control_type was actually set at the ROS side of things
}

void ShadowHand::send_position(const string &joint_name, double target)
{
}

void ShadowHand::send_torque(const string &joint_name, double target)
{
}

const JointStates & ShadowHand::get_joint_states() const
{
  return wrapper_->joint_states_;
}

const vector<Tactile> & ShadowHand::get_tactiles() const
{
  return wrapper_->tactiles_;
}

const vector<string> & ShadowHand::get_list_of_joints() const
{
  return wrapper_->joint_states_.names;
}

} // namespace

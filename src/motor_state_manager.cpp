#include "booster_motion/motor_state_manager.hpp"

#include <algorithm>

namespace booster_motion
{
namespace
{

constexpr std::array<b1::JointIndex, b1::kJointCnt> kJointMap = {
  b1::JointIndex::kHeadYaw,
  b1::JointIndex::kHeadPitch,
  b1::JointIndex::kLeftShoulderPitch,
  b1::JointIndex::kLeftShoulderRoll,
  b1::JointIndex::kLeftElbowPitch,
  b1::JointIndex::kLeftElbowYaw,
  b1::JointIndex::kRightShoulderPitch,
  b1::JointIndex::kRightShoulderRoll,
  b1::JointIndex::kRightElbowPitch,
  b1::JointIndex::kRightElbowYaw,
  b1::JointIndex::kWaist,
  b1::JointIndex::kLeftHipPitch,
  b1::JointIndex::kLeftHipRoll,
  b1::JointIndex::kLeftHipYaw,
  b1::JointIndex::kLeftKneePitch,
  b1::JointIndex::kCrankUpLeft,
  b1::JointIndex::kCrankDownLeft,
  b1::JointIndex::kRightHipPitch,
  b1::JointIndex::kRightHipRoll,
  b1::JointIndex::kRightHipYaw,
  b1::JointIndex::kRightKneePitch,
  b1::JointIndex::kCrankUpRight,
  b1::JointIndex::kCrankDownRight,
};

}  // namespace

std::string_view joint_name(b1::JointIndex joint)
{
  switch (joint) {
    case b1::JointIndex::kHeadYaw:
      return "HeadYaw";
    case b1::JointIndex::kHeadPitch:
      return "HeadPitch";
    case b1::JointIndex::kLeftShoulderPitch:
      return "LeftShoulderPitch";
    case b1::JointIndex::kLeftShoulderRoll:
      return "LeftShoulderRoll";
    case b1::JointIndex::kLeftElbowPitch:
      return "LeftElbowPitch";
    case b1::JointIndex::kLeftElbowYaw:
      return "LeftElbowYaw";
    case b1::JointIndex::kRightShoulderPitch:
      return "RightShoulderPitch";
    case b1::JointIndex::kRightShoulderRoll:
      return "RightShoulderRoll";
    case b1::JointIndex::kRightElbowPitch:
      return "RightElbowPitch";
    case b1::JointIndex::kRightElbowYaw:
      return "RightElbowYaw";
    case b1::JointIndex::kWaist:
      return "Waist";
    case b1::JointIndex::kLeftHipPitch:
      return "LeftHipPitch";
    case b1::JointIndex::kLeftHipRoll:
      return "LeftHipRoll";
    case b1::JointIndex::kLeftHipYaw:
      return "LeftHipYaw";
    case b1::JointIndex::kLeftKneePitch:
      return "LeftKneePitch";
    case b1::JointIndex::kCrankUpLeft:
      return "CrankUpLeft";
    case b1::JointIndex::kCrankDownLeft:
      return "CrankDownLeft";
    case b1::JointIndex::kRightHipPitch:
      return "RightHipPitch";
    case b1::JointIndex::kRightHipRoll:
      return "RightHipRoll";
    case b1::JointIndex::kRightHipYaw:
      return "RightHipYaw";
    case b1::JointIndex::kRightKneePitch:
      return "RightKneePitch";
    case b1::JointIndex::kCrankUpRight:
      return "CrankUpRight";
    case b1::JointIndex::kCrankDownRight:
      return "CrankDownRight";
  }

  return "Unknown";
}

std::size_t joint_to_index(b1::JointIndex joint)
{
  return static_cast<std::size_t>(joint);
}

MotorStateManager::MotorStateManager()
: Node("motor_state_manager")
{
  for (std::size_t i = 0; i < motor_state.size(); ++i) {
    motor_state[i].joint = kJointMap[i];
  }

  motor_state_subscriber = this->create_subscription<booster_interface::msg::LowState>(
    "/low_state",
    10,
    [this](booster_interface::msg::LowState::SharedPtr msg) {
      this->update_current_motor(msg);
    });
}

void MotorStateManager::update_current_motor(
  const booster_interface::msg::LowState::SharedPtr & msg)
{
  const auto & serial_states = msg->motor_state_serial;
  if (serial_states.size() < motor_state.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received %zu serial motor states, expected at least %zu.",
      serial_states.size(),
      motor_state.size());
  }

  const std::size_t count = std::min(serial_states.size(), motor_state.size());
  std::lock_guard<std::mutex> lock(mutex);
  for (std::size_t i = 0; i < count; ++i) {
    const auto & src = serial_states[i];
    auto & dst = motor_state[i];

    dst.joint = kJointMap[i];
    dst.valid = true;
    dst.mode = src.mode;
    dst.q = src.q;
    dst.dq = src.dq;
    dst.ddq = src.ddq;
    dst.tau_est = src.tau_est;
    dst.temperature = src.temperature;
    dst.lost = src.lost;
    dst.reserve = src.reserve;
  }
}

bool MotorStateManager::get_motor_state(b1::JointIndex joint, CurrentMotorState & state) const
{
  const auto index = joint_to_index(joint);
  if (index >= motor_state.size()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex);
  state = motor_state[index];
  return state.valid;
}

void MotorStateManager::print_motor_info(b1::JointIndex joint)
{
  CurrentMotorState state;
  if (!get_motor_state(joint, state)) {
    RCLCPP_WARN(
      get_logger(),
      "No /low_state data received yet for joint %d (%.*s).",
      static_cast<int>(joint),
      static_cast<int>(joint_name(joint).size()),
      joint_name(joint).data());
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "joint %d (%.*s): q=%.4f dq=%.4f ddq=%.4f tau_est=%.4f",
    static_cast<int>(joint),
    static_cast<int>(joint_name(joint).size()),
    joint_name(joint).data(),
    state.q,
    state.dq,
    state.ddq,
    state.tau_est);
}

void MotorStateManager::print_all_motor_info()
{
  for (const auto joint : kJointMap) {
    print_motor_info(joint);
  }
}

}  // namespace booster_motion

#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string_view>

#include "booster/robot/b1/b1_api_const.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace booster_motion
{

namespace b1 = booster::robot::b1;

struct CurrentMotorState
{
  b1::JointIndex joint{b1::JointIndex::kHeadYaw};
  bool valid{false};
  int8_t mode{0};
  float q{0.0F};
  float dq{0.0F};
  float ddq{0.0F};
  float tau_est{0.0F};
  int8_t temperature{0};
  uint32_t lost{0};
  std::array<uint32_t, 2> reserve{0, 0};
};

std::string_view joint_name(b1::JointIndex joint);
std::size_t joint_to_index(b1::JointIndex joint);

class MotorStateManager : public rclcpp::Node
{
public:
  explicit MotorStateManager();

  void update_current_motor(const booster_interface::msg::LowState::SharedPtr & msg);
  bool get_motor_state(b1::JointIndex joint, CurrentMotorState & state) const;
  void print_motor_info(b1::JointIndex joint);
  void print_all_motor_info();

private:
  mutable std::mutex mutex;
  std::array<CurrentMotorState, b1::kJointCnt> motor_state;
  rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr motor_state_subscriber;
};

}  // namespace booster_motion

#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <vector>

#include "booster_motion/joint_maps.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/motor_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace booster_motion
{

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

class MotorStateManager : public rclcpp::Node
{
public:
  explicit MotorStateManager();

  void update_current_motor(const booster_interface::msg::LowState::SharedPtr & msg);
  bool get_motor_state(b1::JointIndex joint, CurrentMotorState & state) const;
  void print_motor_info(b1::JointIndex joint);
  void print_all_motor_info();
  void disable_torque(b1::JointIndex joint);
  void disable_arm_torque();
  void disable_all_torque();
  void publish_motor_cmd(const booster_interface::msg::LowCmd & cmd);

private:
  mutable std::mutex mutex;
  std::array<CurrentMotorState, b1::kJointCnt> motor_state;
  rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr motor_cmd_publisher;
  rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr motor_state_subscriber;
  void disable_torque(const std::vector<b1::JointIndex> & joints);
};

}  // namespace booster_motion

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "booster_motion/motor_state_manager.hpp"
#include "rclcpp/rclcpp.hpp"

bool parse_joint_index(const char * input, int & joint_index)
{
  errno = 0;
  char * end = nullptr;
  const long value = std::strtol(input, &end, 10);

  if (errno != 0 || end == input || *end != '\0') {
    return false;
  }

  if (value < 0 || value >= static_cast<long>(booster_motion::b1::kJointCnt)) {
    return false;
  }

  joint_index = static_cast<int>(value);
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("read_joints");

  auto motor_state_manager = std::make_shared<booster_motion::MotorStateManager>();

  bool print_all = true;
  booster_motion::b1::JointIndex joint = booster_motion::b1::JointIndex::kHeadYaw;

  if (argc > 2) {
    RCLCPP_ERROR(logger, "Usage: read_joints [joint_index]");
    rclcpp::shutdown();
    return 1;
  }

  if (argc == 2) {
    int joint_index = 0;
    if (!parse_joint_index(argv[1], joint_index)) {
      RCLCPP_ERROR(
        logger,
        "Invalid joint index '%s'. Valid range is 0 to %zu.",
        argv[1],
        booster_motion::b1::kJointCnt - 1);
      rclcpp::shutdown();
      return 1;
    }

    print_all = false;
    joint = static_cast<booster_motion::b1::JointIndex>(joint_index);
    RCLCPP_INFO(
      logger,
      "Printing joint %d (%.*s).",
      joint_index,
      static_cast<int>(booster_motion::joint_name(joint).size()),
      booster_motion::joint_name(joint).data());
  } else {
    RCLCPP_INFO(logger, "Printing all joints.");
  }

  auto timer = motor_state_manager->create_wall_timer(
    std::chrono::seconds(1),
    [motor_state_manager, print_all, joint]() {
      if (print_all) {
        motor_state_manager->print_all_motor_info();
      } else {
        motor_state_manager->print_motor_info(joint);
      }
    });

  rclcpp::spin(motor_state_manager);
  rclcpp::shutdown();
  return 0;
}

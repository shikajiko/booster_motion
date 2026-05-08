#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "booster_motion/motor_state_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

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

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("read_joints");

  auto motor_state_manager = std::make_shared<booster_motion::MotorStateManager>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(motor_state_manager);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  bool print_all = true;
  booster_motion::b1::JointIndex joint = booster_motion::b1::JointIndex::kHeadYaw;

  if (argc > 2) {
    RCLCPP_ERROR(logger, "Usage: read_joints [joint_index]");
    executor.cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
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
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
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

  while (rclcpp::ok()) {
    if (print_all) {
      motor_state_manager->print_all_motor_info();
    } else {
      motor_state_manager->print_motor_info(joint);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}

#include <memory>
#include <string>
#include <thread>

#include "booster_motion/motor_state_manager.hpp"
#include "booster_motion/rpc_client.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("booster_motion");

  auto motor_state_manager = std::make_shared<booster_motion::MotorStateManager>();
  auto rpc_client = std::make_shared<booster_motion::RpcClient>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(motor_state_manager);
  executor.add_node(rpc_client);

  RCLCPP_INFO(
    logger,
    "Running motor_state_manager and rpc_client nodes.");

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  while (rclcpp::ok()) {
    std::string cmd;
    std::cin >> cmd;

    if (cmd == "eu") {
      const bool success = rpc_client->enable_upper_body_control();
      if (!success) {
        RCLCPP_ERROR(
          logger,
          "enable upper body control command failed");
      }
    } else if (cmd == "du") {
      const bool success = rpc_client->disable_upper_body_control();
      if (!success) {
        RCLCPP_ERROR(
          logger,
          "disable upper body control command failed");
      }
    } else if (cmd == "q" || cmd == "quit" || cmd == "exit") {
      break;
    } else {
      RCLCPP_WARN(
        logger,
        "Unknown command '%s'. Use 'eu', 'du', or 'q'.",
        cmd.c_str());
    }
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}

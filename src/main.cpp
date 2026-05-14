#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "booster_client_interface/srv/set_mode.hpp"
#include "booster_client_interface/srv/set_upper_control.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_joint_interface/msg/set_joints.hpp"
#include "booster_joint_interface/msg/set_torques.hpp"
#include "booster_joint_interface/msg/transition_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using SetMode = booster_client_interface::srv::SetMode;
using LowState = booster_interface::msg::LowState;
using SetJoints = booster_joint_interface::msg::SetJoints;
using SetTorques = booster_joint_interface::msg::SetTorques;
using TransitionCommand = booster_joint_interface::msg::TransitionCommand;
using SetUpperControl = booster_client_interface::srv::SetUpperControl;

constexpr uint8_t kLeftShoulderPitchId = 2;
constexpr uint8_t kRightShoulderPitchId = 6;
constexpr std::array<uint8_t, 8> kArmJointIds = {2, 3, 4, 5, 6, 7, 8, 9};
constexpr float kShoulderStepRad = 20.0F * 3.14159265358979323846F / 180.0F;
constexpr float kVelocityScale = 1.0F;

std::string trim(const std::string & input)
{
  const auto first = std::find_if_not(
    input.begin(),
    input.end(),
    [](unsigned char c) { return std::isspace(c) != 0; });

  const auto last = std::find_if_not(
    input.rbegin(),
    input.rend(),
    [](unsigned char c) { return std::isspace(c) != 0; }).base();

  if (first >= last) {
    return {};
  }

  return std::string(first, last);
}

std::string to_lower(std::string input)
{
  std::transform(
    input.begin(),
    input.end(),
    input.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return input;
}

std::vector<std::string> split_words(const std::string & input)
{
  std::istringstream stream(input);
  std::vector<std::string> words;
  std::string word;

  while (stream >> word) {
    words.push_back(word);
  }

  return words;
}

std::optional<uint8_t> parse_mode(const std::string & input)
{
  const auto mode = to_lower(input);

  if (mode == "0" || mode == "damp" || mode == "damping") {
    return TransitionCommand::MODE_DAMPING;
  }

  if (mode == "1" || mode == "stand" || mode == "prepare") {
    return TransitionCommand::MODE_STAND;
  }

  if (mode == "2" || mode == "walk" || mode == "walking") {
    return TransitionCommand::MODE_WALK;
  }

  if (mode == "3" || mode == "custom") {
    return TransitionCommand::MODE_CUSTOM;
  }

  return std::nullopt;
}

const char * mode_name(uint8_t mode)
{
  switch (mode) {
    case TransitionCommand::MODE_DAMPING:
      return "damping";
    case TransitionCommand::MODE_STAND:
      return "stand";
    case TransitionCommand::MODE_WALK:
      return "walk";
    case TransitionCommand::MODE_CUSTOM:
      return "custom";
    default:
      return "unknown";
  }
}

class BoosterMotionTerminal : public rclcpp::Node
{
public:
  BoosterMotionTerminal()
  : rclcpp::Node("booster_motion_terminal")
  {
    mode_client_ = create_client<SetMode>("client/set_mode");
    upper_control_client = create_client<SetUpperControl>("client/set_upper_control");
    joint_publisher_ = create_publisher<SetJoints>("joint/set_joints", 10);
    torque_publisher_ = create_publisher<SetTorques>("joint/set_torques", 10);
    joint_state_subscriber_ = create_subscription<LowState>(
      "joint/joint_states",
      10,
      [this](const LowState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(low_state_mutex_);
        latest_low_state_ = *msg;
      });
  }

  void run_terminal()
  {
    print_help();

    std::string line;
    while (rclcpp::ok() && !stop_requested_) {
      std::cout << "booster_motion> " << std::flush;
      if (!std::getline(std::cin, line)) {
        break;
      }

      handle_command(trim(line));
    }
  }

private:
  void print_help() const
  {
    std::cout
      << "\nCommands:\n"
      << "  mode damp|stand|walk|custom   Change robot mode through client/set_mode\n"
      << "  damp|stand|walk|custom        Same as mode <name>\n"
      << "  aru                           Right shoulder pitch +20 deg, velocity scale 1\n"
      << "  ard                           Right shoulder pitch -20 deg, velocity scale 1\n"
      << "  alu                           Left shoulder pitch +20 deg, velocity scale 1\n"
      << "  ald                           Left shoulder pitch -20 deg, velocity scale 1\n"
      << "  arms off                      Disable torque for all arm joints\n"
      << "  arms on                       Enable torque for all arm joints\n"
      << "  upc on                        Enable upper body custom control\n"
      << "  upc off                       Disable upper body custom control\n"
      << "  help                          Print this command list\n" 
      << "  quit                          Exit\n\n";
  }

  void handle_command(const std::string & command)
  {
    if (command.empty()) {
      return;
    }

    const auto words = split_words(command);
    const auto head = to_lower(words.front());

    if (head == "help" || head == "h" || head == "?") {
      print_help();
      return;
    }

    if (head == "quit" || head == "exit" || head == "q") {
      stop_requested_ = true;
      return;
    }

    if (head == "mode" || head == "m") {
      if (words.size() != 2) {
        RCLCPP_WARN(get_logger(), "Usage: mode damp|stand|walk|custom");
        return;
      }

      const auto mode = parse_mode(words[1]);
      if (!mode.has_value()) {
        RCLCPP_WARN(get_logger(), "Unknown mode: '%s'", words[1].c_str());
        return;
      }

      send_mode_request(*mode);
      return;
    }

    if (const auto mode = parse_mode(head); mode.has_value()) {
      send_mode_request(*mode);
      return;
    }

    if (head == "aru") {
      publish_shoulder_delta(kRightShoulderPitchId, kShoulderStepRad, "right shoulder up");
      return;
    }

    if (head == "ard") {
      publish_shoulder_delta(kRightShoulderPitchId, -kShoulderStepRad, "right shoulder down");
      return;
    }

    if (head == "alu") {
      publish_shoulder_delta(kLeftShoulderPitchId, kShoulderStepRad, "left shoulder up");
      return;
    }

    if (head == "ald") {
      publish_shoulder_delta(kLeftShoulderPitchId, -kShoulderStepRad, "left shoulder down");
      return;
    }

    if ((head == "arms" || head == "arm") && words.size() == 2 && to_lower(words[1]) == "off") {
      publish_arm_torque(false);
      return;
    }

    if ((head == "arms" || head == "arm") && words.size() == 3 &&
      to_lower(words[1]) == "torque" && to_lower(words[2]) == "off")
    {
      publish_arm_torque(false);
      return;
    }

    if ((head == "arms" || head == "arm") && words.size() == 2 && to_lower(words[1]) == "on") {
      publish_arm_torque(true);
      return;
    }

    if ((head == "arms" || head == "arm") && words.size() == 3 &&
      to_lower(words[1]) == "torque" && to_lower(words[2]) == "on")
    {
      publish_arm_torque(true);
      return;
    }

    if ((head == "upc") && words.size() == 2 && to_lower(words[1]) == "on") {
      send_upper_control(true);
      return;
    }

    if ((head == "upc") && words.size() == 2 && to_lower(words[1]) == "off") {
      send_upper_control(false);
      return;
    }


    

    RCLCPP_WARN(get_logger(), "Unknown command: '%s'", command.c_str());
  }

  void send_mode_request(uint8_t mode)
  {
    if (!mode_client_->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "Mode service 'client/set_mode' is not ready. Start booster_client/rpc_client_node first.");
      return;
    }

    auto request = std::make_shared<SetMode::Request>();
    request->mode = mode;

    RCLCPP_INFO(get_logger(), "Requesting mode change: %s (%u)", mode_name(mode), mode);
    mode_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<SetMode>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(get_logger(), "Mode change accepted: %s (%u)", mode_name(mode), mode);
        } else {
          RCLCPP_WARN(get_logger(), "Mode change failed: %s (%u)", mode_name(mode), mode);
        }
      });
  }

  void send_upper_control(bool enable)
  {
    if (!upper_control_client->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "Service 'client/set_upper_control' is not ready. Start booster_client/rpc_client_node first.");
      return;
    }

    auto request = std::make_shared<SetUpperControl::Request>();
    request->enable = enable;

    RCLCPP_INFO(get_logger(), "Requesting upc change");
    mode_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<SetUpperControl>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(get_logger(), "UPC change success");
        } else {
          RCLCPP_WARN(get_logger(), "UPC change failed");
        }
      });
  }

  void publish_shoulder_delta(uint8_t joint_id, float delta, const char * label)
  {
    const auto current_position = get_current_joint_position(joint_id);
    if (!current_position.has_value()) {
      RCLCPP_WARN(
        get_logger(),
        "No joint state for joint id %u yet. Start booster_joint_manager and wait for joint/joint_states.",
        joint_id);
      return;
    }

    SetJoints msg;
    auto & joint = msg.joints.emplace_back();
    joint.id = joint_id;
    joint.position = *current_position + delta;
    joint.velocity = kVelocityScale;

    joint_publisher_->publish(msg);
    RCLCPP_INFO(
      get_logger(),
      "Published %s command: joint=%u current=%.4f target=%.4f velocity_scale=%.2f",
      label,
      joint_id,
      *current_position,
      joint.position,
      joint.velocity);
  }

  void publish_arm_torque(bool enable_torque)
  {
    SetTorques msg;
    msg.ids.assign(kArmJointIds.begin(), kArmJointIds.end());
    msg.torque_enable = enable_torque;

    torque_publisher_->publish(msg);
    RCLCPP_INFO(
      get_logger(),
      "Published arm torque %s command for %zu joints",
      enable_torque ? "enable" : "disable",
      msg.ids.size());
  }

  std::optional<float> get_current_joint_position(uint8_t joint_id) const
  {
    std::lock_guard<std::mutex> lock(low_state_mutex_);
    if (!latest_low_state_.has_value()) {
      return std::nullopt;
    }

    const auto index = static_cast<std::size_t>(joint_id);
    if (index >= latest_low_state_->motor_state_serial.size()) {
      return std::nullopt;
    }

    return latest_low_state_->motor_state_serial[index].q;
  }

  rclcpp::Client<SetMode>::SharedPtr mode_client_;
  rclcpp::Publisher<SetJoints>::SharedPtr joint_publisher_;
  rclcpp::Publisher<SetTorques>::SharedPtr torque_publisher_;
  rclcpp::Subscription<LowState>::SharedPtr joint_state_subscriber_;

  mutable std::mutex low_state_mutex_;
  std::optional<LowState> latest_low_state_;
  std::atomic_bool stop_requested_{false};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto terminal = std::make_shared<BoosterMotionTerminal>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(terminal);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  terminal->run_terminal();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();

  return 0;
}

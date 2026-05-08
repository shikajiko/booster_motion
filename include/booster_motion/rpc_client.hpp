#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "booster_interface/srv/rpc_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "booster_motion/motor_state_manager.hpp"

namespace booster_motion
{

class RpcClient : public rclcpp::Node
{
public:
  explicit RpcClient(MotorStateManager &motor_state_manager);

  bool enable_upper_body_control(
    std::chrono::milliseconds service_timeout = std::chrono::milliseconds(5000),
    std::chrono::milliseconds response_timeout = std::chrono::milliseconds(3000));

  bool disable_upper_body_control(
    std::chrono::milliseconds service_timeout = std::chrono::milliseconds(5000),
    std::chrono::milliseconds response_timeout = std::chrono::milliseconds(3000));

private:
  bool set_upper_body_custom_control(
    bool is_enable,
    std::chrono::milliseconds service_timeout = std::chrono::milliseconds(5000),
    std::chrono::milliseconds response_timeout = std::chrono::milliseconds(3000));
    
  using RpcService = booster_interface::srv::RpcService;

  std::shared_ptr<RpcService::Request> make_upper_body_request(bool is_enable) const;
  std::string service_name;
  rclcpp::Client<RpcService>::SharedPtr client;
};

}  // namespace booster_motion

#include "booster_motion/rpc_client.hpp"
#include "booster_interface/message_utils.hpp"

namespace booster_motion
{

RpcClient::RpcClient(MotorStateManager &motor_state_manager)
: Node("rpc_client")
{
  (void)motor_state_manager;
  service_name = "booster_rpc_service";
  client = this->create_client<RpcService>(service_name);
}

bool RpcClient::enable_upper_body_control(
  std::chrono::milliseconds service_timeout,
  std::chrono::milliseconds response_timeout)
{
  return set_upper_body_custom_control(true, service_timeout, response_timeout);
}

bool RpcClient::disable_upper_body_control(
  std::chrono::milliseconds service_timeout,
  std::chrono::milliseconds response_timeout)
{
  return set_upper_body_custom_control(false, service_timeout, response_timeout);
}

bool RpcClient::set_upper_body_custom_control(
  bool is_enable,
  std::chrono::milliseconds service_timeout,
  std::chrono::milliseconds response_timeout)
{
  if (!client->wait_for_service(service_timeout)) {
    RCLCPP_ERROR(
      get_logger(),
      "RPC service '%s' is not available.",
      service_name.c_str());
    return false;
  }

  auto future = client->async_send_request(make_upper_body_request(is_enable));
  const auto status = future.wait_for(response_timeout);

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(
      get_logger(),
      "UpperBodyCustomControl(%s) RPC did not complete.",
      is_enable ? "true" : "false");
    return false;
  }

  const auto response = future.get();
  RCLCPP_INFO(
    get_logger(),
    "UpperBodyCustomControl(%s) response: status=%ld body=%s",
    is_enable ? "true" : "false",
    response->msg.status,
    response->msg.body.c_str());

  return response->msg.status == 0;
}

std::shared_ptr<RpcClient::RpcService::Request> RpcClient::make_upper_body_request(bool is_enable) const
{
  auto request = std::make_shared<RpcService::Request>();
  request->msg = booster_interface::CreateMsg<
    booster::robot::b1::LocoApiId::kUpperBodyCustomControl,
    booster::robot::b1::UpperBodyCustomControlParameter>(is_enable);
  return request;
}

}  // namespace booster_motion

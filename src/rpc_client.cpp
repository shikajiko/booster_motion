#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/message_utils.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class RpcClient :  public rclcpp::Node
{
public:
    RpcClient() : Node("rpc_client"){

    }
    void enable_upper_body_control();
    void disable_upper_body_control();

private:
    rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr client;
    auto request = std::make_shared<booster_interface::srv::RpcService::Request>();
    auto req_enable_upper_body_control = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kUpperBodyCustomControl,
        booster::robot::b1::UpperBodyCustomControlParameter>(true);
    >
    auto req_disable_upper_body_control = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kUpperBodyCustomControl,
        booster::robot::b1::UpperBodyCustomControlParameter>(false);
    >
        
}
void RpcClient::enable_upper_body_control(){
    while (!client->wait_for_service(1s)){
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "service not available, waiting again...");
    }
    auto future = rpc_client_->async_send_request(req_enable_upper_body_control);
    auto result = rclcpp::spin_until_future_complete(
        this->shared_from_this(),
        future,
        3s 
    );

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enable UpperBodyCustomControl");
        return false;
    }

    auto response = future.get();
    RCLCPP_INFO(
        this->get_logger(),
        "UpperBodyCustomControl response: status=%ld body=%s",
        response->msg.status,
        response->msg.body.c_str());

    return response->msg.status == 0;
}

void RpcClient::disable_upper_body_control(){
     while (!client->wait_for_service(1s)){
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "service not available, waiting again...");
    }
    auto future = rpc_client_->async_send_request(req_enable_disable_body_control);
    auto result = rclcpp::spin_until_future_complete(
        this->shared_from_this(),
        future,
        3s 
    );

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enable UpperBodyCustomControl");
        return false;
    }

    auto response = future.get();
    RCLCPP_INFO(
        this->get_logger(),
        "UpperBodyCustomControl response: status=%ld body=%s",
        response->msg.status,
        response->msg.body.c_str());

    return response->msg.status == 0;
}
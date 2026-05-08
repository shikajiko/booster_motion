#include <array>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>

#include "booster/robot/b1/b1_api_const.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "rclcpp/rclcpp.hpp"


namespace b1 = booster::robot::b1;

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

constexpr std::string_view joint_name(b1::JointIndex joint)
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
} // namespace

struct CurrentMotorState
{
    b1::JointIndex joint;
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
    MotorStateManager()
        : Node("motor_state_manager")
    {
        for (std::size_t i = 0; i < motor_state_.size(); ++i) {
            motor_state_[i].joint = kJointMap[i];
        }

        motor_state_subscriber_ = this->create_subscription<booster_interface::msg::LowState>(
            "/low_state",
            10,
            [this](booster_interface::msg::LowState::SharedPtr msg) {
                this->update_current_motor(msg);
            });
    }

    void update_current_motor(const booster_interface::msg::LowState::SharedPtr &msg)
    {
        const auto &serial_states = msg->motor_state_serial;
        if (serial_states.size() < motor_state_.size()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Received %zu serial motor states, expected at least %zu.",
                serial_states.size(),
                motor_state_.size());
        }

        const std::size_t count = std::min(serial_states.size(), motor_state_.size());
        std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t i = 0; i < count; ++i) {
            const auto &src = serial_states[i];
            auto &dst = motor_state_[i];

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

    void print_motor_info(b1::JointIndex joint)
    {
        CurrentMotorState state;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state = motor_state_[index];
        }

        if (!state.valid) {
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
            state.tau_est
          	);
    }

private:
    std::mutex mutex_;
    std::array<CurrentMotorState, b1::kJointCnt> motor_state_;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr motor_state_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorStateManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

#pragma once

#include <array>
#include <cstddef>
#include <string_view>

#include "booster/robot/b1/b1_api_const.hpp"

namespace booster_motion
{

namespace b1 = booster::robot::b1;

inline constexpr std::array<b1::JointIndex, b1::kJointCnt> kAllJoints = {
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

inline constexpr std::array<b1::JointIndex, 8> kArmJoints = {
  b1::JointIndex::kLeftShoulderPitch,
  b1::JointIndex::kLeftShoulderRoll,
  b1::JointIndex::kLeftElbowPitch,
  b1::JointIndex::kLeftElbowYaw,
  b1::JointIndex::kRightShoulderPitch,
  b1::JointIndex::kRightShoulderRoll,
  b1::JointIndex::kRightElbowPitch,
  b1::JointIndex::kRightElbowYaw,
};

std::string_view joint_name(b1::JointIndex joint);
std::size_t joint_to_index(b1::JointIndex joint);

}  // namespace booster_motion

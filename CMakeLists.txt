cmake_minimum_required(VERSION 3.8)
project(booster_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(booster_interface REQUIRED)

add_executable(motor_state_manager src/motor_state_manager.cpp)
target_include_directories(motor_state_manager PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../booster_robotics_sdk/include
)
ament_target_dependencies(motor_state_manager
  rclcpp
  booster_interface
)

install(TARGETS
  motor_state_manager
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

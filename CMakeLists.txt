cmake_minimum_required(VERSION 3.8)
project(booster_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(booster_interface REQUIRED)

add_library(booster_motion_nodes
  src/motor_state_manager.cpp
  src/rpc_client.cpp
)
target_include_directories(booster_motion_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../booster_robotics_sdk/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(booster_motion_nodes
  rclcpp
  booster_interface
)

add_executable(booster_motion src/main.cpp)
target_link_libraries(booster_motion
  booster_motion_nodes
)
ament_target_dependencies(booster_motion
  rclcpp
)

add_executable(read_joints src/read_joints.cpp)
target_link_libraries(read_joints
  booster_motion_nodes
)
ament_target_dependencies(read_joints
  rclcpp
)

install(TARGETS
  booster_motion
  read_joints
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS booster_motion_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
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

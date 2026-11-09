cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

# Lidar in, drive command out. algorithm.cpp is the part to edit.
add_executable(planner src/planner.cpp src/algorithm.cpp)
target_include_directories(planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(
  planner
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${vesc_msgs_TARGETS}
)

# Remote kill-switch receiver. Listens for the laptop-held controller's UDP
# heartbeat and publishes the /estop heartbeat the planner waits on. It is the
# only /estop publisher in the stack, so run exactly one.
add_executable(killswitch src/killswitch.cpp)
target_compile_features(killswitch PUBLIC c_std_99 cxx_std_17)
target_link_libraries(
  killswitch
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS planner killswitch
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when> a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(planner_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(planner_gap_follow src/planner_gap_follow.cpp)
target_include_directories(planner_gap_follow PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(planner_gap_follow PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(
  planner_gap_follow
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
)

# Emergency-stop console. Run in its own terminal; it needs a real stdin, which
# nodes started by ros2 launch do not get.
add_executable(estop src/estop.cpp)
target_compile_features(estop PUBLIC c_std_99 cxx_std_17)
target_link_libraries(
  estop
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
)

# Remote kill-switch receiver. Listens for the laptop-held controller's UDP
# heartbeat and drives /estop exactly like the estop console does. Run ONE of the
# two, never both - they would fight over /estop at 100 Hz.
add_executable(killswitch src/killswitch.cpp)
target_compile_features(killswitch PUBLIC c_std_99 cxx_std_17)
target_link_libraries(
  killswitch
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
)

# Constant-radius circle test, for measuring the real minimum turning radius.
# Respects /estop exactly like the planner does.
add_executable(circle_test src/circle_test.cpp)
target_compile_features(circle_test PUBLIC c_std_99 cxx_std_17)
target_link_libraries(
  circle_test
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS planner_gap_follow estop killswitch circle_test
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

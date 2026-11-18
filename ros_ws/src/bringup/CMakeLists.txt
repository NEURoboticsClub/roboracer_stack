cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Small helper nodes, run with `ros2 run bringup <name>` or from a launch file.
install(PROGRAMS
  scripts/imu_gravity_relay.py
  scripts/scan_instant_relay.py
  DESTINATION lib/${PROJECT_NAME}
)
# Imported by the relays above (Python finds it next to the script).
install(FILES scripts/monotonic_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(shared_link_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GpsValues.msg"
  "msg/KairosValues.msg"
  "msg/VehicleControl.msg"
  "srv/GetVehicleControl.srv"
  "srv/SetEStopState.srv"
  "srv/ExecCmd.srv"
  "srv/FieldUpdate.srv"
  DEPENDENCIES std_msgs
)

# already called somewhere caused duplicate error
# ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  shared_link_bridge/shared_link_bridge_node.py
  shared_link_bridge/keyboard_teleop.py
  shared_link_bridge/controller_teleop.py
  shared_link_bridge/estop_beacon.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  shared_link_bridge/UDPConnection.py
  shared_link_bridge/premade_msgs.py
  shared_link_bridge/tui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(wiferion_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Error.msg"
  "msg/Status.msg"
  "msg/MobileState.msg"
  "msg/StationaryState.msg"
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(wiferion_charger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(clearpath_ros2_socketcan_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wiferion_interfaces REQUIRED)

include_directories(
  include
)

add_library(wiferion_driver
  "src/driver.cpp"
)

set(DEPENDENCIES
  "can_msgs"
  "clearpath_ros2_socketcan_interface"
  "rclcpp"
  "std_msgs"
  "wiferion_interfaces"
)

ament_target_dependencies(wiferion_driver ${DEPENDENCIES})

add_executable(wiferion_node
  "src/node.cpp"
)

target_link_libraries(wiferion_node wiferion_driver)

ament_target_dependencies(wiferion_node ${DEPENDENCIES})

install(TARGETS wiferion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_package()

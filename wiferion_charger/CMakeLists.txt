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
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
  ament_cmake_copyright
  ament_cmake_cpplint
  ament_cmake_uncrustify
)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()

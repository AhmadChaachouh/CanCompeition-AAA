cmake_minimum_required(VERSION 3.8)
project(behavior-tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

#add_executable(detect_can src/detect_can.cpp)
#ament_target_dependencies(detect_can
#  rclcpp
#  std_msgs
#  sensor_msgs
#  geometry_msgs
#  behaviortree_cpp_v3
#  rosidl_typesupport_cpp
#)

#add_executable(can_detected src/can_detected.cpp)
#ament_target_dependencies(can_detected
#  rclcpp
#  std_msgs
#  sensor_msgs
#  geometry_msgs
#  behaviortree_cpp_v3
#  rosidl_typesupport_cpp
#)

add_executable(main src/main.cpp)
ament_target_dependencies(main
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  action_msgs
  rclcpp_action
  behaviortree_cpp_v3
  rosidl_typesupport_cpp
)

#target_link_libraries(detect_can behaviortree_cpp_v3::behaviortree_cpp_v3)
#target_link_libraries(can_detected behaviortree_cpp_v3::behaviortree_cpp_v3)
#target_link_libraries(main behaviortree_cpp_v3::behaviortree_cpp_v3)

install(TARGETS
#  detect_can
#  can_detected
  main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)


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

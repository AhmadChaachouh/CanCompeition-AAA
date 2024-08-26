cmake_minimum_required(VERSION 3.8)
project(robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add include directories
include_directories(
  include
)

add_executable(can_detected_node src/can_detected.cpp src/main)
ament_target_dependencies(can_detected_node 
  behaviortree_cpp_v3
)

add_executable(detect_can_node src/detect_can_node.cpp src/main)
ament_target_dependencies(detect_can_node 
    rclcpp
    cv_bridge
    OpenCV
    sensor_msgs
    geometry_msgs
    behaviortree_cpp_v3

)

# add_executable(approach_can_node src/approach_can.cpp)
# ament_target_dependencies(approach_can_node 
#   rclcpp 
#   sensor_msgs 
#   geometry_msgs 
#   behaviortree_cpp_v3
# )

# add_executable(can_approached_node src/can_approached.cpp)
# ament_target_dependencies(can_approached_node 
#   behaviortree_cpp_v3
# )

# add_executable(grip_can_node src/grip_can.cpp)
# ament_target_dependencies(grip_can_node
#   rclcpp
#   behaviortree_cpp_v3
# )

# add_executable(detect_qr_code_node src/detect_qr_code.cpp)
# ament_target_dependencies(detect_qr_code_node
#   rclcpp
#   cv_bridge
#   OpenCV
#   sensor_msgs
#   geometry_msgs
#   behaviortree_cpp_v3
# )

# Link libraries
target_link_libraries(detect_can_node
  rclcpp
  behaviortree_cpp_v3::behaviortree_cpp_v3
)

target_link_libraries(can_detected_node
  rclcpp
  behaviortree_cpp_v3::behaviortree_cpp_v3
)

add_executable(main src/main)
ament_target_dependencies(main
  rclcpp
  cv_bridge
  OpenCV
  sensor_msgs
  geometry_msgs
  behaviortree_cpp_v3
)

install(TARGETS
  can_detected_node
  DESTINATION lib/${PROJECT_NAME}

  detect_can_node
  DESTINATION lib/${PROJECT_NAME}

  # can_approached
  # DESTINATION lib/${PROJECT_NAME}

  # grip_can_node
  # DESTINATION lib/${PROJECT_NAME}

  # detect_qr_code_node
  # DESTINATION lib/${PROJECT_NAME}

  main
  DESTINATION lib/${PROJECT_NAME}
)


# Add parameter file installation
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
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

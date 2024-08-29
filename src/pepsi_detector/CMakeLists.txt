cmake_minimum_required(VERSION 3.8)
project(pepsi_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)  # Add this line

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)


# Add executable
add_executable(pepsi_subscriber src/pepsi_subscriber.cpp)

# Specify dependencies for the executable
ament_target_dependencies(pepsi_subscriber
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

add_executable(qr_code_subscriber src/qr_code_subscriber.cpp)

# Specify dependencies for the qr_code_subscriber executable
ament_target_dependencies(qr_code_subscriber
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Install the executable
install(TARGETS
  pepsi_subscriber
  qr_code_subscriber
  DESTINATION lib/${PROJECT_NAME})



install(PROGRAMS
  
  src/pepsi_publisher.py
  src/qr_code_detector.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY weights/
  DESTINATION share/${PROJECT_NAME}/weights
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch)



ament_package()

# Use the custom base image created earlier
FROM ros_base:latest

# Set environment variables
ENV ROS_DISTRO=galactic
ENV ROS_DOMAIN_ID=15

# Create a workspace directory
WORKDIR /ros_ws
COPY ./src ./src
# Build the workspace
RUN /bin/bash -c "source /opt/ros/galactic/setup.bash && colcon build"

# # Navigate to the Behavior Tree build directory and clean it, then build it
# WORKDIR /ros_ws/src/behavior_tree_cpp_v3
# RUN mkdir -p build && cd build && cmake .. && make

# Source the workspace and setup entrypoint
RUN echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

# Set entrypoint to run the ROS 2 launch file directly
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/galactic/setup.bash && source /ros_ws/install/setup.bash && ros2 launch pepsi_detector pepsi_detection_launch.py"]
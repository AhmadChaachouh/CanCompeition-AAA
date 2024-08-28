# Stage 1: Python dependencies
FROM python:3.9-slim as python-base

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt 

# Stage 2: Official ROS 2 Galactic base image
FROM ros:galactic-ros-base as ros2-base



# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=galactic

# Install necessary ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-${ROS_DISTRO}-cv-bridge \
	ros-${ROS_DISTRO}-image-transport \
	ros-${ROS_DISTRO}-geometry-msgs \
	ros-${ROS_DISTRO}-sensor-msgs \
	ros-${ROS_DISTRO}-rclpy \
	ros-${ROS_DISTRO}-laser-proc \
	&& rm -rf /var/lib/apt/lists/*

# Copy the Python dependencies from the first stage
COPY --from=python-base /usr/local/lib/python3.9/site-packages /usr/local/lib/python3.9/site-packages
COPY --from=python-base /usr/local/bin /usr/local/bin

# Create a workspace
WORKDIR /ros2_ws/src

# Copy ROS 2 package into the workspace
COPY ./src/qr_code_finder .

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run qr_code_finder qr_code_detector"]

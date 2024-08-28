# Stage 1: Install Python dependencies
FROM python:3.9-slim AS python-base

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Install PyTorch and its dependencies
RUN pip install --no-cache-dir \
    torch==1.9.0+cpu \
    torchvision==0.10.0+cpu \
    torchaudio==0.9.0 \
    -f https://download.pytorch.org/whl/torch_stable.html

# Install Ultralytics
RUN pip install --no-cache-dir ultralytics

# Stage 2: Official ROS 2 Galactic base image
FROM ros:galactic-ros-base

# Install additional dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-laser-proc \
    python3.9 \
    python3.9-dev \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*



# Copy Python packages from the first stage
COPY --from=python-base /usr/local/lib/python3.9 /usr/local/lib/python3.9
COPY --from=python-base /usr/local/bin /usr/local/bin


# Create a workspace
WORKDIR /ros2_ws/src

# Copy ROS 2 package into the workspace
COPY ./src/pepsi_detector .

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]


# Default command: Run your node, replace with the appropriate command
CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch pepsi_detector pepsi_detection_launch.py"]

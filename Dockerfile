# Use the official ROS Galactic base image
FROM osrf/ros:galactic-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-system-dev \
    libboost-thread-dev \
    liblog4cpp5-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a working directory
WORKDIR /app

# Clone and build BehaviorTree.CPP
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git \
    && cd BehaviorTree.CPP \
    && mkdir build && cd build \
    && cmake .. \
    && make \
    && make install

# Copy your application files into the container
COPY . /app

# Build your application (assuming it uses BehaviorTree.CPP)
RUN cmake . && make

# Set the command to run your application
CMD ["./your_executable"]

# Or for a Python app, you might use:
# CMD ["python3", "your_script.py"]

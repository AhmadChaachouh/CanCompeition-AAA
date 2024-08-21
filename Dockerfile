# Start with a base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create a working directory
WORKDIR /app

# Copy files into the container
COPY . /app

# Install Python dependencies (if applicable)
RUN pip3 install --no-cache-dir -r requirements.txt

# Build your project (if applicable)
# RUN make or cmake ..

# Set the command to run your application
CMD ["./your_executable"]

# Or for a Python app, you might use:
# CMD ["python3", "your_script.py"]

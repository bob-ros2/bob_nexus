# Mastermind Universal Image (Experiment 7!)
FROM ros:humble-ros-base

# Non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Install System Dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    libsdl2-dev \
    libsdl2-ttf-dev \
    nlohmann-json3-dev \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python Dependencies from the project
# We'll assume the project is mounted at /app
# But we can pre-install some common ones for speed
RUN pip3 install --no-cache-dir \
    pyyaml \
    qdrant-client \
    openai \
    flask

# Set Workspace
WORKDIR /app

# Setup ROS 2 Environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Default Command
CMD ["/bin/bash"]

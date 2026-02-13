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

# Set Workspace
WORKDIR /app

# Copy Requirements first for better caching
COPY requirements/ /app/requirements/

# Install Python Dependencies from requirements files
RUN pip3 install --no-cache-dir \
    -r requirements/requirements.txt \
    -r requirements/qdrant.txt

# Setup ROS 2 Environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /app/ros2_ws/install/setup.bash ]; then source /app/ros2_ws/install/setup.bash; fi" >> /root/.bashrc

# Copy the rest of the application
COPY . /app/

# Build ROS 2 Workspace (if src is not empty)
RUN . /opt/ros/humble/setup.bash && \
    if [ -d ros2_ws/src ] && [ "$(ls -A ros2_ws/src)" ]; then \
    colcon build --symlink-install; \
    fi

# Ensure scripts are executable
RUN chmod +x mastermind.sh \
    && chmod +x master/cli.sh \
    && chmod +x master/dashboard.sh \
    && find master -name "*.sh" -o -name "*.py" -exec chmod +x {} + \
    && find skills -name "*.sh" -o -name "*.py" -exec chmod +x {} + \
    && chmod +x onboarding.sh

# Default Command
CMD ["/bin/bash"]

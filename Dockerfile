# Mastermind Universal Image (Experiment 7!)
FROM ros:humble-ros-base

LABEL org.opencontainers.image.source=https://github.com/bob-ros2/bob-nexus
LABEL org.opencontainers.image.description="BOB NEXUS Orchestrator Framework Base Image"

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
    jq \
    vim \
    git \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Set Workspace
WORKDIR /app

# Copy Requirements first for better caching
COPY requirements/ /app/requirements/

# Install Python Dependencies
RUN pip3 install --no-cache-dir \
    -r requirements/requirements.txt \
    -r requirements/qdrant.txt

# Setup ROS 2 Environment and PATH in bashrc
ENV PATH="/app/master:${PATH}"
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /app/ros2_ws/install/setup.bash ]; then source /app/ros2_ws/install/setup.bash; fi" >> /root/.bashrc \
    && echo 'export PATH="/app/master:$PATH"' >> /root/.bashrc

# Copy the framework code
# Note: .dockerignore ensures secrets (entities, .env, conf.yaml) are NOT copied.
COPY . /app/

# Ensure scripts are executable
RUN chmod +x mastermind.sh \
    && chmod +x master/cli.sh \
    && chmod +x master/dashboard.sh \
    && chmod +x master/chat.sh \
    && find master -name "*.sh" -o -name "*.py" -exec chmod +x {} + \
    && find skills -name "*.sh" -o -name "*.py" -exec chmod +x {} + \
    && chmod +x onboarding.sh

# Expose volumes for persistence
# entities: generated agents
# memory: Qdrant/JSON storage
# ros2_ws: Cloned source packages and build files
# master/config: conf.yaml
VOLUME ["/app/entities", "/app/memory", "/app/ros2_ws", "/app/master/config"]

# Default Command
# For a fresh image, the user should run ./onboarding.sh first.
CMD ["/bin/bash"]

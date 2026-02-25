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
    ffmpeg \
    v4l-utils \
    curl \
    jq \
    vim \
    git \
    sudo \
    docker.io \
    docker-compose-v2 \
    && rm -rf /var/lib/apt/lists/*

# Set Workspace
WORKDIR /app

# Copy Requirements first for better caching
COPY requirements/ /app/requirements/

# Install Python Dependencies
RUN pip3 install --no-cache-dir \
    --constraint requirements/constraints.txt \
    -r requirements/requirements.txt \
    -r requirements/qdrant.txt

# Setup ROS 2 Environment and PATH in bashrc
# Standard Project Paths
ENV BOB_NEXUS_DIR="/app"
ENV BOB_LLM_DIR="/app/ros2_ws/src/bob_llm"
ENV PATH="/app/master:${PATH}"

# Prepare Core Underlay (Swarm 10.1)
# We bake the core framework repositories directly into the base image.
RUN mkdir -p /app/ros2_ws/src && cd /app/ros2_ws/src \
    && git clone --depth 1 https://github.com/bob-ros2/bob_llm.git \
    && git clone --depth 1 https://github.com/bob-ros2/bob_launch.git \
    && git clone --depth 1 https://github.com/bob-ros2/bob_topic_tools.git \
    && git clone --depth 1 https://github.com/bob-ros2/bob_msgs.git \
    && git clone --depth 1 https://github.com/bob-ros2/bob_sdlviz.git

# Build Core Underlay
RUN . /opt/ros/humble/setup.sh \
    && cd /app/ros2_ws \
    && colcon build --event-handlers console_cohesion+ \
    && rm -rf build log

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /app/ros2_ws/install/setup.bash ]; then source /app/ros2_ws/install/setup.bash; fi" >> /root/.bashrc \
    && echo 'NEXUS_ROOT=$(find /app /root /home -maxdepth 2 -name "mastermind.sh" 2>/dev/null | head -n 1 | xargs dirname 2>/dev/null || echo "/app")' >> /root/.bashrc \
    && echo 'if [ -f "$NEXUS_ROOT/.profile" ]; then source "$NEXUS_ROOT/.profile"; fi' >> /root/.bashrc \
    && echo 'export PATH="$NEXUS_ROOT/master:$PATH"' >> /root/.bashrc \
    && echo "alias ll='ls -alF --color=auto'" >> /root/.bashrc \
    && echo "alias ls='ls --color=auto'" >> /root/.bashrc \
    && echo "PS1='\[\033[01;36m\][🤖]\[\033[01;32m\]\u@\${NAME:-\$HOSTNAME}\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /root/.bashrc

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

# Default Command
# For a fresh image, the user should run ./onboarding.sh first.
CMD ["/bin/bash"]

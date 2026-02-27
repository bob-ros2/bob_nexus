#!/bin/bash
# Swarm 9.21: Multi-Image Nexus Rebuild

# Ensure HOST_NEXUS_DIR is exported for the compose call
export HOST_NEXUS_DIR=$(pwd)

echo "[*] Ensuring global volumes exist..."
docker volume create nexus_pipes >/dev/null
docker volume create nexus_entities >/dev/null

echo "[*] Tearing down existing isolated Nexus..."
docker compose -f docker/compose-nexus.yaml down || true

echo "[*] Ensuring global network (alpha) is clean for Compose..."
docker network rm alpha 2>/dev/null || true

echo "[*] Building Base Image (bob-nexus:latest)..."
docker build -t bob-nexus:latest .

echo "[*] Building AV Extension Image (bob-nexus:av)..."
docker build -t bob-nexus:av -f docker/Dockerfile.av .

echo "[*] Building Isolated Master (bob-nexus-isolated)..."
docker compose -f docker/compose-nexus.yaml build --no-cache

echo "[*] Launching Mastermind Swarm..."
docker compose -f docker/compose-nexus.yaml up -d

echo "[*] Waiting for Core initialization..."
sleep 2

echo "[*] Swarm is up and running."
echo "[*] Use './master/cli/talk.py' to interact with agents."

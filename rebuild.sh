#!/bin/bash
# Swarm 9.21: Multi-Image Nexus Rebuild

# Ensure HOST_NEXUS_DIR is exported for the compose call
export HOST_NEXUS_DIR=$(pwd)

echo "[*] Ensuring global volumes exist..."
docker volume create nexus_pipes >/dev/null
docker volume create nexus_entities >/dev/null

echo "[*] Tearing down existing isolated Nexus..."
docker compose -f docker/compose-nexus.yaml down || true

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

echo "[*] Entering Nexus Console..."
docker exec -it bob_nexus_isolated bash

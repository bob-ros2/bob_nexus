#!/bin/bash
# Swarm 9.21: Multi-Image Nexus Rebuild

# Ensure HOST_NEXUS_DIR is exported for the compose call
export HOST_NEXUS_DIR=$(pwd)

echo "[*] Ensuring global volumes exist..."
docker volume create nexus_pipes >/dev/null
docker volume create nexus_entities >/dev/null

echo "[*] Ensuring global network (alpha) exists with correct MTU..."
CURRENT_MTU=$(docker network inspect alpha --format '{{index .Options "com.docker.network.driver.mtu"}}' 2>/dev/null || echo "0")
if [ "$CURRENT_MTU" != "1280" ]; then
    echo "    [!] MTU mismatch (found $CURRENT_MTU, need 1280). Recreating alpha network..."
    docker network rm alpha 2>/dev/null || true
    docker network create --opt com.docker.network.driver.mtu=1280 alpha
else
    echo "    [ok] Network alpha already has MTU 1280."
fi

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

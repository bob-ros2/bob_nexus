# Ensure HOST_NEXUS_DIR is exported for the compose call
export HOST_NEXUS_DIR=$(pwd)

echo "[*] Tearing down existing isolated Nexus..."
docker compose -f docker/compose-nexus.yaml down || true

echo "[*] Building Base Image (bob-nexus)..."
docker build -t bob-nexus:latest .

echo "[*] Building Isolated Master (bob-nexus-isolated)..."
docker compose -f docker/compose-nexus.yaml build

echo "[*] Launching Mastermind Swarm..."
docker compose -f docker/compose-nexus.yaml up -d

echo "[*] Waiting for Core initialization..."
sleep 2

echo "[*] Entering Nexus Console..."
docker exec -it bob_nexus_isolated bash

# Welcome to bob_nexus 🚀 (v0.5.0)

![bob-nexus](media/bob-nexus.png)

**The Central Nervous System and Orchestration Hub for the bob-ros2 Ecosystem.**

`bob_nexus` is the authoritative management layer for spawning, configuring, and deploying specialized AI entities. It acts as the "Home of the Swarm," providing a unified interface to orchestrate both LLM-based agents and standard ROS 2 nodes (visualizers, drivers, etc.) across various environments (Host, Docker, Swarm).

---

## 🚀 Quick Start & Onboarding

### 1. Initialize the Nexus
```bash
# This will clone core dependencies and build the internal ros2_ws
./onboarding.sh
```

### 2. Configure your Environment
Edit the generated `master/config/.env` file and add your API keys (e.g., DeepSeek, OpenAI).

### 3. Wake Up the Swarm
```bash
./swarm.sh spawn  # Create entities from templates (First time only)
./swarm.sh up     # Start the spawned swarm
# This will spin up the management layer and all default agents.
```

### 4. Create your first Assistant
```bash
# cli is available in the global PATH after awakening
cli spawn assistant alice bob_llm
cli up alice
```

### 5. Chat with the Swarm
```bash
# Connect via the cloud backend (OAI) - Default
./master/chat.sh --backend oai

# Integrated ROS-Backend (via local bob_llm node)
./master/chat.sh --backend bob_llm
```

---

## ⚙️ Operation Modes (Host vs. Swarm)

Bob Nexus supports two primary execution modes, each handling entity workspaces differently.

### 🏠 Host Mode (`mode: "host"`)
Entities run as native processes on the host system.
- **Workspace**: The folder `./entities/<category>/<name>` **IS** the actual working directory.
- **I/O**: Logs (`stdout.log`, `stderr.log`) and build artifacts are written directly to this host folder.
- **Best for**: Lightweight agents, GUI-heavy nodes, or environments where Docker isolation is unnecessary.

### 🐳 Swarm Mode (`mode: "swarm"`)
Entities run as isolated Docker containers within a managed project.
- **Registry (Host)**: `./entities/<category>/<name>` acts as the **Blueprint/Registry**. It provides the initial configuration and seeds the entity.
- **Workspace (Volume)**: The "Reality" lives in a **Named Docker Volume** (mounted at `/root`). This ensures high-performance I/O and persistence across container lifecycles.
- **Syncing**: Onboarding logic (within the container) automatically synchronizes changes from the Registry to the Volume on startup.

---

## 🛠️ System Overview

`bob_nexus` leverages **ROS 2 (Humble)** as its underlying transport and discovery mechanism. While most entities are high-level agents, the communication backbone relies on the resilience of the ROS 2 ecosystem.

### Key Concepts
- **Self-Assembling Entities (SAE)**: Each entity dynamically builds its own ROS 2 environment on startup using JIT template processing and automated `onboarding.sh` execution.
- **Unified Manifest (`agent.yaml`)**: The single point of truth for an entity's identity, repositories, and capabilities.
- **Host-to-Container Permission Sync**: Automatic mapping of `HOST_UID`/`HOST_GID` to ensure all files remain owned by the host user.
- **Dynamic Skill Binding**: Shared capabilities (Skills) are bundled to entities, enabling instant updates across the swarm.
- **Hierarchical Orchestration**: Layered Docker Compose support—merge global blueprints with entity-specific overrides.
- **Secrets Vault**: Secure credential management via read-only Docker mounts (`master/secrets`), ensuring sensitive keys never touch environment variables or logs.
- **Integrated Quality Suite**: Validate configurations, symlinks, and network health via `make test`.

---

## 📡 Governance & Dashboards

Access the collective consciousness through multiple interfaces:

### 1. The Swarm Controller (`./swarm.sh`)
The primary orchestration entry point for fleet-wide operations.
- **up / down**: Manage the lifecycle of the entire Nexus or specific entities.
- **status**: Check the health of all containers and services.

### 2. The Agent Talk TUI (`python3 master/cli/talk.py`)
A specialized TUI for interactive mission control.
- **Mission Communication**: Direct chat link to an `agent_core` state machine living in an entity's home directory.
- **Live Monitoring**: See agent status and mission logs in a cyberpunk-styled interface.

### 3. The Nexus Chat Interface (`./master/chat.sh`)
The direct communication link to your entities.
- **Backend OAI**: Direct link to your cloud LLM (DeepSeek, OpenAI, etc.).
- **Backend Bob**: Interactive ROS 2 link to your local `bob_llm` node.

### 4. Vision & Streaming
Specialized templates (like `twitch_stream`) allow entities to act as high-performance visual dashboards with integrated FFmpeg streaming support (Twitch/YouTube).

### 5. Observability (Grafana & Loki)
The Nexus includes a built-in observability stack (`infrastructure/observer`) based on Grafana, Loki, and Promtail.
- **Real-time Logs**: Aggregates logs from all entities and Docker containers.
- **NEXUS Health Dashboard**: Pre-configured Grafana dashboard for system-wide telemetry.

---

## 🏗️ Architecture

### Directory Structure
```text
.
├── master/          # Core orchestration logic (Python) & configuration
│   └── config/      # Central persistent configuration (.env, conf.yaml)
├── skills/          # Reusable skill modules (e.g., memory, director)
├── templates/       # Entity blueprints and Docker Composers
├── entities/        # Generated entity instances (Registry)
├── requirements/    # Environment dependency definitions
└── ros2_ws/         # The global ROS 2 workspace (Read-Only for assistants)
```

### The Mastermind CLI (`cli`)
The `cli` tool is your scalpel for entity lifecycles. It is automatically available in your PATH once the Nexus environment is sourced (or via `master/cli.sh`).

| Command | Description | Example |
| :--- | :--- | :--- |
| `spawn` | Create a new entity from a template | `cli spawn assistant bob bob_llm` |
| `up` | Start an entity (Host, Docker, or Compose) | `cli up bob` |
| `down` | Gracefully stop an entity | `cli down bob` |
| `status` | List all entities, their status, and PIDs/CIDs | `cli status` |
| `link` | Attach a shared skill to an entity | `cli link bob memory` |
| `refresh` | Re-generate entity system prompt | `cli refresh bob` |
| `refresh-skills` | Re-bundle all skills defined in manifest | `cli refresh-skills bob` |

---

## ⚙️ Configuration (`conf.yaml`)

Custom orchestration and behavioral logic is defined in `master/config/conf.yaml`.

### Orchestration Modes
- **Host**: Entities run as local processes.
- **Swarm**: Entities run as Docker containers or Compose projects.

### Entity Categories & Workspace Governance
- **Infrastructure/Assistant**: Global `ros2_ws` is mounted as **Read-Only**.
- **Master**: Full **Read-Write** access to the global workspace.
- **Isolated Assistants**: Overlay-based workspace for custom repo builds.

---

## 🧠 System Prompts & Dynamic Skills

The Nexus manages entity behavior through a sophisticated system prompt orchestration layer.

### 1. Externalized Prompts (Best Practice)
We strongly recommend using `system_prompt: ./system_prompt.txt` in your `agent.yaml`. This avoids YAML formatting issues and allows for marker-based dynamic skill injection.

### 2. Automated Skill Injection
The Nexus automatically advertises linked skills by looking for specialized markers in your prompt file:
```text
# --- NEXUS SKILLS PROMPT START ---
(Dynamically updated skill descriptions go here)
# --- NEXUS SKILLS PROMPT END ---
```
Use `cli refresh <name>` to forcefully re-scan linked `SKILL.md` files and update the agent's awareness.

---

## 🛡️ Quality & Integrity
Maintain system standards via the **Quality Suite**:
- `make lint`: Static analysis with **Ruff**.
- `make format`: Consistent code formatting.
- `make test`: Global integrity checks (YAML, Paths, Symlinks).
- `python3 tests/system_sanity.py`: E2E verification using a Mock LLM Backend.

---

## 🗺️ Roadmap
- **Zenoh Bridge**: Encrypted internet tunnels for cross-site swarms.
- **Stray Logic**: Integration of non-technical strategic entities (Finance, Marketing).
- **Hardened Isolation**: Per-entity system users for extreme process segregation.
- **Perception Layer**: Dedicated Vision-Language (VLM) processing nodes.

---
> *"The identity is independent of the matter. Whether process, container, or distributed across networks—the nexus remains the core."* -- Experiment 7!

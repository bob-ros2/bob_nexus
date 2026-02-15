# bob_nexus 🚀

**The Central Nervous System and Orchestration Hub for the bob-ros2 Ecosystem.**

`bob_nexus` is the authoritative management layer for spawning, configuring, and deploying specialized AI entities. It acts as the "Home of the Mastermind," providing a unified interface to orchestrate both LLM-bases agents and standard ROS 2 nodes (visualizers, drivers, etc.) across various environments (Host, Docker, Swarm).

---

## � Quick Start & Onboarding

For a fresh setup (native or inside a container), use the automated onboarding script to initialize your environment, download core ROS 2 dependencies, and build the workspace.

### 1. Initialize the Nexus
```bash
# This will clone bob_llm, bob_launch, and bob_topic_tools and build the ros2_ws
./onboarding.sh
```

### 2. Configure your Environment
Edit the generated `master/config/.env` file and add your API keys (e.g., DeepSeek, OpenAI, or Matrix).

### 3. Wake Up the Mastermind
```bash
./mastermind.sh
# Select [1] AWAKENING to start the core management entity.
```

### 4. Create your first Assistant
```bash
# cli.sh is now conveniently available in the root!
./cli.sh spawn assistant alice bob_llm
./cli.sh up alice
```

### 5. Chat with the Swarm
```bash
# Connect to the local bob_llm assistant
./master/chat.sh --backend bob_llm

# Or use the cloud backend (OAI)
./master/chat.sh --backend oai
```

---

## �🛠️ System Overview

`bob_nexus` leverages **ROS 2 (Humble)** as its underlying transport and discovery mechanism. While most entities are high-level agents, the communication backbone relies on the resilience of the ROS 2 ecosystem.

### Key Logic
- **Decentralized Capabilities**: Skills are developed centrally and symlinked to entities.
- **Template-Driven**: Entities are born from blueprints in `templates/` with recursive variable resolution.
- **Dynamic Orchestration**: Support for `subprocess` (Host), `docker run` (Swarm), and `docker compose` (Project Swarm).
- **Quality Assured**: Built-in integrity suite to validate configurations, symlinks, and network health.

---

## 📡 Governance & Dashboards

Access the collective consciousness through multiple interfaces:

### 1. The Awakening Console (`./mastermind.sh`)
The primary interactive entry point.
- **Toggle Core**: Start/Stop the main Mastermind entity.
- **Integrated View**: Quick access to Telemetry, Manual Shell, and the Integrated Dashboard.

### 2. The Nexus Dashboard (`./master/dashboard.sh`)
Real-time awareness of the Swarm.
- **Terminal Mode** (`./dashboard.sh`): A high-performance cyberpunk terminal dashboard with live CPU/MEM/Entity stats.
- **Status Mode** (`./dashboard.sh status`): Instant one-time status dump.
- **Window Mode** (`./dashboard.sh window`): A native high-contrast OS GUI (Tkinter) with copyable paths and auto-refresh.

### 3. The Nexus Chat Interface (`./master/chat.sh`)
The direct communication link to your entities.
- **Backend OAI**: Direct link to your cloud LLM (DeepSeek, OpenAI, etc.).
- **Backend Bob**: Interactive ROS 2 link to your local `bob_llm` node.

---

## 🏗️ Architecture

### Directory Structure
```yaml
.
├── master/          # Core orchestration logic (Python) & Dashboard
│   └── config/      # Central persistent configuration (.env, conf.yaml)
├── skills/          # Reusable skill modules (e.g., memory, vision)
├── templates/       # Entity blueprints and Docker Composers
│   └── composers/   # Reusable Docker Compose templates for groups/swarms
├── entities/        # Generated entity instances (The "Swarm")
├── requirements/    # Environment dependency definitions
└── tests/           # Integrity and architecture validation suite
```

### The Mastermind CLI
The `cli.sh` is your scalpel for entity lifecycles. It is symlinked to the root for easy access.

| Command | Description | Example |
| :--- | :--- | :--- |
| `spawn` | Create a new entity from a template | `./cli.sh spawn assistant bob bob_launch` |
| `up` | Start an entity (Host, Docker, or Compose) | `./cli.sh up bob` |
| `down` | Gracefully stop an entity | `./cli.sh down bob` |
| `status` | List all entities, their status, and PIDs/CIDs | `./cli.sh status` |
| `link` | Attach a shared skill to an entity | `./cli.sh link bob memory` |
| `refresh` | Re-generate entity system prompt | `./cli.sh refresh bob` |

---

## ⚙️ Configuration (`conf.yaml`)

Custom orchestration and behavioral logic is defined in `master/config/conf.yaml`.

### Orchestration Modes
- **Host**: Entities run as local processes.
- **Swarm**: Entities run as Docker containers or Compose projects.
- **Networking**: Support for custom Docker networks and ROS Discovery Servers.

### Default Skill Policy
Manage entity "weaponry" by category. Spawning an entity automatically links skills defined here:
```yaml
skills:
  master:
    - ["core", "memory"]
    - ["core", "sdlviz"]
    - ["core", "send_matrix"]
  assistant:
    - ["core", "memory"]
  defaults:
    - ["core", "memory"]
```

---

## 🛡️ Quality & Integrity
Maintain the standards of the stack using the **Quality Suite**:
- `make lint`: Static analysis with **Ruff**.
- `make format`: Consistent code formatting.
- `make test`: Global integrity checks (YAML validity, Symlink health).
- **System Sanity Checks**: Verified E2E testing of the whole stack.

### 🧪 System Sanity Tests
The Nexus includes a dedicated integration suite that mocks the OAI environment to verify tool execution and entity lifecycles without any cost.

**Run locally:**
```bash
python3 tests/system_sanity.py -v
```

**Run in an isolated Docker environment:**
```bash
cd tests && docker compose -f docker-compose.test.yaml up --build
```

---

## 🗺️ Roadmap

Future enhancements for the Mastermind:
- **Polymorpher Swarm**: Expansion beyond ROS-only entities to include pure Inference (llama.cpp), Infrastructure (Qdrant), and Bridge (Zenoh) entities.
- **Global Swarm (Bridge)**: Implementation of Zenoh-based encrypted tunnels for cross-network/internet ROS communication.
- **User-per-Entity Prinzip**: Implementation of strict process isolation by creating dedicated system users for each spawned entity (Security Hardening).
- **Perception Layer**: Dedicated entities for Vision-Language Models (Moondream) and sensory edge-processing.
- **Integrated Memory Explorer**: Web-view for the shared vector database.
- **Auto-Scale Swarm**: Dynamic spawning of assistants based on topic load.

---

> *"The identity is independent of the matter. Whether process, container, or distributed across networks—the nexus remains the core."* -- Experiment 7!

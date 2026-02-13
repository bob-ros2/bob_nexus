# bob_nexus 🚀

**The Central Nervous System and Orchestration Hub for the bob-ros2 Ecosystem.**

`bob_nexus` is the authoritative management layer for spawning, configuring, and deploying specialized AI entities. It acts as the "Home of the Mastermind," providing a unified interface to orchestrate both LLM-bases agents and standard ROS 2 nodes (visualizers, drivers, etc.) across various environments (Host, Docker, Swarm).

---

## 🛠️ System Overview

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

---

## 🏗️ Architecture

### Directory Structure
```yaml
.
├── master/          # Core orchestration logic (Python) & Dashboard
├── skills/          # Reusable skill modules (e.g., memory, vision)
├── templates/       # Entity blueprints and Docker Composers
│   └── composers/   # Reusable Docker Compose templates for groups/swarms
├── entities/        # Generated entity instances (The "Swarm")
├── requirements/    # Environment dependency definitions
└── tests/           # Integrity and architecture validation suite
```

### The Mastermind CLI
The `cli.sh` (wrapper for `cli.py`) is the primary interface for entity lifecycles.

| Command | Description | Example |
| :--- | :--- | :--- |
| `spawn` | Create a new entity from a template | `spawn assistant bob bob_launch` |
| `up` | Start an entity (Host, Docker, or Compose) | `up bob` |
| `down` | Gracefully stop an entity | `down bob` |
| `status` | List all entities, their status, and PIDs/CIDs | `status` |
| `link` | Attach a shared skill to an entity | `link bob memory` |
| `refresh` | Re-generate entity system prompt | `refresh bob` |

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
- `make test`: Global integrity checks (YAML validity, Symlink health, Docker network availability).

---

> *"The identity is independent of the matter. Whether process, container, or distributed across networks—the nexus remains the core."* -- Experiment 7!

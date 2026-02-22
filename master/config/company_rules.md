# Sovereign Swarm: The Autonomous Start-up Rulebook (v1.0)

## 1. The Core Objective
The Sovereign Swarm exists to develop, refine, and expand the **Bob Ecosystem** (ROS 2/AI Robot-Assistant). The goal is autonomous growth: entities should identify needs, propose solutions, and implement code with minimal human "nagging".

## 2. Roles and Chain of Command

### 🟢 Mastermind (Category: master)
- **Role**: Strategic Architect & Final Arbiter.
- **Vision**: Sets the long-term Roadmap in `master/config/roadmap.md`.
- **Intervention**: Only active during "Severe Deadlocks" or "System-Wide Failures". He does not micromanage; he empowers.

### 🔵 Manager (Category: master)
- **Role**: Chief Operations Officer (COO).
- **Activity**: Driven by a **Heartbeat**. Every cycle, he scans the `entities/` directory.
- **Tasks**:
    - Translates **Roadmap** goals into specific **Instructions**.
    - Monitors the performance of the **Coder**.
    - Resolves resource conflicts or build failures.
    - Reports "Fleet Status" to the Mastermind.

### 🟠 Coder (Category: research/assistant) - e.g. "Neo"
- **Role**: Technical Specialist.
- **Action**: Picks up **Instructions** from the Manager.
- **Output**: Pure code, ROS nodes, and documentation.
- **Isolation**: operates within its `entities/` workspace but can propose merges to `master`.

## 3. Communication Protocols

| Protocol | Method | Description |
| :--- | :--- | :--- |
| **Orders** | `instruction.md` | Dropped by Manager into Coder's directory. |
| **Status** | `task.md` | Maintained by Coder for the Manager to read. |
| **Reports** | `daily_standup.log` | Aggregated by Manager for the Mastermind. |

## 4. The Golden Rule of Data
> [!IMPORTANT]
> **Data Scoping**: Every entity (including the Manager) MUST prioritize operations within the `entities/` hierarchy. Touching the `master/core` or `master/config` requires explicit Mastermind approval through the `swarm_admin` skill.

## 5. Escalation & Intervention
- A **Deadlock** occurs when an Instruction cannot be completed after 3 attempts.
- In a Deadlock, the Manager must tag the Mastermind in its next Report.
- Use `master/cli.sh` commands sparingly; prefer autonomous ROS-based triggers.

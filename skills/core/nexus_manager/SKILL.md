---
name: Nexus Manager
description: "Advanced swarm orchestration and mission delegation tools for the Prime agent."
---

# Nexus Manager Skill

This skill allows a high-level agent (Prime) to coordinate other entities in the Bob Nexus swarm.

## Capabilities

### 1. `swarm_status`
Returns the operational status of all known entities in the swarm.

### 2. `swarm_delegate`
Delegates a technical mission to another agent (e.g., Neo) by writing to its `inbox.json`.

### 3. `swarm_check_result`
Reads the `outbox.json` of a delegated entity to retrieve mission results.

## Usage Guidelines
- Always check `swarm_status` before delegating to ensure the target is available.
- Use structured missions (YAML/JSON) to ensure clear communication between agents.

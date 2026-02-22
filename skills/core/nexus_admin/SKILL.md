---
name: "nexus_admin"
description: "Core administrative tools for the Bob Nexus Swarm. Allows listing entities, controlling their lifecycle (up, down, spawn), and retrieving logs."
---

# Nexus Admin Skill
This skill provides the direct bridge between the Code Agent and the Swarm's orchestration layer.

## Tools
- `list_swarm()`: Provides a comprehensive status overview of all entities.
- `get_status(name, category)`: Detailed status including current task summary.
- `assign_instruction(name, instruction, category)`: Direct task delivery to an entity.
- `control_entity(action, name, category, template)`: Direct lifecycle control (spawn, up, down).
- `get_logs(name, lines=50)`: Retrieves real-time logs from an entity container.

## Delegation Pattern
Assigning tasks to specialized nodes is a key feature of the Swarm.
Example:
1. `get_status("neo")`: Check if Neo is idle.
2. `assign_instruction("neo", "Refactor chat_backend.py to fix SSE parsing")`: Delegate a coding task.
3. `get_status("neo")`: Monitor progress.

## Usage
Use these tools to maintain awareness of the swarm's state, perform operational commands, and **delegate complex work** to other expert nodes without leaving the chat interface.

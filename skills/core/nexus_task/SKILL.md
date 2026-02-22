---
name: "nexus_task"
description: "Persistent task management for the Nexus Swarm. Tracks objectives, progress, and blockers in a centralized nexus_task.md file."
---

# Nexus Task Skill
This skill allows the agent to maintain a high-level view of current work and ensure continuity across sessions.

## Tools
- `get_task_list()`: Reads the current global task list.
- `update_task_list(content)`: Overwrites the global task list with new content.
- `add_task_item(item)`: Appends a new item to the task list.

## Usage
Use this to document the swarm's goals and track progress on complex multi-step instructions.

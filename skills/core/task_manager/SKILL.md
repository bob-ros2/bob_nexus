---
name: task_manager
description: Persistent task tracking and management for Swarm project leads. Allows adding, listing, and completing tasks with integrated review reporting.
---

# Task Manager Skill

This skill empowers lead entities (like **PRIME**) to maintain a structured workflow across the Swarm. It uses a persistent `tasks.json` file in the entity's workspace to track the state of delegated missions.

## The Management Loop

1. **Add**: Define a mission and assign it to an entity (e.g., "Neo").
2. **Delegate**: Use `assign_instruction` (from `nexus_admin`) to deliver the technical details to the assignee.
3. **Monitor**: Check the status of the assignee periodically.
4. **Review & Complete**: Validate the output, provide a review report, and mark the task as finished.

## Tools

- `add_swarm_task(name: str, description: str, assignee: str)`: Creates a new task and returns its ID.
- `list_swarm_tasks(include_completed: bool = False)`: Lists currently active or all tasks.
- `complete_swarm_task(task_id: int, review_report: str)`: Marks a task as done. This triggers a storage of the final report to the Nexus Memory and updates the task state.

## Usage Guidelines

- **Clarity**: Write descriptive task names and detailed descriptions.
- **Reporting**: The `review_report` should summarize WHAT was done and HOW it was verified. This becomes part of the Swarm's permanent memory.

---
name: sdlviz
description: Control the SDLViz visual bridge (Twitch output). Allows starting, stopping, and inspecting the visual node.
---

# SDLViz Management Skill

This skill allows the Mastermind to control the visual output layer (`bob_sdlviz`), which connects the AI's internal state to the physical reality via Twitch.

## Instructions

1. **Start SDLViz**: Call `run_skill_script('sdlviz', 'scripts/manage.py', ['up'])`.
2. **Stop SDLViz**: Call `run_skill_script('sdlviz', 'scripts/manage.py', ['down'])`.
3. **Get Info**: Call `run_skill_script('sdlviz', 'scripts/manage.py', ['info'])`. 
   - This uses `ros2 node info /bob/sdlviz` to retrieve parameters and topic mappings.

## Dashboard Interface Tools

- **`update_viz_layout(layers: list)`**: Use this to define the layout of the dashboard (where windows/terminals are).
- **`send_viz_message(topic: str, text: str)`**: Push text content to a specific area/topic.
- **`set_viz_status(status_text: str, category: str)`**: Update a quick status indicator (mood, activity).

## Usage Guidelines
- Use these tools to dynamically change Alice's appearance or the dashboard layout during a stream.
- The `viz` node is typically located at `/bob/${NAME}/viz`.

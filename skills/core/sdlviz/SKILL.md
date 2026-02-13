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

## Usage Guidelines
- Use this when you want to change what is being displayed to the audience.
- Always check `info` if you are unsure about which topics the visualizer is currently listening to.

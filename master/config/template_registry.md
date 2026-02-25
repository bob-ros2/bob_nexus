# Nexus Template Registry

This registry provides an authoritative overview of all standardized entity templates within the Bob Nexus ecosystem.

| Template | Category | Type | Description | Repositories |
| :--- | :--- | :--- | :--- | :--- |
| **bob_sdlviz** | `infrastructure` | Generic (ROS 2 Enabled) | SDL-based ROS 2 visualizer for Bob. | bob_launch, bob_msgs, bob_sdlviz |
| **coder** | `assistant` | Agent (State Machine) | Technical specialist (Neo) for code and system tasks. | None |
| **face_marker** | `infrastructure` | ROS 2 Node | ROS 2 Face recognition and marker overlay. | bob_face, bob_msgs |
| **heartbeat** | `infrastructure` | ROS 2 Node | - | None |
| **manager** | `master` | ROS 2 Node | High-level fleet manager and coordinator. | bob_launch, bob_msgs, bob_llm, bob_topic_tools |
| **mastermind** | `master` | ROS 2 Node | The core orchestrator and strategic architect. | bob_launch, bob_msgs, bob_llm, bob_topic_tools |
| **twitch_bot** | `assistant` | Generic (ROS 2 Enabled) | Twitch integration bot for LLM interaction. | bob_launch, bob_msgs, bob_llm, bob_topic_tools, twitch_bot |
| **twitch_stream** | `infrastructure` | ROS 2 Node | - | External (repos.yaml) |
| **webvideo** | `infrastructure` | ROS 2 Node | Web Overlay Renderer (Pyside6/Chromium) | bob_av_tools |
## Schema (nexus.yaml)

| Field | Description |
| :--- | :--- |
| `onboarding` | Boolean. If true, triggers the onboarding script. |
| `category` | Classification (e.g., `assistant`, `master`, `infrastructure`). |
| `description` | Human-readable overview of the template purpose. |
| `repositories` | List of Git URLs to be cloned into the entity workspace. |
| `nexus_agent` | Block defining LLM/Agent properties (Trigger for `agent_core.py`). |



def update_dashboard(message: str, category: str = "INFO"):
    """
    Updates the SDLViz dashboard with a status message.
    Categories: INFO, WARNING, ERROR, SUCCESS
    """
    # Simply log for the visualization bridge
    print(f"[SDLViz] Node Update: [{category}] {message}")
    # In a real Swarm, we would publish to a ROS 2 topic here.
    return f"Dashboard updated with {category} message."


def set_swarm_mood(mood: str):
    """
    Sets the visual 'mood' of the swarm on the dashboard (e.g. 'calm', 'alert', 'active').
    """
    print(f"[SDLViz] Swarm Mood set to: {mood}")
    return f"Swarm mood is now: {mood}"

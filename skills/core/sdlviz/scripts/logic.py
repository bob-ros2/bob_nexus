import subprocess

def show_message(text: str):
    """
    Displays a message on the SDLViz dashboard.
    """
    # In this simulation, we just log it. 
    # In a real setup, this would publish to a ROS 2 topic or call a service.
    print(f"[SDLViz] Displaying message: {text}")
    return f"Message displayed: {text}"

def set_swarm_status(status: str):
    """
    Updates the swarm status indicator (e.g. 'OK', 'WARNING', 'ERROR').
    """
    print(f"[SDLViz] Swarm Status set to: {status}")
    return f"Status updated to: {status}"

def list_active_nodes():
    """
    Returns a list of active ROS 2 nodes currently visible in the swarm.
    """
    try:
        result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, check=False)
        if result.returncode == 0:
            return result.stdout.splitlines()
        return "Error: ROS 2 environment not sourced or no nodes found."
    except Exception as e:
        return str(e)

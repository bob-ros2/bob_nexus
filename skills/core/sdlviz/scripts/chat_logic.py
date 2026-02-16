import json
from std_msgs.msg import String

# Module-level variable to store the ROS node
_node = None
_publisher_events = None

def register(module, node):
    """
    Called by bob_llm to register tools.
    We use this to capture the ROS node instance.
    """
    global _node, _publisher_events
    _node = node
    
    # Create the publisher for SDLViz events
    # Important: In a multi-entity setup, we need a way to find the correct viz node.
    # By default, we assume it's one level up if we are in /.../llm
    
    # Get current namespace
    ns = _node.get_namespace()
    # If we are in /bob/alice/llm, target viz is likely at /bob/alice/viz
    target_ns = ns
    if ns.endswith('/llm'):
        target_ns = ns[:-4]
    
    topic = f"{target_ns}/viz/events"
    _node.get_logger().info(f"[SDLViz Skill] Initializing publisher on: {topic}")
    _publisher_events = _node.create_publisher(String, topic, 10)
    
    # Use the default register to extract other functions
    from bob_llm.tool_utils import register as default_register
    return default_register(module, _node)

def update_viz_layout(layers: list):
    """
    Updates the SDLViz layout with a list of layer definitions.
    Layers can be of type 'String', 'MarkerLayer', or 'VideoStream'.
    Example: [{"type": "String", "topic": "/bob/log", "area": [10, 10, 400, 200]}]
    """
    if not _publisher_events:
        return "Error: Publisher not initialized."
    
    msg = String()
    msg.data = json.dumps(layers)
    _publisher_events.publish(msg)
    return f"Sent layout update with {len(layers)} layers."

def send_viz_message(topic: str, text: str):
    """
    Sends a text message to a specific topic that a Viz layer might be watching.
    Useful for updating a 'String' layer.
    """
    if not _node:
        return "Error: Node not initialized."
    
    # Create a transient publisher if it doesn't exist? 
    # Or just use the node's capability?
    # For now, let's just use a simple String publisher on the fly if needed, 
    # but that's inefficient. Better to have it managed.
    
    pub = _node.create_publisher(String, topic, 10)
    msg = String()
    msg.data = text
    pub.publish(msg)
    _node.destroy_publisher(pub) # Not ideal but works for sporadic updates
    
    return f"Sent text to {topic}."

def set_viz_status(status_text: str, category: str = "INFO"):
    """
    Quickly updates the status/mood of the agent on the dashboard.
    Uses a standard topic /bob/status (configurable).
    """
    ns = _node.get_namespace()
    if ns.endswith('/llm'):
        ns = ns[:-4]
        
    topic = f"{ns}/status"
    return send_viz_message(topic, f"[{category}] {status_text}")

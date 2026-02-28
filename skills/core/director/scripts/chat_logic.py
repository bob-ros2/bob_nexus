import json
from std_msgs.msg import String

# Module-level variables to store the ROS node and publisher
_node = None
_publisher_events = None

def register(module, node):
    """
    Called by bob_llm to register tools.
    """
    global _node, _publisher_events
    _node = node

    # Get current namespace (e.g., /bob/streamer/llm)
    ns = _node.get_namespace()
    
    # Target the viz node in the parent namespace (e.g., /bob/streamer/viz/events)
    # The user's bob_launch.yaml uses:
    # viz node name: streamer (in the entity namespace)
    # so the topic is /bob/streamer/streamer/events
    
    # If we are in /bob/streamer/llm, target viz is likely at /bob/streamer/streamer
    target_ns = ns
    if ns.endswith("/llm"):
        target_ns = ns[:-4]

    # In the twitch_stream template, the sdlviz node is named 'streamer'
    topic = f"{target_ns}/streamer/events"
    _node.get_logger().info(f"[Director Skill] Initializing publisher on: {topic}")
    _publisher_events = _node.create_publisher(String, topic, 10)

    # Use the default register to extract other functions
    from bob_llm.tool_utils import register as default_register
    return default_register(module, _node)

def update_director_layout(layers: list):
    """
    Updates the SDLViz layout with a list of layer definitions.
    Example: [{"id": "cam", "type": "VideoStream", "topic": "/tmp/cam_fifo", "area": [0,0,854,480]}]
    """
    if not _publisher_events:
        return "Error: Director publisher not initialized."

    msg = String()
    msg.data = json.dumps(layers)
    _publisher_events.publish(msg)
    return f"Director: Sent layout update with {len(layers)} layers."

def send_director_message(topic: str, text: str):
    """
    Sends a text message to a specific topic (e.g., for a 'String' layer).
    """
    if not _node:
        return "Error: Node not initialized."

    pub = _node.create_publisher(String, topic, 10)
    msg = String()
    msg.data = text
    pub.publish(msg)
    _node.destroy_publisher(pub)

    return f"Director: Published message to {topic}."

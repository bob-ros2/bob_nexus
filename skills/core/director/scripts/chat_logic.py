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
    # but the topic is typically /entity_ns/events instead of /entity_ns/streamer/events
    topic = f"{target_ns}/events"
    _node.get_logger().info(f"[Director Skill] Initializing publisher on: {topic}")
    _publisher_events = _node.create_publisher(String, topic, 10)

    # Use the default register to extract other functions
    from bob_llm.tool_utils import register as default_register
    return default_register(module, _node)

def update_director_layout(layers: list):
    """
    Updates the visual dashboard (SDLViz) layout. 
    Requires a list of layer objects. IMPORTANT: Refer to load_skill('director') for the full Layer Protocol.
    Key fields: id, type, action, expire, area, topic, title, text, text_color, bg_color, align, line_limit, clear_on_new.
    Example: [{"id": "cam", "type": "Image", "topic": "/image", "area": [0,0,854,480], "title": "Main Camera"}]
    """
    if isinstance(layers, str):
        try:
            layers = json.loads(layers)
        except Exception as e:
            return f"Error parsing layers JSON string: {e}"

    if not isinstance(layers, list):
        return f"Error: layers must be a list, got {type(layers).__name__}"

    if not _publisher_events:
        return "Error: Director publisher not initialized."

    msg = String()
    msg.data = json.dumps(layers)
    _publisher_events.publish(msg)
    return f"Director: Sent layout update with {len(layers)} layers."

def send_director_message(topic: str, text: str):
    """
    Publishes a text message to a ROS topic monitored by a 'String' layer.
    Example: topic='/bob/status_log', text='System initialized'
    """
    if not _node:
        return "Error: Node not initialized."

    pub = _node.create_publisher(String, topic, 10)
    msg = String()
    msg.data = text
    pub.publish(msg)
    _node.destroy_publisher(pub)

    return f"Director: Published message to {topic}."

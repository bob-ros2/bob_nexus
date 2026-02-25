#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import yaml
import json

class NexusDispatcher(Node):
    def __init__(self):
        super().__init__('nexus_dispatcher')
        
        # Configuration
        self.declare_parameter('input_topic', 'llm_prompt')
        self.declare_parameter('target_entity', 'master/prime') # Relative to entities/
        self.declare_parameter('nexus_dir', '/app')
        
        self.input_topic = self.get_parameter('input_topic').value
        self.target_entity = self.get_parameter('target_entity').value
        self.nexus_dir = self.get_parameter('nexus_dir').value
        
        # Inbox Path
        self.inbox_path = os.path.join(self.nexus_dir, "entities", self.target_entity, "inbox.json")
        
        # Subscription
        self.sub = self.create_subscription(
            String,
            self.input_topic,
            self.topic_callback,
            10
        )
        
        self.get_logger().info(f"Dispatcher active: Listening on {self.input_topic} -> {self.inbox_path}")

    def topic_callback(self, msg):
        prompt = msg.data.strip()
        if not prompt:
            return

        # Simple filter to avoid processing the Heartbeat itself as a mission if it's too frequent
        if "HEARTBEAT" in prompt.upper() and len(prompt) < 100:
            self.get_logger().debug("Ignoring raw short heartbeat.")
            return

        mission = {
            "source": f"ros_topic:{self.input_topic}",
            "prompt": prompt,
            "type": "strategic_mission"
        }

        try:
            # Ensure target directory exists
            os.makedirs(os.path.dirname(self.inbox_path), exist_ok=True)
            
            # Atomic write (write to .tmp then rename)
            tmp_path = self.inbox_path + ".tmp"
            with open(tmp_path, 'w') as f:
                json.dump(mission, f, indent=2)
            os.rename(tmp_path, self.inbox_path)
            
            self.get_logger().info(f"Dispatched mission from {self.input_topic} to Prime inbox.")
        except Exception as e:
            self.get_logger().error(f"Failed to dispatch mission: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NexusDispatcher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

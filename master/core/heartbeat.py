#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SwarmHeartbeat(Node):
    def __init__(self):
        super().__init__('swarm_heartbeat')
        
        self.declare_parameter('period', 60.0) # Standard: 1 minute
        self.declare_parameter('target_topic', 'llm_prompt')
        self.declare_parameter('message', 'HEARTBEAT: Check your workspace for updates and proceed with your mission.')
        
        period = self.get_parameter('period').value
        self.target_topic = self.get_parameter('target_topic').value
        self.message = self.get_parameter('message').value
        
        self.pub = self.create_publisher(String, self.target_topic, 10)
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info(f"Heartbeat started: {period}s on {self.target_topic}")

    def timer_callback(self):
        msg = String()
        msg.data = self.message
        self.pub.publish(msg)
        self.get_logger().info(f"Sent heartbeat to {self.target_topic}")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmHeartbeat()
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

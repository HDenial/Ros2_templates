#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
     
     
class ListenerNode(Node):
    def __init__(self):
        super().__init__("robot_listener") 
        self.subscriber_= self.create_subscription(String, "News", self.listener_callback, 10) # create subscriber
        self.get_logger().info("Listener started")

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)
        #self.get_logger().info(f"Received: {msg.data}") shine
     
     
def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

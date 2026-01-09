#!/usr/bin/env python3
import rclpy
from rclpy.node import Node



class MyNode(Node):

    def __init__(self):
        super().__init__("Py_test") # Initialize the node with the name "Py_test"
        self.tempo = 0
        self.get_logger().info("Hello, world!") # Log a message to the console
        self.create_timer(1.0,self.timer_callback) # Create a timer that calls the timer_callback method every 1 second
    
    def timer_callback(self):
        self.get_logger().info(f"{self.tempo} seconds have passed since the node started") # Log the number of seconds that have passed since the node started
        self.tempo += 1
        

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 Python client library"
    node = MyNode() # Create an instance of the MyNode class
    rclpy.spin(node) # Keep the node running until it is shut down
    node.destroy_node() # Destroy the node
    rclpy.shutdown() # Shutdown the ROS 2 Python client library

if __name__ == '__main__': # Check if the script is being run directly
    main() # Call the main function
# This script defines a simple ROS 2 node that logs a message every second.
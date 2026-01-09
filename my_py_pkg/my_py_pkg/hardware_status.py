#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus 
     
     
class HardwareStatusPubNode(Node): 
    def __init__(self):
        super().__init__("hard_status")
        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10) 
        self.timer = self.create_timer(1, self.publish_hw_status)
        self.get_logger().info("Hardware Status Publisher Node has been started")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 27.6
        msg.are_motors_read = True
        msg.status = "OK"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")
     
def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPubNode()
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

# Dont forget to add entry point in setup.py before building the package

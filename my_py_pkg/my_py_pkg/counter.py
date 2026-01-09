#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import Trigger
     
     
class ListenerNode(Node): 
    def __init__(self):
        super().__init__("counter") 
        self.subscriber_= self.create_subscription(Int64, "Number", self.listener_callback, 10) # create subscriber to "Number" topic with Int64 type
        self.get_logger().info("Listener started")
        self.publisher_= self.create_publisher(Int64, "Counted_number", 10) # create publisher to "Counted_number" topic with Int64 type
        self.counter= 0
        self.get_logger().info("Publisher started")
        self.server_ = self.create_service(Trigger, "Counter_reset", self.reset_counter_callback)   # create service with Trigger type
        self.get_logger().info("Counter reset service started")
    
    def listener_callback(self, msg):
        self.counter+= msg.data  # increment counter by received message data attribute; ros2 interface show example_interfaces/msg/Int64 to see message structure
        self.get_logger().info(f"Received: {msg.data}")
        msg_to_publish = Int64()
        msg_to_publish.data = self.counter
        self.publisher_.publish(msg_to_publish) # call publisher to publish the message on the topic it refers to
        self.get_logger().info(f"Published: {msg_to_publish.data}")

    def reset_counter_callback(self, request: Trigger.Request, response: Trigger.Response): #try triggering the service with ros2 service call /Counter_reset example_interfaces/srv/Trigger "{}"
        self.counter = 0
        self.get_logger().info("Counter reset")
        response.success = True
        response.message = "Counter reset successfully"
        return response
     
def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

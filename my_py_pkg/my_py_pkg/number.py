#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

     
     
class Newstation(Node): 
    def __init__(self):
        super().__init__("number") # set node name
        self.publisher_= self.create_publisher(Int64, "Number", 10) # create publisher
        self.timer = self.create_timer(1, self.publish_news) # set timer between every new publish
        self.get_logger().info("Publisher started")

    def publish_news(self):
        msg = Int64() # create message and set its type
        msg.data = 1 # set message data
        self.publisher_.publish(msg) # publish only the message
        #self.get_logger().info(f"Publishing: {msg.data}") #publish message with shine
     
     
def main(args=None):
    rclpy.init(args=args)
    node = Newstation()
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

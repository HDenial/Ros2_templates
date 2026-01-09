#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

     
     
class Newstation(Node): 
    def __init__(self):
        super().__init__("robot_publisher") # set node name
        self.declare_parameter("robot_name", "Rob")
        self.robot_name = self.get_parameter("robot_name").value # get parameter value
        self.publisher_= self.create_publisher(String, "News", 10) # create publisher for strings on the /News topic
        self.timer = self.create_timer(0.5, self.publish_news) # set timer between every new publish
        self.get_logger().info("Publisher started")

    def publish_news(self):
        msg = String() # create message and set its type
        msg.data = f"Hello world, Im {self.robot_name}" # set message data
        self.publisher_.publish(msg) # publish the entire message
        #self.get_logger().info(f"Publishing: {msg.data}") #publish message with shine
     
     
def main(args=None):
    rclpy.init(args=args)
    node = Newstation()
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

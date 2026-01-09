#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from example_interfaces.msg import Int64

     
     
class Newstation(Node): 
    def __init__(self):
        super().__init__("number") # set node name
        self.declare_parameter("number", 1) #parameter declaration
        self.declare_parameter("timer_period", 1.0)
        self.number_ = self.get_parameter("number").value # get parameter value
        self.timer_period_ = self.get_parameter("timer_period").value
        self.publisher_= self.create_publisher(Int64, "Number", 10) # create publisher
        self.timer = self.create_timer(self.timer_period_, self.publish_news) # set timer between every new publish
        self.get_logger().info("Publisher started")
        self.add_on_set_parameters_callback(self.parameter_callback) # set callback for parameter change

    def publish_news(self):
        msg = Int64() # create message and set its type
        msg.data = self.number_ # set message data
        self.publisher_.publish(msg) # publish only the message
        self.get_logger().info(f"Publishing: {msg.data}") #publish message with shine
     
    def parameter_callback(self, params: list[Parameter]): # callback function for parameter change at runtime using set
        for param in params:
            if param.name == "number":
                self.number_ = param.value
                self.get_logger().info(f"Number parameter changed to: {self.number_}")
            elif param.name == "timer_period":
                self.timer_period_ = param.value
                self.get_logger().info(f"Timer period changed to: {self.timer_period_}")
            
                

        
     
def main(args=None):
    rclpy.init(args=args)
    node = Newstation()
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

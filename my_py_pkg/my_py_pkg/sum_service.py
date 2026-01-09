#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
     
     
class SumServerNode(Node): 
    def __init__(self):
        super().__init__("SumServer")
        self.server_ = self.create_service(AddTwoInts, "SumServer", self.add_two_ints_callback)
        self.get_logger().info("SumServer is ready to add two integers.")
    def add_two_ints_callback(self, request:AddTwoInts.Request, response: AddTwoInts.Response):
        self.get_logger().info(f"Adding {request.a} and {request.b}.")
        response.sum = request.a + request.b
        return response 
     
     
def main(args=None):
    rclpy.init(args=args)
    node = SumServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

# Dont forget to add entry point in setup.py
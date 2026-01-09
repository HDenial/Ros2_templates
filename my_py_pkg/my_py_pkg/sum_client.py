#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts     
     
class SumClientNode(Node):
    def __init__(self):
        super().__init__("SumClient")
        self.client_ = self.create_client(AddTwoInts, "SumServer") # Create a client for the service

    def ClientCall(self,a,b):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for server...")
        
        # Create a request object
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        
        self.get_logger().info(f"Sending request: {request.a} + {request.b}")
        future = self.client_.call_async(request) # Asynchronous call
        future.add_done_callback(self.Callback) # Callback function to handle the response

    def Callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Sum Result: {response.sum}") # Log the result
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

     
     
def main(args=None):
    rclpy.init(args=args)
    node = SumClientNode()
    node.ClientCall(5, 3) # Example call to the service with two integers 
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

# Dont forget to add entry point in setup.py
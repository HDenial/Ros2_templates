#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import BatteryCheck 
from my_robot_interfaces.msg import LedState    
     
class LedPanelNode(Node): 
    def __init__(self):
        super().__init__("led_panel") 
        self.led = LedState()
        self.declare_parameter("led_state", [False, False, False]) # initial LED states
        self.led.led_state = self.get_parameter("led_state").value
        self.server_= self.create_service(BatteryCheck, "set_led", self.led_panel_callback)
        self.get_logger().info("LED Control Service Node has been started")
        self.publisher = self.create_publisher(LedState, "led_status", 10)
        self.get_logger().info(f"Current LED states: {self.led.led_state}")
    
    def led_panel_callback(self, request, response):
        if request.led_id < 0 or request.led_id > 2:
            self.get_logger().error("Invalid LED ID")
            response.success = False
            return response
        else:
            if request.alert == True:
                self.led.led_state[request.led_id] = True
                self.get_logger().info(f"LED {request.led_id} is ON")
                response.success = True
            else:
                self.led.led_state[request.led_id] = False
                self.get_logger().info(f"LED {request.led_id} is OFF")
                response.success = True
        
        self.publisher.publish(self.led)
        self.get_logger().info(f"Current LED states: {self.led.led_state}")
        return response
        
     
def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() 
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

# Dont forget to add entry point in setup.py g
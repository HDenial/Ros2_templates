#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import BatteryCheck     
     
class BatteryStateNode(Node):
    def __init__(self):
        super().__init__("battery_state")
        self.charge = 5
        self.alert = False
        self.client_= self.create_client(BatteryCheck, "set_led")
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for LED service to be available...")
        self.timer = self.create_timer(1, self.discharge)
        self.reset_timer = None
        self.get_logger().info("Battery State Node has been started")
    
    
    def discharge(self):
        self.charge -= 1
        msg = BatteryCheck.Request()
        if self.charge <= 0:
            msg.alert = True
            msg.led_id = 2
            self.get_logger().info("Battery is low")
            self.charge = 0
            if self.reset_timer is None:
                self.reset_timer = self.create_timer(3, self.reset_charge)
        else:
            msg.alert = self.alert
            msg.led_id = 2
            self.get_logger().info(f"Battery charge: {self.charge}")    
        future= self.client_.call_async(msg)
        future.add_done_callback(self.callback)
    
    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("LED state updated successfully")
            else:
                self.get_logger().error("Failed to update LED state")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def reset_charge(self): #resets charge for testing
        self.charge = 5 
        self.reset_timer.cancel()
        self.reset_timer = None
        
def main(args=None):
    rclpy.init(args=args)
    node = BatteryStateNode() 
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

# Dont forget to add entry point in setup.py
#!/usr/bin/env python3
import rclpy 
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("IN Constructor")
        self.number_ = 1
        self.publish_frequency_ = 1.0
        self.number_publisher_ = None
        self.number_timer_ = None


    # Create Ros2 communication, connect to HW
    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_configure")
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10) #only publishes when active
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number)
        self.number_timer_.cancel() #stop timer until activated
        return TransitionCallbackReturn.SUCCESS
    
    # Destroy Ros2 communication, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_cleanup")
        self.destroy_timer(self.number_timer_)
        self.destroy_publisher(self.number_publisher_)
        return TransitionCallbackReturn.SUCCESS
    
    # Activate/enable HW, start publishing/reading
    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn: 
        self.get_logger().info("IN on_activate")
        self.number_timer_.reset() #restart timer
        return super().on_activate(previous_state)
    
    # Deactivate/disable HW, stop publishing/reading
    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_deactivate")
        self.number_timer_.cancel()
        return super().on_deactivate(previous_state)
    
    # Kill node, stop all communication
    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("IN on_shutdown")
        self.destroy_timer(self.number_timer_)
        self.destroy_publisher(self.number_publisher_)
        return TransitionCallbackReturn.SUCCESS  

    # Error occured, stop everything
    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_error")
        self.destroy_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS  #sucessful returns to previous state, failed shutsdown
        #trigger on_error calling TrasitionCallbackReturn.FAILURE or raise Exception()

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

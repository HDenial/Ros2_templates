#!/usr/bin/env python3
import rclpy
import rclpy.callback_groups
from rclpy.node import Node
import time
from my_robot_interfaces.action import CountUntil  
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.action import GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading 


class CountUntilServerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("count_until_server") # MODIFY NAME
        self.goal_handle: ServerGoalHandle = None  # Initialize goal handle
        self.goal_lock = threading.Lock()  # Lock for thread safety
        self.goal_queue = []  # Queue to hold goals
        self._count_until_server = ActionServer(self, CountUntil,
                                                 "count_until",
                                                 goal_callback=self.goal_callback,
                                                 handle_accepted_callback=self.queue_callback,
                                                 cancel_callback=self.cancel_callback,
                                                 execute_callback=self.execute_callback,
                                                 callback_group=ReentrantCallbackGroup())
        self.get_logger().info("CountUntil action server is ready.")

    def goal_callback(self, goal_request: CountUntil.Goal) -> GoalResponse:
        # Check if the goal request is valid
        self.get_logger().info(f"Received goal request: target_number={goal_request.target_number}, period={goal_request.period}")

        # Policy: refuse new goal if there is already an active goal /breaks queue system
        # with self.goal_lock:
        #     if self.goal_handle is not None and self.goal_handle.is_active:
        #         self.get_logger().warn("Goal request rejected: there is already an active goal.")
        #         return GoalResponse.REJECT

        # Validate the goal request
        if goal_request.target_number < 0  or goal_request.period <= 0:
            self.get_logger().error("Received invalid goal request.")
            return GoalResponse.REJECT
        
        # Policy: Always execute most recent  valid goal    / breaks queue system
        # with self.goal_lock:
        #     if self.goal_handle is not None and self.goal_handle.is_active:
        #         self.get_logger().info("There is an active goal, aborting it to accept the new goal.")
        #         self.goal_handle.abort()

        self.get_logger().info("Goal request is valid. Accepting goal.")
        return GoalResponse.ACCEPT 
    
    def queue_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock:  # Ensure thread safety
            if self.goal_handle is not None:
                self.get_logger().info("There is an active goal, adding new goal to the queue.")
                self.goal_queue.append(goal_handle)
                return
            else:
                self.get_logger().info("No active goal, executing the new goal immediately.")
                goal_handle.execute()  # Execute the goal immediately

    
    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        # Handle cancellation request
        self.get_logger().info("Received cancel request for goal.")
        self.get_logger().info("Goal is being canceled.")
        return CancelResponse.ACCEPT
        #return CancelResponse.REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock:  # Ensure thread safety
            self.goal_handle = goal_handle
        #get request data
        target_number=goal_handle.request.target_number
        period = goal_handle.request.period

        #execute the action
        self.get_logger().info(f"Counting from 0 to {target_number} with a period of {period} seconds.")
        result = CountUntil.Result()
        feedback = CountUntil.Feedback()
        counter = 0
        for i in range(target_number):

            if not goal_handle.is_active:
                self.get_logger().info("Goal is no longer active.")
                result.reached_number = counter
                self.process_goal_queue()  # Process the next goal in the queue
                return result
            
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal was canceled.")
                goal_handle.canceled()
                result.reached_number = counter
                self.process_goal_queue()  # Process the next goal in the queue
                return result
            
            if i < target_number:
                counter += 1

            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Count: {counter}")
            time.sleep(period)  # Simulate work by sleeping for the period

        goal_handle.succeed()  # Mark the goal as succeeded

        # Send result
        result.reached_number = counter
        self.process_goal_queue()  # Process the next goal in the queue
        return result
    
    def process_goal_queue(self):
        with self.goal_lock:
            if len(self.goal_queue) > 0:
                self.get_logger().info("Processing next goal from the queue.")
                self.goal_queue.pop(0).execute()
            else:
                self.goal_handle = None
        
     
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode() # MODIFY NAME
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()

# Dont forget to add entry point in setup.py
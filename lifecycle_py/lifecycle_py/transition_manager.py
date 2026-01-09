import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class TransitionManager(Node):
    def __init__(self):
        super().__init__("transition_manager")
        self.declare_parameter("managed_node_name", rclpy.Parameter.Type.STRING)
        node_name = self.get_parameter("managed_node_name").value
        service_change_state_name = "/" + node_name + "/change_state"
        self.client = self.create_client(ChangeState, service_change_state_name)

    def change_state(self, transition: Transition):
        self.client.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def initialization_sequence(self):
        #unconfigured to Inactive
        self.get_logger().info("Trying to configure")
        transition= Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "Configure"
        self.change_state(transition)
        self.get_logger().info("Configuring OK, now inactive")

        #sleep just as placeholder for other operations
        time.sleep(2)

        #Inactive to Active
        self.get_logger().info("Trying to activate")
        transition= Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "Activate"
        self.change_state(transition)
        self.get_logger().info("Activating OK, now active")



def main(args=None):
    rclpy.init(args=args)
    node = TransitionManager()
    node.initialization_sequence()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool

class MasterKeyboardNode(Node):
    def __init__(self):
        super().__init__('plc_keyboard_node')
        self.subscriber_ = self.create_subscription(String, 'key_pressed', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Bool, 'led_command', 10)
        self.get_logger().info("Listening for key presses on 'key_pressed' topic.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received key: {msg.data}")
        led_msg = Bool()
        if msg.data == 'h':  # allow if h is pressed
            led_msg.data = True
            self.get_logger().info("Allowed LED toggle command.")

        else:
            self.get_logger().warn("Key not allowed, ignoring.")
            led_msg.data = False
            self.get_logger().info("Ignored LED toggle command.")
        self.publisher_.publish(led_msg)


def main():
    rclpy.init()
    node = MasterKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

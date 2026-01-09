# plc_node_keyboard.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import termios
import tty
import select

class MasterKeyboardNode(Node):
    def __init__(self):
        super().__init__('plc_keyboard_node')
        self.publisher_ = self.create_publisher(Bool, 'led_command', 10)

        self.get_logger().info("Press 'h' to toggle LED, 'q' to quit.")

        # Setup stdin for non-blocking keyboard input
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Create timer to poll keyboard input
        self.timer = self.create_timer(0.1, self.key_loop)
        self.led_state = False

    def key_loop(self):
        if self.kbhit():
            key = sys.stdin.read(1)
            if key == 'h':
                self.led_state = not self.led_state
                msg = Bool()
                msg.data = self.led_state
                self.publisher_.publish(msg)
                self.get_logger().info(f"Sent LED {'ON' if self.led_state else 'OFF'}")
            elif key == 'q':
                self.get_logger().info("Quitting...")
                rclpy.shutdown()

    def kbhit(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        return dr != []

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


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

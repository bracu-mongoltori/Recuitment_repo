# keyboard_control.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Keyboard control started. Use W/A/S/D to move, Q to quit.")
        self.listen_to_keys()

    def get_key(self):
        """Non-blocking key reader (Linux only)"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def send_velocity(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent cmd: linear_x={linear_x}, angular_z={angular_z}")

    def listen_to_keys(self):
        try:
            while True:
                key = self.get_key()

                if key == 'w':
                    self.send_velocity(0.2, 0.0)  # Forward
                elif key == 's':
                    self.send_velocity(-0.2, 0.0)  # Backward
                elif key == 'a':
                    self.send_velocity(0.0, 0.5)  # Turn left
                elif key == 'd':
                    self.send_velocity(0.0, -0.5)  # Turn right
                elif key == 'x':
                    self.send_velocity(0.0, 0.0)  # Stop
                elif key == 'q':
                    self.send_velocity(0.0, 0.0)
                    break
        except KeyboardInterrupt:
            self.send_velocity(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


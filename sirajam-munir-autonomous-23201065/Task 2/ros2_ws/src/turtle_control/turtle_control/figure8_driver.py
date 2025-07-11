# turtle_control/figure8_driver.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.parameter import Parameter
import math

class Figure8Driver(Node):
    def __init__(self):
        super().__init__('figure8_driver')

        self.declare_parameter('pattern_speed', 1.0)
        self.pattern_speed = self.get_parameter('pattern_speed').value

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.timer = self.create_timer(0.1, self.move_in_figure8)  # 10 Hz for motion
        self.pose_log_timer = self.create_timer(1.0, self.log_pose)  # 1 Hz for logging

        self.current_pose = Pose()

    def pose_callback(self, msg):
        self.current_pose = msg

    def log_pose(self):
        self.get_logger().info(f'Pose: x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}, theta={self.current_pose.theta:.2f}')

    def move_in_figure8(self):
        twist = Twist()
        t = self.get_clock().now().nanoseconds / 1e9
        twist.linear.x = self.pattern_speed
        twist.angular.z = 2.0 * math.sin(t)
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Figure8Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

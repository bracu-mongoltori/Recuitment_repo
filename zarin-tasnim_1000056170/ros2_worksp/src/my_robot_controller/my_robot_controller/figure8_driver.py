#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
import math

class FigureEightDriver(Node):
    def __init__(self):
        super().__init__('figure8_driver')

        # Parameter for speed
        self.declare_parameter(
            'pattern_speed',
            1.0,
            ParameterDescriptor(description='Speed multiplier for the figure-eight pattern')
        )
        self.speed = self.get_parameter('pattern_speed').get_parameter_value().double_value

        # ROS pub/sub
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timers
        self.timer = self.create_timer(0.05, self.move_turtle)  # 20Hz control
        self.log_timer = self.create_timer(1.0, self.log_pose)

        self.pose = None
        self.t = 0.0
        self.dt = 0.05
        self.A = 4.0  # width of figure-eight
        self.B = 4.0  # height of figure-eight

    def pose_callback(self, msg):
        self.pose = msg

    def move_turtle(self):
        if self.pose is None:
            return

        # Target point on figure-eight, centered at (5.5, 5.5)
        target_x = 5.5 + self.A * math.sin(self.t)
        target_y = 5.5 + self.B * math.sin(self.t) * math.cos(self.t)

        # Compute error to target
        error_x = target_x - self.pose.x
        error_y = target_y - self.pose.y

        # Distance and heading to target
        distance = math.sqrt(error_x ** 2 + error_y ** 2)
        angle_to_target = math.atan2(error_y, error_x)

        # Heading error (difference between where turtle is facing vs needs to face)
        heading_error = self.normalize_angle(angle_to_target - self.pose.theta)

        # Control
        msg = Twist()
        msg.linear.x = self.speed * distance
        msg.angular.z = 4.0 * heading_error  # turn sharper if needed

        self.cmd_vel_pub.publish(msg)

        # Move forward in time
        self.t += self.dt * self.speed
        if self.t > 2 * math.pi:
            self.t = 0.0

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def log_pose(self):
        if self.pose:
            self.get_logger().info(
                f"Pose - x: {self.pose.x:.2f}, y: {self.pose.y:.2f}, θ: {self.pose.theta:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

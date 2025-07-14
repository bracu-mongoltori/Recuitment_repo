import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Figure8Driver(Node):
    def __init__(self):
        super().__init__('figure8_driver')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.05, self.move_callback)  # 20 Hz for smooth movement
        self.log_timer = self.create_timer(1.0, self.log_pose)    # 1 Hz for logging
        self.pattern_speed = self.declare_parameter('pattern_speed', 1.0).get_parameter_value().double_value
        self.time = 0.0
        self.current_pose = None

    def move_callback(self):
        # Figure-eight trajectory using sine for angular velocity
        msg = Twist()
        msg.linear.x = self.pattern_speed
        msg.angular.z = 2.0 * self.pattern_speed * math.sin(self.time)
        self.publisher_.publish(msg)
        self.time += 0.05

    def pose_callback(self, msg):
        self.current_pose = msg

    def log_pose(self):
        if self.current_pose:
            self.get_logger().info(
                f"Pose: x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}, theta={self.current_pose.theta:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = Figure8Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

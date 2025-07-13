import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

class Figure8Driver(Node):
    def __init__(self):
        super().__init__('figure8_driver')

        # Declare ROS2 parameter
        self.declare_parameter('pattern_speed', 2.0)
        self.speed = self.get_parameter('pattern_speed').get_parameter_value().double_value

        # Publisher for turtle movement
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber to turtle pose
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer for movement (10 Hz for smooth curve)
        self.timer = self.create_timer(0.1, self.drive_loop)

        # Separate timer for logging pose (1 Hz)
        self.pose_timer = self.create_timer(1.0, self.log_pose)

        self.start_time = time.time()
        self.current_pose = None

        self.get_logger().info('Figure8Driver node started.')

    def drive_loop(self):
        msg = Twist()
        msg.linear.x = self.speed

        # Alternate direction every 4 seconds to create a figure-eight
        if int(time.time() - self.start_time) % 8 < 4:
            msg.angular.z = 1.5
        else:
            msg.angular.z = -1.5

        self.publisher_.publish(msg)

    def pose_callback(self, msg):
        self.current_pose = msg  # store latest pose

    def log_pose(self):
        if self.current_pose:
            x = self.current_pose.x
            y = self.current_pose.y
            theta = self.current_pose.theta
            self.get_logger().info(f'Pose -> x: {x:.2f}, y: {y:.2f}, θ: {theta:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math

class Figure8Driver(Node):
    def __init__(self):
        super().__init__('figure8_driver')

        # ROS2 Parameters
        self.declare_parameter('pattern_speed', 2.0)
        self.speed = self.get_parameter('pattern_speed').get_parameter_value().double_value
        self.declare_parameter('angular_speed_multiplier', 0.8)
        self.angular_speed_multiplier = self.get_parameter('angular_speed_multiplier').get_parameter_value().double_value

        # Publishers and Subscribers
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timers
        self.timer = self.create_timer(0.05, self.drive_loop) # Higher frequency for smoother control
        self.pose_timer = self.create_timer(1.0, self.log_pose)

        self.current_pose = None
       
        # The state machine starts directly with the first turn
        self.state = 'turn_left'
        self.state_start_time = time.time()

        self.get_logger().info('Figure8Driver node started. Initiating figure-8 pattern immediately.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def drive_loop(self):
        if self.current_pose is None:
            # Wait until the first pose message is received
            return

        msg = Twist()
        msg.linear.x = self.speed

        # State machine for figure 8
        if self.state == 'turn_left':
            msg.angular.z = self.speed * self.angular_speed_multiplier
            # Check if one full circle is complete
            if time.time() - self.state_start_time > (2 * math.pi / (self.speed * self.angular_speed_multiplier)):
                 self.state = 'turn_right'
                 self.get_logger().info('State change: turn_right')
                 self.state_start_time = time.time()

        elif self.state == 'turn_right':
            msg.angular.z = -self.speed * self.angular_speed_multiplier
            # Check if the second full circle is complete
            if time.time() - self.state_start_time > (2 * math.pi / (self.speed * self.angular_speed_multiplier)):
                self.state = 'turn_left'  # Loop back to 'turn_left'
                self.get_logger().info('State change: turn_left')
                self.state_start_time = time.time()

        self.publisher_.publish(msg)

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


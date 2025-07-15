import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim.srv import SetPen
import time
import math

class Figure8Driver(Node):
    def __init__(self):
        super().__init__('figure8_driver')

        # Parameters
        self.declare_parameter('pattern_speed', 2.0)
        self.speed = self.get_parameter('pattern_speed').get_parameter_value().double_value
        self.declare_parameter('angular_speed_multiplier', 0.8)
        self.angular_speed_multiplier = self.get_parameter('angular_speed_multiplier').get_parameter_value().double_value

        # Publishers and Subscribers
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timers
        self.timer = self.create_timer(0.05, self.drive_loop)  # smooth control
        self.pose_timer = self.create_timer(1.0, self.log_pose)

        # Service to toggle trace
        self.create_service(Empty, '/toggle_trace', self.toggle_trace_callback)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.pen_on = True

        self.current_pose = None

        # State machine
        self.state = 'turn_left'
        self.state_start_time = time.time()

        self.get_logger().info('🐢 Figure8Driver started. Beginning figure-eight motion.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def drive_loop(self):
        if self.current_pose is None:
            return

        msg = Twist()
        msg.linear.x = self.speed

        if self.state == 'turn_left':
            msg.angular.z = self.speed * self.angular_speed_multiplier
            if time.time() - self.state_start_time > (2 * math.pi / (self.speed * self.angular_speed_multiplier)):
                self.state = 'turn_right'
                self.state_start_time = time.time()
                self.get_logger().info('↪️ Switching to turn_right')

        elif self.state == 'turn_right':
            msg.angular.z = -self.speed * self.angular_speed_multiplier
            if time.time() - self.state_start_time > (2 * math.pi / (self.speed * self.angular_speed_multiplier)):
                self.state = 'finished'
                self.state_start_time = time.time()
                self.get_logger().info('✅ Figure-8 complete, restarting...')

        elif self.state == 'finished':
            # Instead of stopping, restart the loop
            self.state = 'turn_left'
            self.state_start_time = time.time()
            return

        self.publisher_.publish(msg)

    def log_pose(self):
        if self.current_pose:
            x = self.current_pose.x
            y = self.current_pose.y
            theta = self.current_pose.theta
            self.get_logger().info(f'📍 Pose -> x: {x:.2f}, y: {y:.2f}, θ: {theta:.2f}')

    def toggle_trace_callback(self, request, response):
        self.pen_on = not self.pen_on

        if not self.pen_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ /turtle1/set_pen service not available')
            return response

        req = SetPen.Request()
        req.r = 255
        req.g = 0
        req.b = 0
        req.width = 2
        req.off = int(not self.pen_on)

        self.pen_client.call_async(req)
        self.get_logger().info(f'🖋️ Pen toggled to {"ON" if self.pen_on else "OFF"}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = Figure8Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


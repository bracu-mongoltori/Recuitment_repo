import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool    
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist

class TraceToggleService(Node):
    def __init__(self):
        super().__init__('trace_toggle_service')    # Initializing

        # Create service to toggle trace
        self.srv = self.create_service(SetBool, 'toggle_trace', self.toggle_trace_callback)

        # Create a client for the turtlesim pen service
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')

        # Create publisher for turtle movement
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.get_logger().info('TraceToggleService node ready.')

    def toggle_trace_callback(self, request, response):
        # Prepare request to /turtle1/set_pen
        pen_req = SetPen.Request()
        pen_req.r = 0
        pen_req.g = 0
        pen_req.b = 0
        pen_req.width = 2
        pen_req.off = 0 if request.data else 1  # 0 = draw, 1 = no draw

        # Call the pen service
        self.pen_client.call_async(pen_req)

        # Move the turtle forward
        msg = Twist()
        msg.linear.x = 2.0  # Move forward
        self.publisher_.publish(msg)

        response.success = True
        response.message = 'Pen turned ' + ('ON' if request.data else 'OFF') + ' and turtle is moving.'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TraceToggleService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# turtle_control/trace_toggle.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from turtlesim.srv import SetPen

class TraceToggleService(Node):
    def __init__(self):
        super().__init__('trace_toggle_service')

        # Create a client to the /turtle1/set_pen service
        self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        
        # Wait until the service becomes available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')

        # Create our custom service
        self.create_service(SetBool, '/toggle_trace', self.toggle_callback)
        self.get_logger().info('Service /toggle_trace ready.')

    def toggle_callback(self, request, response):
        # Create the SetPen request
        pen_req = SetPen.Request()
        pen_req.r = 255
        pen_req.g = 0
        pen_req.b = 0
        pen_req.width = 2
        pen_req.off = not request.data  # True if you want pen OFF

        # Call the SetPen service asynchronously
        future = self.cli.call_async(pen_req)

        # Optional: give it a brief spin to ensure the call gets processed
        rclpy.spin_once(self, timeout_sec=0.1)

        # Set the response
        response.success = True
        response.message = f"Pen turned {'ON' if request.data else 'OFF'}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TraceToggleService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

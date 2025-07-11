# turtle_control/trace_toggle.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from turtlesim.srv import SetPen

class TraceToggleService(Node):
    def __init__(self):
        super().__init__('trace_toggle_service')
        self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        self.create_service(SetBool, '/toggle_trace', self.toggle_callback)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')

        self.get_logger().info('Service /toggle_trace ready.')

    def toggle_callback(self, request, response):
        pen_state = request.data
        req = SetPen.Request()
        req.r = 255
        req.g = 0
        req.b = 0
        req.width = 2
        req.off = not pen_state
        self.cli.call_async(req)
        response.success = True
        response.message = "Pen turned " + ("ON" if pen_state else "OFF")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TraceToggleService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from turtlesim.srv import SetPen

class TraceToggleService(Node):
    def __init__(self):
        super().__init__('trace_toggle')

        self.srv = self.create_service(SetBool, '/toggle_trace', self.toggle_trace_callback)

        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')

        self.get_logger().info('TraceToggleService is ready.')

    def toggle_trace_callback(self, request, response):
        self.get_logger().info(f"Received toggle request: {request.data}")
        pen_req = SetPen.Request()

        if request.data:
            pen_req.r = 55
            pen_req.g = 0
            pen_req.b = 62
            pen_req.width = 2
            pen_req.off = 0
            self.get_logger().info("Turning pen ON")
        else:
            pen_req.r = 0
            pen_req.g = 0
            pen_req.b = 0
            pen_req.width = 2
            pen_req.off = 1
            self.get_logger().info("Turning pen OFF")

        future = self.pen_client.call_async(pen_req)

        timeout_time = self.get_clock().now().seconds_nanoseconds()[0] + 2.0
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now > timeout_time:
                self.get_logger().error("Timed out waiting for /set_pen service")
                response.success = False
                response.message = "SetPen service timeout"
                return response

        if future.result() is not None:
            response.success = False
            response.message = "SetPen service call raised an exception"
        else:
            response.success = True
            response.message = "Pen turned ON" if request.data else "Pen turned OFF"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = TraceToggleService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

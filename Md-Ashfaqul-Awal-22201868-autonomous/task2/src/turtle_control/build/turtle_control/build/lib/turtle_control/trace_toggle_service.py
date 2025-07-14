import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from turtlesim.srv import SetPen

class TraceToggleService(Node):
    def __init__(self):
        super().__init__('trace_toggle_service')
        self.srv = self.create_service(SetBool, '/toggle_trace', self.toggle_trace_callback)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

    def toggle_trace_callback(self, request, response):
        # Wait for the pen service to be available
        if not self.pen_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "Pen service unavailable"
            return response

        # Set pen: off (r=0,g=0,b=0,width=0,off=1), on (off=0)
        pen_req = SetPen.Request()
        pen_req.r = 0
        pen_req.g = 0
        pen_req.b = 0
        pen_req.width = 2
        pen_req.off = not request.data

        future = self.pen_client.call_async(pen_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is not None:
            response.success = True
            response.message = "Pen toggled " + ("on" if request.data else "off")
        else:
            response.success = False
            response.message = "Failed to call set_pen"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TraceToggleService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sindeed_alam_autonomous.algorithms import astar, dijkstra
from sindeed_alam_autonomous.maps import get_waypoints, get_start_goal
from sindeed_alam_autonomous.visualizer import plot_grid

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        grid = get_waypoints()
        start, goal = get_start_goal()

        for name, func in [("Dijkstra", dijkstra), ("A*", astar)]:
            path, explored = func(grid, start, goal)
            self.get_logger().info(f"{name} path length: {len(path)}")
            plot_grid(grid, path, explored, start, goal, title=f"{name} Path")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

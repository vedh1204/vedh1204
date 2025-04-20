import rclpy
import heapq
import math
import numpy as np  # Added numpy import
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationError
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.timer import Timer
from rclpy.qos import QoSProfile

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        self.grid = None
        self.start = None
        self.goal = None

        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        self.goal_subscription = self.create_subscription( 
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.navigator = BasicNavigator()

    def goal_callback(self, msg):  
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"New goal received: {self.goal}")
        if self.grid is not None and self.start is not None:
            self.compute_path()

    def occupancy_grid_to_numpy(self, grid_msg):
        """Convert occupancy grid to numpy array with proper reshaping"""
        return np.array(grid_msg.data).reshape(
            (grid_msg.info.height, grid_msg.info.width)
        )

    def astar(self, grid, start, goal):
       
        if grid is None:
            return []
            
        rows, cols = grid.shape  # Use numpy shape
        open_set = [(0 + self.heuristic(start, goal), 0, start, [start])]
        visited = set()

        while open_set:
            f, g, current, path = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                return path

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    # Use numpy array indexing
                    if grid[neighbor[0], neighbor[1]] == 0 and neighbor not in visited:
                        heapq.heappush(open_set, (
                            g + 1 + self.heuristic(neighbor, goal),
                            g + 1,
                            neighbor,
                            path + [neighbor]
                        ))
        return []


    def map_callback(self, msg):
        """Updated map processing"""
        self.grid = self.occupancy_grid_to_numpy(msg)
        if self.grid is not None and self.start is not None and self.goal is not None:
            self.compute_path()


def main(args=None):
    rclpy.init(args=args)
    planner = AStarPlanner()
    rclpy.spin(planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



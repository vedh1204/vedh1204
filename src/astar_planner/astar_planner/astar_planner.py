import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import heapq

# Your A* heuristic and pathfinding code
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan

def astar(grid, start, goal):
    rows, cols = grid.shape
    open_set = [(0 + heuristic(start, goal), 0, start, [start])]
    visited = set()

    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return path

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor] == 0 and neighbor not in visited:
                    heapq.heappush(open_set, (
                        g+1+heuristic(neighbor, goal),
                        g+1,
                        neighbor,
                        path + [neighbor]
                    ))
    return []

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.grid = None  # Initialize grid to None or appropriate value
        self.start = None
        self.goal = None

        # Subscription to receive map data
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        # Subscription to receive goal pose data
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        # Publisher to send path
        self.path_publisher = self.create_publisher(
            String,  # Change this to a more suitable message type for path
            'planned_path',
            10
        )

    def map_callback(self, msg):
        # Convert the map data into a grid representation
        self.grid = self.convert_to_grid(msg)

    def goal_callback(self, msg):
        # Extract goal position from the PoseStamped message
        goal = (msg.pose.position.x, msg.pose.position.y)

        if self.grid is not None:
            start = self.start  # Use the robot's current position (you may get it from `/odom`)
            path = astar(self.grid, start, goal)

            # Publish the path
            self.publish_path(path)

    def convert_to_grid(self, map_msg):
        # Convert map message to a 2D numpy array (grid)
        # Here’s an example conversion (adjust as needed)
        import numpy as np
        data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        return data

    def publish_path(self, path):
        # Publish the planned path to the path_publisher
        path_msg = String()  # Update this with the correct message type for paths
        path_msg.data = str(path)  # For testing, you can send the path as a string
        self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




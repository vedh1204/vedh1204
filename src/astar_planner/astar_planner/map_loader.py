import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapLoader(Node):
    def __init__(self, planner_node):
        super().__init__('map_loader')
        self.planner_node = planner_node
        self.map_data = None

        # Subscription to map data
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        self.get_logger().info("Received map data")
        self.map_data = self.convert_to_grid(msg)
        self.planner_node.astar_planner.set_grid(self.map_data)

    def convert_to_grid(self, map_msg):
        # Convert the OccupancyGrid into a 2D numpy grid (occupancy map)
        data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        return data




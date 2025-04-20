#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('astar_path_planner')
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.timer = self.create_timer(2.0, self.publish_path)

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        # Dummy hardcoded path (replace with actual A* output)
        for i in range(10):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = i * 0.5
            pose.pose.position.y = i * 0.2
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)
        self.get_logger().info('Published path!')

def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


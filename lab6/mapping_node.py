#!/usr/bin/env python3
################################################################################
#
# ROS 2 Node to wrap the OccupancyGrid2d class
#
################################################################################

import rclpy
from mapping.occupancy_grid_2d import OccupancyGrid2d


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGrid2d()
    if not node._initialized:
        node.get_logger().error("Failed to initialize the mapping node.")
        return
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
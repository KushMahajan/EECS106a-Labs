# ROS Libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped 
from tf2_ros import Buffer, TransformListener
import numpy as np

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.cube_sub = self.create_subscription(PointStamped, '/cube_pose', self.cube_callback, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rclpy.spin_once(self, timeout_sec=2)

        self.cube_pose = None

    def cube_callback(self, cube_pose):
        if self.cube_pose is None:
            self.cube_pose = self.transform_cube_pose(cube_pose)
            self.get_logger().info('Received cube pose')

    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frame
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            PointStamped: point in base_link_frame in form [x, y, z]
        """

        # ------------------------
        #TODO: Add your code here!
        # ------------------------

        return

def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

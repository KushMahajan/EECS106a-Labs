# ROS Libraries
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import ColorRGBA


class TablePlane(Node):
    def __init__(self):
        super().__init__('table_plane_demo')
        self.z_values = None

        self.marker_pub = self.create_publisher(Marker, '/ground_plane_marker', 1)

        # Declare parameters for plane coefficients
        self.declare_parameter('plane.a', 1.00)
        self.declare_parameter('plane.b', 0.0)
        self.declare_parameter('plane.c', 1.0)
        self.declare_parameter('plane.d', 0.0)

        self.update_plane_coefficients()

        # Timer to check for coefficient changes every 0.5s
        self.create_timer(0.01, self.update_plane_coefficients)

    def update_plane_coefficients(self):
        self.a = self.get_parameter('plane.a').get_parameter_value().double_value
        self.b = self.get_parameter('plane.b').get_parameter_value().double_value
        self.c = self.get_parameter('plane.c').get_parameter_value().double_value
        self.d = self.get_parameter('plane.d').get_parameter_value().double_value
        self.plane_normal = np.sqrt(self.a ** 2 + self.b ** 2 + self.c ** 2)

        self.get_logger().info(f'Updated plane coefficients: a={self.a:.3f}, b={self.b:.3f}, c={self.c:.3f}, d={self.d:.3f}')
        self.publish_ground_plane_marker()

    def publish_ground_plane_marker(self):
        marker = Marker()
        marker.header.frame_id = "camera_depth_optical_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ground_plane"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        center = np.array([0.0, 0.0, -self.d / self.c])
        normal = np.array([self.a, self.b, self.c])
        normal = normal / np.linalg.norm(normal)
        z_axis = np.array([0, 0, 1])
        rotation_vector = np.cross(z_axis, normal)
        angle = np.arccos(np.clip(np.dot(z_axis, normal), -1.0, 1.0))

        if np.linalg.norm(rotation_vector) < 1e-6:
            quat = R.from_rotvec([0, 0, 0]).as_quat()
        else:
            quat = R.from_rotvec(rotation_vector / np.linalg.norm(rotation_vector) * angle).as_quat()

        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        marker.scale.x = 20.0
        marker.scale.y = 10.0
        marker.scale.z = 0.01

        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
        self.marker_pub.publish(marker)

        def update_plane_coefficients(self):
            self.a = self.get_parameter('plane.a').get_parameter_value().double_value
            self.b = self.get_parameter('plane.b').get_parameter_value().double_value
            self.c = self.get_parameter('plane.c').get_parameter_value().double_value
            self.d = self.get_parameter('plane.d').get_parameter_value().double_value

            self.get_logger().info(f'Updated plane coefficients: a={self.a:.3f}, b={self.b:.3f}, c={self.c:.3f}, d={self.d:.3f}')
            self.publish_ground_plane_marker()


def main(args=None):
    rclpy.init(args=args)
    node = TablePlane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
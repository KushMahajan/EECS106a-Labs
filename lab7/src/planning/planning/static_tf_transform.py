#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R

import numpy as np

class ConstantTransformPublisher(Node):
    def __init__(self):
        super().__init__('constant_tf_publisher')
        self.br = StaticTransformBroadcaster(self)

        # Homogeneous transform G_ar->base
        G = np.array([
            [-1, 0, 0, 0.0],
            [ 0, 0, 1, 0.16],
            [ 0, 1, 0, -0.13],
            [ 0, 0, 0, 1.0]
        ])

        # Create TransformStamped

        # ---------------------------
        # TODO: Fill out TransformStamped message
        # --------------------------
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'ar_marker_6'
        static_tf.child_frame_id = 'base_link'

        Q = R.from_matrix(G[0:3, 0:3]).as_quat()
        static_tf.transform.rotation.x = Q[0]
        static_tf.transform.rotation.y = Q[1]
        static_tf.transform.rotation.z = Q[2]
        static_tf.transform.rotation.w = Q[3]
        static_tf.transform.translation.x = G[0, 3]
        static_tf.transform.translation.y = G[1, 3]
        static_tf.transform.translation.z = G[2, 3]

        self.transform = static_tf

        self.timer = self.create_timer(0.05, self.broadcast_tf)

    def broadcast_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)

def main():
    rclpy.init()
    node = ConstantTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
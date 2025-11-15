import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped 
import tf2_geometry_msgs


class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscription
        self.cube_pose_sub = self.create_subscription(
            PointStamped,
            '/cube_pose',
            self.cube_pose_callback,
            10
        )

        # Publisher
        self.cube_pose_pub = self.create_publisher(
            PointStamped,
            '/cube_pose_base_link',
            10
        )

        self.get_logger().info("transform_cube_pose node started.")

    def cube_pose_callback(self, msg: PointStamped):
        transformed = self.transform_cube_pose(msg)
        if transformed is not None:
            self.cube_pose_pub.publish(transformed)


    def transform_cube_pose(self, msg: PointStamped):
        """
        Transform point from camera frame to base_link frame.
        """
        try:
            # Make sure the timestamp is valid
            msg.header.stamp = self.get_clock().now().to_msg()

            # TF lookup
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,   # use the actual incoming frame!
                rclpy.time.Time()
            )

            # Transform
            transformed_point = tf2_geometry_msgs.do_transform_point(msg, transform)

            # Update header
            transformed_point.header.stamp = self.get_clock().now().to_msg()
            transformed_point.header.frame_id = 'base_link'

            return transformed_point

        except Exception as e:
            self.get_logger().error(f"Could not transform cube pose: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TransformCubePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

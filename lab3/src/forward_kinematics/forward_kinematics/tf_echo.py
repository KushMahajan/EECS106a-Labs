import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import sys
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TFEcho(Node):
    def __init__(self, target, source):
        super().__init__('tf_echo_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target, self.source = target, source
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('tf_echo_node started')
        self.get_logger().info(f'listening to transforms between {self.source} and {self.target}')
    

    def timer_callback(self):
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(self.target, self.source, rclpy.time.Time())
            tr = t.transform.translation
            q = t.transform.rotation
            self.get_logger().info(f'Transform from {self.source} to {self.target}:\nTranslation: ({tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f})\nQuaternion: ({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info(f'could not find transform between {self.source} and {self.target}')
            pass
    
def main():
    rclpy.init()
    if len(sys.argv) != 3:
        print('usage: ros2 run forward_kinematics tf_echo.py <target_frame> <source_frame>')
        return
    node = TFEcho(sys.argv[1], sys.argv[2])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
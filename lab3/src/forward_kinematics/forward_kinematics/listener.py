import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .forward_kinematics import ur7e_forward_kinematics_from_joint_state

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')

        self.sub = self.create_subscription(JointState, '/joint_states', self.__joint_state_callback, 10)

    def __joint_state_callback(self, message):
        gst=ur7e_forward_kinematics_from_joint_state(message)
        np.set_printoptions(precision=4, suppress=True)
        self.get_logger().info(f'\n{gst}')



def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
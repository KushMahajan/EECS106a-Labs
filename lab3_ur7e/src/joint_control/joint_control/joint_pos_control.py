#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import tty
import termios
import threading
import select
import time

class JointPos(Node):
    def __init__(self, angles):
        super().__init__('joint_pos_controller')

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
            'wrist_2_joint', 'wrist_3_joint'
        ]

        self.angles = angles
        self.pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.5, self.once)
        self.sent = False

    def once(self):
        if not self.sent:
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = self.angles
            point.velocities = [0.0]*6
            point.time_from_start.sec = 3
            traj.points.append(point)
            self.pub.publish(traj)
            self.get_logger().info(f"Sent initial joint positions: {self.angles}")
            self.sent = True
        

def main():
    rclpy.init()
    if len(sys.argv) != 7:
        print("Usage: keyboard_controller.py <joint1> <joint2> <joint3> <joint4> <joint5> <joint6>")
        return
    angles = [float(arg) for arg in sys.argv[1:7]]
    node = JointPos(angles)
    time.sleep(3)
    start_time = time.time()
    wait_duration = 3
    while rclpy.ok() and (time.time() - start_time) < wait_duration:
        rclpy.spin_once(node)
    node.get_logger().info("Initial joint positions set. Exiting.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

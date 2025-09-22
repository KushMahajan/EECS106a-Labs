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

# Key mappings
INCREMENT_KEYS = ['1','2','3','4','5','6']
DECREMENT_KEYS = ['q','w','e','r','t','y']
JOINT_STEP = 0.065 # radians per key press

class KeyboardController(Node):
    def __init__(self):
        super().__init__('ur7e_keyboard_controller')
        
        self.joint_names = [
            'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
            'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint'
        ]
        self.joint_positions = [0.0]*6
        self.got_joint_states = False  # Failsafe: don't publish until joint states received
        
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        
        self.running = True
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
        self.got_joint_states = True

    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            print("Keyboard controller running. Increment: 123456 | Decrement: qwerty | Ctrl+C to exit")
            while self.running:
                try:
                    if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        if key == '\x03':
                            return
                        self.handle_key(key)
                except KeyboardInterrupt:
                    self.running = False
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def handle_key(self, key):
        if not self.got_joint_states:
            print("Waiting for joint states...")
            return
        
        new_positions = self.joint_positions.copy()
        if key in INCREMENT_KEYS:
            idx = INCREMENT_KEYS.index(key)
            new_positions[idx] += JOINT_STEP
        elif key in DECREMENT_KEYS:
            idx = DECREMENT_KEYS.index(key)
            new_positions[idx] -= JOINT_STEP
        elif key == '\x03':
            return
        
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = new_positions
        point.velocities = [0.0]*6
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)
        
        self.joint_positions = new_positions

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.running = False
        print("\nExiting keyboard controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

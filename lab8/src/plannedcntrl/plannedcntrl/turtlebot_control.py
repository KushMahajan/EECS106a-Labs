#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import transforms3d.euler as euler
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, PointStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from plannedcntrl.trajectory import plan_curved_trajectory  # Your existing Bezier planner
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Publisher and TF setup
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Controller gains
        self.Kp = np.diag([.8, 0.1])
        self.Kd = np.diag([-0.01, -0.01])
        self.Ki = np.diag([0.0, 0.0])

        self.error_integral = np.array([0.0,
                                        0.0])
        self.prev_error = np.array([0.0,
                                    0.0])

        # Subscriber
        self.create_subscription(PointStamped, '/goal_point', self.planning_callback, 10)

        self.get_logger().info('TurtleBot controller node initialized.')

    # ------------------------------------------------------------------
    # Main waypoint controller
    # ----------------, axis=1--------------------------------------------------
    def controller(self, waypoint):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # TODO: Transform the waypoint from the odo8m/world frame into the robot's base_link frame 
            # before computing errors — you'll need this so x_err and yaw_err are in the robot's coordinate system.
            # waypoint_stamped = PointStamped()
            # waypoint_stamped.header.frame_id = 'odom'
            # waypoint_stamped.point.x = waypoint[0]
            # waypoint_stamped.point.y = waypoint[1]
            # waypoint_stamped.point.z = 0.0
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
                # transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                # transformed_waypoint = self.tf_buffer.transform(waypoint_stamped, 'base_link')
                
                # Get the waypoint coordinates in the 'base_link' frame
                # waypoint = (transformed_waypoint.point.x, transformed_waypoint.point.y)
            except:
                self.get_logger().warn('TF lookup failed, retreading - yaying...')
                time.sleep(0.1)
                continue

            print(f"waipoint: x={waypoint[0]:.2f}, y={waypoint[1]:.2f}, yaw={waypoint[2]:.2f}")
            
            x_goal, y_goal, yaw_goal = waypoint[0], waypoint[1], waypoint[2]
            goal_target = PoseStamped()
            goal_target.header.frame_id = 'odom'
            goal_target.pose.position.x = x_goal
            goal_target.pose.position.y = y_goal
            goal_target.pose.position.z = 0.08

            qx, qy, qz, qw = self._quat_from_yaw(yaw_goal)
            goal_target.pose.orientation.x = qx
            goal_target.pose.orientation.y = qy
            goal_target.pose.orientation.z = qz
            goal_target.pose.orientation.w = qw


            goal_base = do_transform_pose(goal_target.pose, trans)

            print(f"Transformed Goal in base_link frame: x={goal_base.position.x:.2f}, y={goal_base.position.y:.2f}")

            x_err = goal_base.position.x
            y_err = goal_base.position.y
            q = goal_base.orientation
            _, _, yaw_err = euler.quat2euler([q.w, q.x, q.y, q.z])

            if abs(x_err) < 0.03 and abs(yaw_err) < 0.2:
                self.get_logger().info("Waypoint reached, moving to next.")
                return

            print(f"Errors: x_err={x_err:.2f}, y_err={y_err:.2f}, yaw_err={yaw_err:.2f}")

            # x_err = transform.tran            x_goal, y_goal, yaw_goal = waypoint[0], waypoint[1], waypoint[2]sform.translation.x - waypoint[0]
            # y_err = transform.transform.translation.y - waypoint[1]
            # q = transform.transform.rotation
            # _, _, yaw_err = euler.quat2euler([q.w, q.x, q.y, q.z])
            # yaw_err = yaw_err - waypoint[2]
            # print(f"Current pos: ({transform.transform.translation.x:.2f}, {transform.transform.translation.y:.2f}), yaw: {yaw_err:.2f}")
            # print(f"Target pos: ({waypoint[0]:.2f}, {waypoint[1]:.2f}), yaw: {waypoint[2]:.2f}")
            # print(f"x_err: {x_err:.2f}, yaw_err: {yaw_err:.2f}")

            error = np.array([x_err,
                              y_err])
            self.error_integral = self.error_integral + error
            error_diff = error - self.prev_error
            self.prev_error = error

            response = np.dot(self.Kp, error) + np.dot(self.Ki, self.error_integral) + np.dot(self.Kd, error_diff)
            
            print(f"Controller response: v={response[0]:.2f}, w={response[1]:.2f}")

            v = response[0]
            w = response[1]

            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.pub.publish(cmd)   

            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Callback when goal point is published
    # ------------------------------------------------------------------
    def planning_callback(self, msg: PointStamped):
        trajectory = plan_curved_trajectory((msg.point.x, msg.point.y))

        for waypoint in trajectory:
            self.controller(waypoint)
        
        print("Reached goal point. Stopping robot.")
        cmd = Twist()
        cmd.linear.x = float(0)
        cmd.angular.z = float(0)
        self.pub.publish(cmd)   

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_from_yaw(yaw):
        """Return quaternion (x, y, z, w) from yaw angle."""
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

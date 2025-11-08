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
        self.Kp = np.diag([0.2, 0.000000000000000000000000000001])
        self.Kd = np.diag([-0.00000000000005, 0.000000000000000000000000000000000000000005])
        self.Ki = np.diag([0, 0])

        # Subscriber
        self.create_subscription(PointStamped, '/goal_point', self.planning_callback, 10)

        self.get_logger().info('TurtleBot controller node initialized.')

    # ------------------------------------------------------------------
    # Main waypoint controller
    # ----------------, axis=1--------------------------------------------------
    def controller(self, waypoint):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # TODO: Transform the waypoint from the odom/world frame into the robot's base_link frame 
            # before computing errors — you'll need this so x_err and yaw_err are in the robot's coordinate system.
            waypoint_stamped = PointStamped()
            waypoint_stamped.header.frame_id = 'odom'
            waypoint_stamped.point.x = waypoint[0]
            waypoint_stamped.point.y = waypoint[1]
            waypoint_stamped.point.z = 0.0
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
                transformed_waypoint = self.tf_buffer.transform(waypoint_stamped, 'base_link')
                
                # Get the waypoint coordinates in the 'base_link' frame
                waypoint = (transformed_waypoint.point.x, transformed_waypoint.point.y)
            except:
                self.get_logger().warn('TF lookup failed, retreading - yaying...')
                continue
            
            #TODO: Calculate x and yaw errors! 
            posx = trans.transform.translation.x
            posy = trans.transform.translation.y
            q = trans.transform.rotation
            roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])

            heading = np.arctan2(waypoint[1] - posy, waypoint[0] - posx)

            x_err = np.sqrt((posx - waypoint[0])**2 + (posy-waypoint[1])**2)
            yaw_err = heading - yaw

            self.last_error = self.error if hasattr(self, 'error') else np.array([[0.0], [0.0]])
            self.error = np.array([[x_err],
                              [yaw_err]])
            
            self.cum_error = self.cum_error + self.error if hasattr(self, 'cum_error') else self.error

            if abs(x_err) < 0.03 and abs(yaw_err) < 0.2:
                self.get_logger().info("Waypoint reached, moving to next.")
                return
            
            v = self.Kp[0, 0] * self.error[0] + self.Kd[0, 0] * (self.error[0] - self.last_error[0])/0.1 + self.Ki[0, 0] * self.cum_error[0]
            w = self.Kp[1, 1] * self.error[1] + self.Kd[1, 1] * (self.error[1] - self.last_error[1]) + self.Ki[1, 1] * self.cum_error[1]
            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.pub.publish(cmd)   

            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Callback when goal point is published
    # ------------------------------------------------------------------
    def planning_callback(self, msg: PointStamped):
        trajectory = plan_curved_trajectory((msg.point.x, 0))

        for waypoint in trajectory:
            self.controller(waypoint)

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

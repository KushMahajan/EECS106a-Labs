################################################################################
#
# OccupancyGrid2d class for ROS 2
#
################################################################################

import rclpy
from rclpy.node import Node

import tf2_ros
from transforms3d.euler import quat2euler

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy

class OccupancyGrid2d(Node):
    def __init__(self):
        super().__init__('occupancy_grid_2d')
        self._initialized = False

        # TF buffer + listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Load parameters
        if not self.load_parameters():
            self.get_logger().error("Error loading parameters.")
            return

        # Callbacks
        if not self.register_callbacks():
            self.get_logger().error("Error registering callbacks.")
            return

        # Initialize grid map
        self._map = np.zeros((self._x_num, self._y_num))
        self._initialized = True
        self.get_logger().info("OccupancyGrid2d initialized successfully.")

    def load_parameters(self):
        self.declare_parameter("random_downsample", 0.1)
        self._random_downsample = self.get_parameter("random_downsample").value

         # Dimensions and bounds.
        # TODO! You'll need to set values for class variables called:
        self.declare_parameter("x_num", 100)
        self._x_num = self.get_parameter("x_num").value
        self.declare_parameter("x_min", -10.0)
        self._x_min = self.get_parameter("x_min").value
        self.declare_parameter("x_max", 10.0)
        self._x_max = self.get_parameter("x_max").value
        self.declare_parameter("y_num", 100)
        self._y_num = self.get_parameter("y_num").value
        self.declare_parameter("y_min", -10.0)
        self._y_min = self.get_parameter("y_min").value
        self.declare_parameter("y_max", 10.0)
        self._y_max = self.get_parameter("y_max").value

        self._x_res = (self._x_max -  self._x_min) / self._x_num
        self._y_res = (self._y_max -  self._y_min) / self._y_num
        

        self.declare_parameter("update/occupied", 0.7)
        self._occupied_update = self.probability_to_logodds(
            self.get_parameter("update/occupied").value)
        self.declare_parameter("update/occupied_threshold", 0.97)
        self._occupied_threshold = self.probability_to_logodds(
            self.get_parameter("update/occupied_threshold").value)
        self.declare_parameter("update/free", 0.3)
        self._free_update = self.probability_to_logodds(
            self.get_parameter("update/free").value)
        self.declare_parameter("update/free_threshold", 0.03)
        self._free_threshold = self.probability_to_logodds(
            self.get_parameter("update/free_threshold").value)

        # Topics.
        # TODO! You'll need to set values for class variables called:
        # -- self._sensor_topic
        # -- self._vis_topic
        self.declare_parameter("sensor_topic", "/scan")
        self._sensor_topic = self.get_parameter("sensor_topic").value
        self.declare_parameter("vis_topic", "/mapping/occupancy_grid_2d/visualization")
        self._vis_topic = self.get_parameter("vis_topic").value

        # Frames.
        # TODO! You'll need to set values for class variables called:
        # -- self._sensor_frame
        # -- self._fixed_frame
        self.declare_parameter("sensor_frame", "base_link")
        self._sensor_frame = self.get_parameter("sensor_frame").value
        self.declare_parameter("fixed_frame", "odom")
        self._fixed_frame = self.get_parameter("fixed_frame").value

        return True

    def register_callbacks(self):

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        # Subscriber.
        self._sensor_sub = self.create_subscription(
            LaserScan, self._sensor_topic, self.sensor_callback, qos_profile
        )
        # Publisher.        
        self._vis_pub = self.create_publisher(Marker, self._vis_topic, 10)
        return True

    # Callback to process sensor measurements.
    def sensor_callback(self, msg):
        if not self._initialized:
            self.get_logger().error("Node not initialized.")
            return

        self.get_logger().debug(f"Sensor pose Callback")
        # Get our current pose from TF.
        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rclpy.time.Time())
        except Exception as e:
            # Writes an error message to the ROS log but does not raise an exception
            self.get_logger().error(f"TF lookup failed: {e}")
            return

        self.get_logger().debug(f"Sensor pose: {pose}")
        # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
        # assuming that the turtlebot is on the ground plane.
        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y

        qx = pose.transform.rotation.x
        qy = pose.transform.rotation.y
        qz = pose.transform.rotation.z
        qw = pose.transform.rotation.w
        roll, pitch, yaw = quat2euler([qw, qx, qy, qz])  # [w,x,y,z]

        if abs(pose.transform.translation.z) > 0.05:
            self.get_logger().warn("Robot not on ground plane.")
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            self.get_logger().warn("Robot roll/pitch too large.")
        # Loop over all ranges in the LaserScan.
        for idx, r in enumerate(msg.ranges):
            if np.random.rand() > self._random_downsample or np.isnan(r):
                continue
            
            # Get angle of this ray in fixed frame.
            angle = msg.angle_min + idx * msg.angle_increment + yaw
            # Compute (x, y) coordinates of the endpoint of this ray.
            end_x = sensor_x + r * np.cos(angle)
            end_y = sensor_y + r * np.sin(angle)

            if r > msg.range_max or r < msg.range_min:
                continue

            # Walk along this ray from the scan point to the sensor.
            # Update log-odds at each voxel along the way.
            # Only update each voxel once. 
            # The occupancy grid is stored in self._map
            # TODO!
            end_voxel = self.point_to_voxel(end_x, end_y)
            if end_voxel is None:
                continue  # skip this ray, endpoint out of bounds
            end_ii, end_jj = end_voxel

            # Use a set to avoid double updates
            updated_voxels = set()

            for d in np.arange(0.0, r, min(self._x_res, self._y_res)/2):
                xi = sensor_x + d * np.cos(angle)
                yi = sensor_y + d * np.sin(angle)
                voxel = self.point_to_voxel(xi, yi)
                if voxel is not None and voxel not in updated_voxels:
                    ii, jj = voxel
                    if (ii, jj) == (end_ii, end_jj):
                        # Reached the endpoint
                        self._map[ii, jj] += self._occupied_update
                        if self._map[ii, jj] > self._occupied_threshold:
                            self._map[ii, jj] = self._occupied_threshold
                    else:
                        self._map[ii, jj] += self._free_update
                        if self._map[ii, jj] < self._free_threshold:
                            self._map[ii, jj] = self._free_threshold
                    updated_voxels.add((ii, jj))

        # Visualize.
        self.visualize()

    # Convert (x, y) coordinates in fixed frame to grid coordinates.
    def point_to_voxel(self, x, y):
        ii = int((x - self._x_min) / self._x_res)
        jj = int((y - self._y_min) / self._y_res)

        if ii < 0 or ii >= self._x_num or jj < 0 or jj >= self._y_num:
            return None  # invalid voxel
        return ii, jj

    # Get the center point (x, y) corresponding to the given voxel.
    def voxel_center(self, ii, jj):
        return (self._x_min + (0.5 + ii) * self._x_res,
                self._y_min + (0.5 + jj) * self._y_res)

    # Convert between probabity and log-odds.
    def probability_to_logodds(self, p):
        return np.log(p / (1.0 - p))

    def logodds_to_probability(self, l):
        return 1.0 / (1.0 + np.exp(-l))

    # Colormap to take log odds at a voxel to a RGBA color.
    def colormap(self, ii, jj):
        p = self.logodds_to_probability(self._map[ii, jj])
        return ColorRGBA(r=p, g=0.1, b=1.0 - p, a=0.75)

    # Visualize the map as a collection of flat cubes instead of
    # as a built-in OccupancyGrid message, since that gives us more
    # flexibility for things like color maps and stuff.
    # See http://wiki.ros.org/rviz/DisplayTypes/Marker for a brief tutorial.
    def visualize(self):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._fixed_frame
        m.ns = "map"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = self._x_res
        m.scale.y = self._y_res
        m.scale.z = 0.01

        for ii in range(self._x_num):
            for jj in range(self._y_num):
                p = Point()
                p.x, p.y = self.voxel_center(ii, jj)
                m.points.append(p)
                m.colors.append(self.colormap(ii, jj))

        self._vis_pub.publish(m)
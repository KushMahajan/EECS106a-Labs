import rclpy
import cv2
import os
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.bridge = CvBridge()

        # Load YOLO model
        package_share_dir = get_package_share_directory('perception')
        model_path = os.path.join(package_share_dir, 'utilities', 'segment_bounding_box_cones.pt')
        self.model = YOLO(model_path)

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 1)
        self.cone_position_pub = self.create_publisher(PointStamped, '/goal_point', 1)
        self.camera_intrinsics = None

        self.get_logger().info('Image Subscriber Node initialized')

    def image_callback(self, msg):
        if self.camera_intrinsics is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        results = self.model(cv_image, verbose=False)

        for result in results:
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()

                img_height, img_width = cv_image.shape[:2]
                for i, mask in enumerate(masks):

                    # TODO: Get number of pixels in mask
                    pixel_count = 0. 

                    CONE_AREA = 0.0208227849

                    # TODO: Get depth of image 
                    depth = 0.

                    self.get_logger().info(f'Cone {i+1}: depth={depth:.3f}m')


                    # TODO: Get u, and v of cone in image coordinates
                    u, v = 0.

                    # TODO: Find X , Y , Z of cone
                    X = 0.
                    Y = 0.
                    Z = 0. 

                    # Convert to turtlebot frame
                    # There's no camera frame for the turtlebots, so we just do this instead 
                    G = np.array([[0, 0, 1, 0.115],
                      [-1, 0, 0, 0],
                      [0, -1, 0, 0],
                      [0, 0, 0, 1]])
                    goal_point = (G @ np.array([X, Y, Z, 1]).reshape(4, 1)).flatten()

                    point_cam = PointStamped()
                    point_cam.header.stamp = msg.header.stamp
                    point_cam.header.frame_id = 'base_link'
                    point_cam.point.x = goal_point[0]
                    point_cam.point.y = goal_point[1]
                    point_cam.point.z = goal_point[2]
                    self.cone_position_pub.publish(point_cam)
            else:
                self.get_logger().info('No cones spotted')


    def camera_info_callback(self, msg):
        # -------------------------------------------
        # TODO: Extract camera intrinsic parameters! 
        # -------------------------------------------
        self.get_logger().info("Recieved Camera Info")
        

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

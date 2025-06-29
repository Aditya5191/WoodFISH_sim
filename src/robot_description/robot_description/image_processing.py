#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_sensor_subscriber')
        self.front_cam_sub = self.create_subscription(
            Image,
            '/camera/image_front_raw',
            self.front_cam_callback,
            10)
        self.down_cam_sub = self.create_subscription(
            Image,
            '/camera/image_down_raw',
            self.down_cam_callback,
            10)
        self.bridge = CvBridge()

    def front_cam_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        # Convert ROS image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.Canny(frame, 50,150)
        # Show the frame
        cv2.imshow('Front Camera', frame)
        cv2.waitKey(1)
    def down_cam_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        # Convert ROS image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Show the frame
        cv2.imshow('Down camera', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)

    # Clean up
    cv2.destroyAllWindows()
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
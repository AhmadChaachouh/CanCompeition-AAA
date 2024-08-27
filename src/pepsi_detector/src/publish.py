#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        img = cv2.imread('/home/anthony/Downloads/pepsi-image.jpeg')  # Load the image
        _, img_encoded = cv2.imencode('.jpg', img)  # Encode the image as a JPEG
        img_msg = CompressedImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.format = "jpeg"
        img_msg.data = img_encoded.tobytes()
        self.publisher_.publish(img_msg)
        self.get_logger().info('Publishing compressed image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class QRCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector_node')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create a subscriber for the camera image
        self.image_sub = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',
            self.image_callback,
            10
        )

        # Create a publisher for the x_center coordinate
        self.x_center_pub = self.create_publisher(Float32, 'qr_code_coordinates', 10)
        
        # Create a publisher for the detection status
        self.qr_detected_pub = self.create_publisher(Bool, 'qr_detected', 10)
        
        # QR code detector
        self.qr_detector = cv2.QRCodeDetector()

    def image_callback(self, msg):
        # Convert the compressed image message to a CV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Detect QR code
        data, bbox, _ = self.qr_detector.detectAndDecode(cv_image)

        detected = False
        target_x = None

        if bbox is not None and len(bbox) > 0:
            # Calculate x-center of the QR code
            xmin = bbox[0][0][0]
            xmax = bbox[0][2][0]
            target_x = (xmin + xmax) / 2
            detected = True
            
            # Draw bounding box and text on the image
            cv2.polylines(cv_image, [np.int32(bbox)], True, (0, 255, 0), 2)
            cv2.putText(cv_image, data, (int(xmin), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish the x_center coordinate if QR code is detected
        if target_x is not None:
            x_center_msg = Float32()
            x_center_msg.data = float(target_x)
            self.x_center_pub.publish(x_center_msg)
            self.get_logger().info(f'Publishing x_center: {target_x}')

        # Publish the detection status
        detected_msg = Bool()
        detected_msg.data = detected
        self.qr_detected_pub.publish(detected_msg)
        self.get_logger().info(f'Publishing detected: {detected}')

        # Display the annotated image
        cv2.imshow("Detected QR Code", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

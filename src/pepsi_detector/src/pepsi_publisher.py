#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import os
import ament_index_python

class PepsiDetectorNode(Node):
    def __init__(self):
        super().__init__('pepsi_detector_node')

        package_name = 'pepsi_detector'  # Replace with your package name
        package_share_directory = ament_index_python.get_package_share_directory(package_name)
        model_path = os.path.join(package_share_directory, 'weights', 'best-v8n.onnx')

        # Initialize YOLOv8 model
        self.model = YOLO(model_path, task='detect')

        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create a subscriber for the camera image
        self.image_sub = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',
            #'/robot_interfaces/compressed',
            self.image_callback,
            10
        )

        self.python_sub = self.create_subscription(
            Bool,
            'python_off',
            self.python_callback,
            10
        )

        # Create a publisher for the x_center coordinate
        self.x_center_pub = self.create_publisher(Float32, 'pepsi_coordinates', 10)
        
        # Create a publisher for the detection status
        self.pepsi_detected_pub = self.create_publisher(Bool, 'pepsi_detected', 10)
        # Variables
        self.target_x = None

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        results = self.model.predict(source=cv_image, conf=0.25, iou=0.45)
        result = results[0]
        annotated_image = result.plot()

        # Check if there are bounding boxes
        if result.boxes.xyxy.shape[0] > 0:
            box = result.boxes.xyxy[0]
            xmin, ymin, xmax, ymax = box
            self.target_x = (xmin + xmax) / 2
            detected = True
        else:
            self.target_x = None
            detected = False

        # Publish the x_center coordinate
        if self.target_x is not None:
            x_center_msg = Float32()
            x_center_msg.data = float(self.target_x)
            self.x_center_pub.publish(x_center_msg)
            self.get_logger().info(f'Publishing x_center: {self.target_x}')

        # Publish the detection status
        detected_msg = Bool()
        detected_msg.data = detected
        self.pepsi_detected_pub.publish(detected_msg)
        self.get_logger().info(f'Publishing detected: {detected}')

        # cv2.imshow("Detected Pepsi Can", annotated_image)
        # cv2.waitKey(1)

    def python_callback(self, msg):
        rclpy.shutdown()   

def main(args=None):
    rclpy.init(args=args)
    node = PepsiDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

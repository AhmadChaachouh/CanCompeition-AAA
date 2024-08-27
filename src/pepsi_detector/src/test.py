#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class PepsiDetectorNode(Node):
    def __init__(self):
        super().__init__('pepsi_detector_node')
        
        # Initialize the YOLOv8 model
        self.model = YOLO("/home/anthony/Downloads/best-v8n.onnx", task='detect')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create a subscriber to the compressed camera image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',  # Update with your compressed image topic name
            self.image_callback,
            10
        )
        
    def image_callback(self, msg):
        # Convert compressed image message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Run inference using YOLOv8
        results = self.model.predict(source=cv_image, conf=0.25, iou=0.45)
        
        # Process the first result
        result = results[0]
        
        # Draw bounding boxes using ultralytics' plot method
        annotated_image = result.plot()  # Annotate image with bounding boxes
        
        # Display the annotated image using OpenCV
        cv2.imshow("Detected Image", annotated_image)
        cv2.waitKey(1)  # Refresh window

        # Check if there are any bounding boxes
        if result.boxes.xyxy.shape[0] > 0:
            self.get_logger().info("Pepsi can detected.")
        else:
            self.get_logger().info("No Pepsi can detected.")

def main(args=None):
    rclpy.init(args=args)
    node = PepsiDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



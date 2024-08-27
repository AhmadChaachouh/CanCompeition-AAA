#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from example_interfaces.action import DetectPepsi
import rclpy.action

class PepsiDetectorNode(Node):
    def __init__(self):
        super().__init__('pepsi_detector_node')

        # Initialize YOLOv8 model
        self.model = YOLO("/home/anthony/Downloads/best-v8n.onnx", task='detect')

        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create a subscriber for the camera image
        self.image_sub = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',
            self.image_callback,
            10
        )

        # Create an action server
        self._action_server = rclpy.action.ActionServer(
            self,
            DetectPepsi,
            'detect_pepsi',
            self.execute_callback
        )
        
        # Variables
        self.target_x = None
        self.laser_scan_ranges = []
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

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
        else:
            self.target_x = None

        cv2.imshow("Detected Pepsi Can", annotated_image)
        cv2.waitKey(1)

    def scan_callback(self, msg):
        self.laser_scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment

    def execute_callback(self, goal_handle):
        feedback_msg = DetectPepsi.Feedback()
        result_msg = DetectPepsi.Result()

        min_distance = goal_handle.request.min_distance
        rate = self.create_rate(10)  # 10 Hz

        while rclpy.ok():
            if goal_handle.is_canceling():
                result_msg.success = False
                goal_handle.canceled(result_msg)
                return
            
            # Provide feedback
            if self.target_x is not None:
                feedback_msg.x_center = self.target_x
                goal_handle.publish_feedback(feedback_msg)
            
            # Check the laser scan ranges
            if self.laser_scan_ranges:
                closest_distance = min(self.laser_scan_ranges)
                if closest_distance < min_distance:
                    result_msg.success = True
                    goal_handle.succeed(result_msg)
                    return
            
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = PepsiDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

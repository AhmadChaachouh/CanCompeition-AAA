#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.get_logger().info('Yolo model will start loading')
        # Load the YOLOv8 model
        self.model = YOLO("/home/anthony/Downloads/best-v8n.onnx", task='detect')  # Replace with your model path
        self.get_logger().info('Yolo model loaded')
        
        # Create a CvBridge object
        self.bridge = CvBridge()
        
        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create a subscriber to the compressed camera image topic
        self.image_sub = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',  # Change to your actual compressed image topic
            self.image_callback,
            10)
        
        # Create a subscriber to the LaserScan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',  # Change to your actual LaserScan topic
            self.scan_callback,
            10)

        # Parameters
        self.stop_distance = 1  # Distance in meters to stop from any object
        self.rotation_speed = 0.3  # Rotation speed
        self.linear_speed = 0.5   # Forward speed

        # Timer for controlling rotation and movement
        self.timer = self.create_timer(0.1, self.control_robot)
        
        # Variables for tracking the robot's state
        self.found_pepsi = False
        self.target_x = None
        self.laser_scan_ranges = []
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0


    def image_callback(self, msg):
        # Convert compressed image message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Run inference using YOLOv8
        results = self.model.predict(source=cv_image, conf=0.25, iou=0.45)

        # Process the first result
        result = results[0]

        # Annotate the image with bounding boxes
        annotated_image = result.plot()  # This will draw the bounding boxes

        # Check if any bounding box is detected (indicating a Pepsi can)
        self.found_pepsi = False
        self.target_x = None
        if result.boxes.xyxy.shape[0] > 0:
            self.found_pepsi = True
            box = result.boxes.xyxy[0]  # Use the first detected box
            xmin, ymin, xmax, ymax = box
            self.target_x = (xmin + xmax) / 2  # Center x-coordinate of the bounding box

        # Display the annotated image
        cv2.imshow("Detected Pepsi Can", annotated_image)
        cv2.waitKey(1)  # Necessary to render the image in the window

    def scan_callback(self, msg):
        self.laser_scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment

    def control_robot(self):
        # Create a Twist message
        cmd = Twist()

        #self.get_logger().info('accessed')
        
        if self.laser_scan_ranges:
            min_distance = min(self.laser_scan_ranges)
            self.get_logger().info(f'Shortest distance detected: {min_distance:.2f} meters')

            if min_distance < self.stop_distance and self.found_pepsi:
                # Stop if too close to any object
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
            elif self.found_pepsi:
                if self.target_x is not None:
                    # Calculate the center of the image
                    center_x = 1920 / 2
                    
                    # Rotate to align with the target
                    if self.target_x < center_x - 200:  # Allowing some tolerance
                        cmd.angular.z = float(self.rotation_speed)
                        
                    elif self.target_x > center_x + 200:
                        cmd.angular.z = float(-self.rotation_speed)
                        
                    else:
                        cmd.angular.z = 0.0
                    
                    # Move forward when aligned
                    cmd.linear.x = float(self.linear_speed)
                else:
                    cmd.angular.z = float(self.rotation_speed)
                    cmd.linear.x = 0.0
            else:
                # Rotate in place if no Pepsi can is found
                cmd.angular.z = float(self.rotation_speed)
                cmd.linear.x = 0.0

        # Publish the velocity command
        self.cmd_vel_pub.publish(cmd)
        #self.get_logger().info(f'Setting angular.z to {cmd.angular.z}')
        self.get_logger().info(f'Setting linear.x to {cmd.linear.x}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


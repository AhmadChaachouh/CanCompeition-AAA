import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.qr_code_detected = False
        self.proximity_threshold = 0.2  # Distance threshold to stop moving (in meters)
        self.qr_proximity_threshold = 100
        self.laser_data = None
        self.get_logger().info('Initialised node')
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detector = cv2.QRCodeDetector()
        data, bbox, _ = detector.detectAndDecode(gray)
        
        if bbox is not None and data:
            self.qr_code_detected = True
            self.move_towards_qr_code(bbox, cv_image.shape)
        else:
            self.qr_code_detected = False
            self.search_for_qr_code()
            
    def laser_callback(self, msg):
        # Convert LaserScan data to numpy array
        self.laser_data = np.array(msg.ranges)
        
    def move_towards_qr_code(self, bbox, shape):
        twist = Twist()
        # Calculate the center of the QR code
        x_center = int(bbox[0][0][0] + bbox[0][2][0]) // 2
        y_center = int(bbox[0][0][1] + bbox[0][2][1]) // 2
        frame_center = shape[1] // 2
        
        # Calculate the distance from the QR code to the center of the frame
        qr_width = np.linalg.norm(np.array(bbox[0][0]) - np.array(bbox[0][1]))
        
        # Check for obstacles using LaserScan data
        if self.laser_data is not None:
            min_laser_distance = np.min(self.laser_data)
            if min_laser_distance < self.proximity_threshold:
                # Stop if an obstacle is too close
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return

        # Adjust the robot's movement based on QR code position and size
        # if qr_width >= self.qr_proximity_threshold:
        #     # Stop when QR code is close enough
        #     twist.linear.x = 0.0
        #     twist.angular.z = 0.0
        # else:
        # Move towards the QR code
        if x_center < frame_center - 100:
            twist.angular.z = 0.2
        elif x_center > frame_center + 100:
            twist.angular.z = -0.2
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.5

        self.cmd_pub.publish(twist)
        
    def search_for_qr_code(self):
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_pub.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
        
        self.subscription = self.create_subscription(
            Image,
            'input_image',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            'output_image',
            10)
        
        self.bridge = CvBridge()
        
        # Load pre-trained model (example using OpenCV's DNN module)
        # This is a simplified example - in practice, you'd load a trained model
        # For this example, we'll use a dummy model
        self.net = None

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process the image to detect objects
        # This is a simplified example that just draws a rectangle
        height, width, _ = cv_image.shape
        center_x = width // 2
        center_y = height // 2
        rect_width = 100
        rect_height = 100
        
        # Draw a rectangle to represent a detected object
        cv2.rectangle(cv_image, 
                     (center_x - rect_width//2, center_y - rect_height//2),
                     (center_x + rect_width//2, center_y + rect_height//2),
                     (0, 255, 0), 2)
        
        # Add text label
        cv2.putText(cv_image, 'Object', 
                   (center_x - rect_width//2, center_y - rect_height//2 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Convert back to ROS Image
        output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        output_msg.header = msg.header
        self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    
    try:
        rclpy.spin(object_detector)
    except KeyboardInterrupt:
        pass
    finally:
        object_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
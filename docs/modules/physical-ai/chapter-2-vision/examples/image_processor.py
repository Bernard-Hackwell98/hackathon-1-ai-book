import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Image,
            'input_image',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            'output_image',
            10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process the image (example: apply Gaussian blur)
        processed_image = cv2.GaussianBlur(cv_image, (15, 15), 0)
        
        # Convert back to ROS Image
        output_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
        output_msg.header = msg.header  # Preserve header information
        
        # Publish the processed image
        self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    
    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass
    finally:
        image_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
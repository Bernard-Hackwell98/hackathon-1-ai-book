import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudProcessor(Node):

    def __init__(self):
        super().__init__('point_cloud_processor')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            'input_pointcloud',
            self.pointcloud_callback,
            10)
        
        self.publisher = self.create_publisher(
            PointCloud2,
            'output_pointcloud',
            10)

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to list of points
        points_list = list(pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True))
        
        # Process the points (example: filter points within a certain range)
        filtered_points = [point for point in points_list if abs(point[0]) < 5.0 and abs(point[1]) < 5.0]
        
        # In a real implementation, you would convert the filtered points back to a PointCloud2 message
        # For this example, we'll just log the number of points
        self.get_logger().info(f'Processed point cloud with {len(filtered_points)} points')
        
        # Publish the original message (in a real implementation, you'd publish the processed one)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    point_cloud_processor = PointCloudProcessor()
    
    try:
        rclpy.spin(point_cloud_processor)
    except KeyboardInterrupt:
        pass
    finally:
        point_cloud_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
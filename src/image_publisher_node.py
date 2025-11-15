#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory


class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        
        # Declare parameters
        self.declare_parameter('max_index', 113)  # Default: last image in dataset
        self.declare_parameter('publish_frequency', 10.0)  # Hz
        self.declare_parameter('loop', True)  # Loop through images continuously
        self.declare_parameter('topic_name', '/camera/image_raw')
        
        # Get parameters
        self.max_index = self.get_parameter('max_index').get_parameter_value().integer_value
        frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        
        # Initialize variables
        self.current_index = 0
        self.bridge = CvBridge()
        
        # Get dataset path
        try:
            package_share_directory = get_package_share_directory('computer_vision')
            # Dataset is installed to share/computer_vision/dataset/data
            self.dataset_path = os.path.join(package_share_directory, 'dataset', 'data')
        except:
            # Fallback to relative path if package not installed (for development)
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self.dataset_path = os.path.join(script_dir, '..', 'dataset', 'data')
        
        self.dataset_path = os.path.abspath(self.dataset_path)
        
        # Verify dataset path exists
        if not os.path.exists(self.dataset_path):
            self.get_logger().error(f'Dataset path does not exist: {self.dataset_path}')
            raise FileNotFoundError(f'Dataset path not found: {self.dataset_path}')
        
        self.get_logger().info(f'Dataset path: {self.dataset_path}')
        self.get_logger().info(f'Max index: {self.max_index}')
        self.get_logger().info(f'Publish frequency: {frequency} Hz')
        self.get_logger().info(f'Loop: {self.loop}')
        
        # Create publisher
        self.publisher = self.create_publisher(Image, topic_name, 10)
        
        # Create timer
        timer_period = 1.0 / frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Image publisher node started!')
    
    def timer_callback(self):
        """Publish the next image in the sequence"""
        # Generate image filename with zero-padding
        image_filename = f'{self.current_index:010d}.png'
        image_path = os.path.join(self.dataset_path, image_filename)
        
        # Check if image exists
        if not os.path.exists(image_path):
            self.get_logger().warn(f'Image not found: {image_path}')
            if self.loop:
                self.current_index = 0
            return
        
        # Read image
        cv_image = cv2.imread(image_path)
        
        if cv_image is None:
            self.get_logger().error(f'Failed to read image: {image_path}')
            return
        
        # Convert OpenCV image to ROS Image message
        try:
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            
            # Publish image
            self.publisher.publish(image_msg)
            self.get_logger().debug(f'Published image {self.current_index}: {image_filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error converting/publishing image: {str(e)}')
            return
        
        # Update index
        self.current_index += 1
        
        # Handle loop or stop at max_index
        if self.current_index > self.max_index:
            if self.loop:
                self.current_index = 0
                self.get_logger().info('Looping back to first image')
            else:
                self.get_logger().info('Reached max_index. Stopping publishing.')
                self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImagePublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


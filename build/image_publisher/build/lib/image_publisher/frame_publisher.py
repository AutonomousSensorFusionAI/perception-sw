import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
import os

class ImagePublisher(Node):
    def __init__(self, image_folder):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'frames', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.image_folder = image_folder
        self.image_files = self.load_images()
        self.index = 0
        self.bridge = CvBridge()

    def load_images(self):
        supported_formats = ('.jpg', '.jpeg', '.png')
        return [f for f in os.listdir(self.image_folder) if f.endswith(supported_formats)]

    def timer_callback(self):
        if not self.image_files:
            self.get_logger().warn('No images found in the specified folder.')
            return
        
        if self.index >= len(self.image_files):
            self.index = 0  # Reset index to loop through images

        image_path = os.path.join(self.image_folder, self.image_files[self.index])
        cv_image = cv2.imread(image_path)
        
        if cv_image is None:
            self.get_logger().warn(f'Failed to read image: {image_path}')
            return

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher_.publish(ros_image)
        self.get_logger().info(f'Publishing: {self.image_files[self.index]}')
        self.index += 1

def main(args=None):
    parser = argparse.ArgumentParser(description='Image Publisher Node')
    parser.add_argument('--image_folder', type=str, required=True, help='Path to the folder containing images')
    args = parser.parse_args()

    rclpy.init(args=None)
    
    image_publisher = ImagePublisher(args.image_folder)
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


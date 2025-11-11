'''
250319, sjy3

Subscribe to an image topic and save it as a jpg file once per second.
'''


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
from datetime import datetime
import os

class ImageSaver(Node):
    def __init__(self, topic_name):
        super().__init__('img_saver')
        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription = self.create_subscription(Image, topic_name, self.image_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f"Subscribed to topic: {topic_name}")
        os.makedirs('outputs', exist_ok=True)

    def image_callback(self, msg):
        self.latest_image = msg

    def timer_callback(self):
        if self.latest_image is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
            except Exception as e:
                self.get_logger().error(f"Image conversion failed: {e}")
                return
            ts = self.latest_image.header.stamp.sec + self.latest_image.header.stamp.nanosec * 1e-9
            dt = datetime.fromtimestamp(ts)
            filename = dt.strftime("%y%m%d_%H%M%S") + ".jpg"
            filepath = os.path.join("outputs", filename)
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f"Saved image: {filepath}")
            self.latest_image = None

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="ROS2 Image Saver Node")
    parser.add_argument('--topic', type=str, default='image_raw', help='Image topic name')
    
    args, _ = parser.parse_known_args()
    node = ImageSaver(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

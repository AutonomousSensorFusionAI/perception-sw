'''
250210 sjy3

Script for extracting Image messages to mp4 video.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import datetime

class ImageToMP4(Node):
    def __init__(self):
        super().__init__('image_to_mp4')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw2',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.video_writer = None
        self.frame_width = None
        self.frame_height = None
        self.fps = 30  # 기본 FPS 설정 (필요시 조정 가능)
        current_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.out_filename = f'output_{current_time}.mp4'
        self.start_time = time.time()

    def image_callback(self, msg):
        try:
            # ROS2 Image 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 비디오 파일 초기화 (첫 프레임에서 실행)
            if self.video_writer is None:
                self.frame_height, self.frame_width = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(self.out_filename, fourcc, self.fps, (self.frame_width, self.frame_height))
                self.get_logger().info(f'비디오 저장 시작: {self.out_filename}, 크기: {self.frame_width}x{self.frame_height}, FPS: {self.fps}')
            
            # 프레임 저장
            self.video_writer.write(frame)
        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류 발생: {e}')

    def stop_recording(self):
        if self.video_writer:
            self.video_writer.release()
            self.get_logger().info('비디오 저장 완료')


def main(args=None):
    rclpy.init(args=args)
    node = ImageToMP4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('녹화 중지 요청')
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

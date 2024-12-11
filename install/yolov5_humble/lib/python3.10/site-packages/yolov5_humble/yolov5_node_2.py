import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from msgs_bbox.msg import BBox, BBoxes
from cv_bridge import CvBridge
import cv2
import torch

class YOLOv5Node(Node):
    def __init__(self):
        super().__init__('yolov5_node')

        # CUDA 장치 사용 여부 확인
        # self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.device = 'cpu'
	
        # YOLOv5 모델 로드 (custom weights 경로를 확인하세요)
        model_path = '/home/wise/ws_yolov5/src/yolov5_humble/resource/ADSP_test.pt'
        self.model = torch.hub.load('ultralytics/yolov5', "custom", path=model_path, device=self.device)
#        self.model.eval()


        self.image_subscriber = self.create_subscription(
            Image,
            '/frames',
            self.image_callback,
            10
        )


        self.detection_publisher = self.create_publisher(BBoxes, '/yolov5_detections', 10)

        # CvBridge 초기화
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # ROS 이미지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLOv5 추론
        results = self.model(frame)

        # 바운딩 박스 데이터 파싱
        res = results.xyxy[0]  # [x1, y1, x2, y2, confidence, class]
        detections = res.tolist()
        print(detections)
	
        # 결과 처리
        bboxes_msg = BBoxes()
        bboxes_msg.bbox = []
	
        for x1, y1, x2, y2, conf, cls in detections:  # 결과 처리
            bbox = BBox()
            bbox.cls = int(cls)
            bbox.x1 = int(x1)
            bbox.y1 = int(y1)
            bbox.x2 = int(x2)
            bbox.y2 = int(y2)
            bbox.conf = round(conf, 4)
            bboxes_msg.bbox.append(bbox)

        # 결과 퍼블리시
        self.detection_publisher.publish(bboxes_msg)

def main(args=None):
    rclpy.init(args=args)
    yolov5_node = YOLOv5Node()
    rclpy.spin(yolov5_node)

    yolov5_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


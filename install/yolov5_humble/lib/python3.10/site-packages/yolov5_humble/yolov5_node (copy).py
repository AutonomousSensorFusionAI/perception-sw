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
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)
        self.model.to(self.device)  # 모델을 적절한 장치로 이동
        self.model.eval()

        # 이미지 구독
        self.image_subscriber = self.create_subscription(
            Image,
            '/frames',
            self.image_callback,
            10
        )

        # 감지 결과 퍼블리시
        self.detection_publisher = self.create_publisher(BBoxes, '/yolov5_detections', 10)

        # CvBridge 초기화
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        resized_image = cv2.resize(cv_image, (640, 640))

        # OpenCV 이미지를 Torch 텐서로 변환
        img_tensor = torch.from_numpy(resized_image.transpose((2, 0, 1)))  # HWC to CHW
        img_tensor = img_tensor.float() / 255.0  # Normalize to [0, 1]
        img_tensor = img_tensor.unsqueeze(0)  # Add batch dimension

        # 텐서를 적절한 장치로 이동
        img_tensor = img_tensor.to(self.device)

        # YOLOv5 모델에 이미지 입력
        with torch.no_grad():  # Gradient 계산 비활성화
            results = self.model(img_tensor)

        # 결과 처리
        bboxes_msg = BBoxes()
        bboxes_msg.bbox = []
	
        print(results)
        for *box, conf, cls in results.pred[0]:  # 결과 처리
            bbox = BBox()
            bbox.cls = int(cls.item())
            bbox.x1 = float(box[0].item())
            bbox.y1 = float(box[1].item())
            bbox.x2 = float(box[2].item())
            bbox.y2 = float(box[3].item())
            bbox.conf = float(conf.item())
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


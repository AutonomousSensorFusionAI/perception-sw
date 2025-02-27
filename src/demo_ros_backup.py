#!/usr/bin/env python3.8
import argparse
import glob
from pathlib import Path
import time
import sys

import numpy as np
import torch

import rclpy  # ROS2
from rclpy.node import Node  # ROS2
from rclpy.duration import Duration
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2  # ROS2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils

# 전역 변수 초기화
c_points = None
c_convert_flag = False
visual_cloud = None

# BSD zone OD 관련 전역 변수들
object_count = 0
right_object_count = 0
right_car_count = 0
right_truck_count = 0
right_motorcycle_count = 0
left_object_count = 0
left_car_count = 0
left_truck_count = 0
left_motorcycle_count = 0

# 좌표 보정 계수 (임시 배열)
c_change_xy = np.ones((65536 * 2, 4))
c_change_xy[:, 0] = 1
c_change_xy[:, 1] = 1
c_change_xy[:, 3] = 0.001

# BSD zone 관련 상수 (단위: m)
LIDAR_TO_BPILLAR = 0.11
BPILLAR_TO_BUMPER = 2.24
BUMPER_TO_XBOUND = 3
LIDAR_TO_XBOUND = LIDAR_TO_BPILLAR + BPILLAR_TO_BUMPER + BUMPER_TO_XBOUND
LIDAR_TO_CARSIDE = 0.9375
CAR_SIDE_TO_YBOUND_START = 0.5
YBOUND_START_TO_YBOUND_END = 2.5
LIDAR_TO_YBOUND_START = LIDAR_TO_CARSIDE + CAR_SIDE_TO_YBOUND_START
LIDAR_TO_YBOUND_END = LIDAR_TO_YBOUND_START + YBOUND_START_TO_YBOUND_END

def save_points(msg: PointCloud2):
    """ROS2 콜백 함수: PointCloud2 메시지를 수신하면 numpy 배열로 변환하여 전역 변수에 저장"""
    global c_points, c_convert_flag, visual_cloud
    c_points = ros_numpy.numpify(msg)
    c_convert_flag = True
    visual_cloud = msg

def euler_to_quaternion(yaw: float, pitch: float, roll: float):
    """오일러 각을 쿼터니언으로 변환"""
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]

class DemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None, ext='.bin'):
        """
        Args:
            dataset_cfg: 데이터셋 설정
            class_names: 클래스 이름 목록
            training: 학습 여부
            root_path: 데이터 파일 경로
            logger: 로거 객체
            ext: 파일 확장자
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        self.root_path = root_path
        self.ext = ext
        if self.root_path.is_dir():
            data_file_list = glob.glob(str(root_path / f'*{self.ext}'))
        else:
            data_file_list = [str(root_path)]
        data_file_list.sort()
        self.sample_file_list = data_file_list

    def __len__(self):
        return len(self.sample_file_list)

    def __getitem__(self, index):
        if self.ext == '.bin':
            points = np.fromfile(self.sample_file_list[index], dtype=np.float32).reshape(-1, 4)
        elif self.ext == '.npy':
            points = np.load(self.sample_file_list[index])
        else:
            raise NotImplementedError
        input_dict = {'points': points, 'frame_id': index}
        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict

def parse_config():
    parser = argparse.ArgumentParser(description='OpenPCDet demo config parser')
    parser.add_argument('--cfg_file', type=str, default='cfgs/kitti_models/pv_rcnn.yaml',
                        help='specify the config for demo')
    parser.add_argument('--data_path', type=str, default='demo_data',
                        help='specify the point cloud data file or directory')
    parser.add_argument('--ckpt', type=str, default=None, help='specify the pretrained model')
    parser.add_argument('--ext', type=str, default='.bin', help='specify the extension of your point cloud data file')
    args = parser.parse_args()
    cfg_from_yaml_file(args.cfg_file, cfg)
    return args, cfg

def create_marker(marker_id: int, position, scale, color, marker_type=Marker.CUBE, lifetime_sec=0.2):
    """마커 생성 함수"""
    marker = Marker()
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    marker.pose.orientation.w = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.a = 0.4
    marker.color.r, marker.color.g, marker.color.b = color
    marker.lifetime = Duration(seconds=lifetime_sec)
    return marker

def publish_zone_markers(node: Node, marker_array: MarkerArray):
    """BSD zone 영역 마커 생성 및 발행"""
    now = node.get_clock().now().to_msg()
    # 오른쪽 zone marker
    right_marker = Marker()
    right_marker.header.stamp = now
    right_marker.header.frame_id = 'os_sensor'
    right_marker.type = Marker.CUBE
    right_marker.action = Marker.ADD
    right_marker.id = 12345
    right_marker.pose.position.x = LIDAR_TO_BPILLAR + ((LIDAR_TO_XBOUND - LIDAR_TO_BPILLAR) / 2)
    right_marker.pose.position.y = LIDAR_TO_YBOUND_START + ((LIDAR_TO_YBOUND_END - LIDAR_TO_YBOUND_START) / 2)
    right_marker.pose.position.z = -1.0
    right_marker.pose.orientation.w = 1.0
    right_marker.scale.x = LIDAR_TO_XBOUND - LIDAR_TO_BPILLAR
    right_marker.scale.y = LIDAR_TO_YBOUND_END - LIDAR_TO_YBOUND_START
    right_marker.scale.z = 0.2
    right_marker.color.a = 0.2
    right_marker.color.r, right_marker.color.g, right_marker.color.b = (1.0, 0.0, 0.0)
    right_marker.lifetime = Duration(seconds=1)
    marker_array.markers.append(right_marker)

    # 왼쪽 zone marker
    left_marker = Marker()
    left_marker.header.stamp = now
    left_marker.header.frame_id = 'os_sensor'
    left_marker.type = Marker.CUBE
    left_marker.action = Marker.ADD
    left_marker.id = 12346
    left_marker.pose.position.x = LIDAR_TO_BPILLAR + ((LIDAR_TO_XBOUND - LIDAR_TO_BPILLAR) / 2)
    left_marker.pose.position.y = -LIDAR_TO_YBOUND_START - (((LIDAR_TO_YBOUND_END - LIDAR_TO_YBOUND_START)) / 2)
    left_marker.pose.position.z = -1.0
    left_marker.pose.orientation.w = 1.0
    left_marker.scale.x = LIDAR_TO_XBOUND - LIDAR_TO_BPILLAR
    left_marker.scale.y = LIDAR_TO_YBOUND_END - LIDAR_TO_YBOUND_START
    left_marker.scale.z = 0.2
    left_marker.color.a = 0.2
    left_marker.color.r, left_marker.color.g, left_marker.color.b = (1.0, 0.0, 0.0)
    left_marker.lifetime = Duration(seconds=1)
    marker_array.markers.append(left_marker)

def main():
    args, cfg = parse_config()
    logger = common_utils.create_logger()
    logger.info('-----------------Quick Demo of OpenPCDet-------------------------')

    # 데이터셋 및 모델 초기화
    demo_dataset = DemoDataset(
        dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
        root_path=Path(args.data_path), ext=args.ext, logger=logger
    )
    model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=demo_dataset)
    model.load_params_from_file(filename=args.ckpt, logger=logger, to_cpu=True)
    model.cuda()
    model.eval()

    # ROS2 노드 생성 및 구독/발행자 설정
    rclpy.init()
    node = Node('roscom')
    node.create_subscription(PointCloud2, '/os_cloud_node/points', save_points, 10)
    pub_visual_cloud = node.create_publisher(PointCloud2, 'sinked_roi_cloud', 1)
    pub_BBox = node.create_publisher(MarkerArray, 'box_array', 1)
    pub_marker = node.create_publisher(MarkerArray, 'marker_array', 2)
    pub_BSD_OD_result = node.create_publisher(String, 'BSD_OD_result', 2)

    global c_points, c_convert_flag, visual_cloud
    c_points = None
    c_convert_flag = False

    BBox_array = MarkerArray()
    marker_array = MarkerArray()
    start_time = time.time()
    frame_cnt = 0

    # 메인 루프
    while rclpy.ok():
        if c_points is not None and c_points.size != 0:
            if c_convert_flag:
                frame_cnt += 1
                input_time = (time.time() - start_time) * 1000
                print(f"{frame_cnt}-1. 프로그램 시작 후 {frame_cnt}번째 LiDAR 데이터 프레임 입력 시간: {input_time:.2f} ms")

                # 포인트 클라우드 변환 및 전처리
                flatten_points = c_points.flatten()
                flatten_points_xyzi = flatten_points[['x', 'y', 'z', 'intensity']]
                c_points = None
                c_convert_flag = False
                points_list = flatten_points_xyzi.tolist()
                points_pre = np.array(points_list)
                points = points_pre * c_change_xy[points_pre.shape[0] - 1, :]

                input_dict = {'points': points, 'frame_id': 0}
                is_detected = False

                with torch.no_grad():
                    data_dict = demo_dataset.prepare_data(data_dict=input_dict)
                    data_dict = demo_dataset.collate_batch([data_dict])
                    load_data_to_gpu(data_dict)
                    pred_dicts, _ = model.forward(data_dict)

                    # 예측 결과 순회
                    for box_num in range(pred_dicts[0]['pred_boxes'].shape[0]):
                        label = pred_dicts[0]['pred_labels'][box_num]
                        score = pred_dicts[0]['pred_scores'][box_num]
                        # 임계값 조건 검사
                        if ((label in [1, 2] and score >= 0.7) or
                            (label == 3 and score >= 0.4) or
                            (label == 4 and score >= 0.1)):

                            box = pred_dicts[0]['pred_boxes'][box_num]
                            # 객체의 전후방 위치 계산
                            front_end_object = box[0] - (box[3] / 2)
                            rear_end_object = box[0] + (box[3] / 2)
                            # 좌우측 경계 계산 (y 값에 따라)
                            if box[1] >= 0:
                                right_end_object = box[1] + (box[4] / 2)
                                left_end_object = box[1] - (box[4] / 2)
                            else:
                                right_end_object = box[1] + (box[4] / 2)
                                left_end_object = box[1] - (box[4] / 2)
                            
                            object_yaw = box[6].item()

                            # BSD Zone OD 결과 처리
                            global object_count, right_object_count, right_car_count, right_truck_count, right_motorcycle_count
                            global left_object_count, left_car_count, left_truck_count, left_motorcycle_count
                            BSD_OD_result = ""
                            BSD_OD_result2 = ""
                            
                            # 오른쪽 zone OD 검사
                            if ((LIDAR_TO_BPILLAR <= front_end_object <= LIDAR_TO_XBOUND) and 
                                (LIDAR_TO_YBOUND_START <= left_end_object <= LIDAR_TO_YBOUND_END) and
                                (((2.5 <= object_yaw <= 3.99)) or ((5.8 <= object_yaw <= 6.99)))):
                                object_count += 1
                                right_object_count += 1
                                is_detected = True
                                if label in [1, 2]:
                                    right_car_count += 1
                                    BSD_OD_result = f"BSD Must Zone OD Result: Right Car {right_car_count}"
                                    BSD_OD_result2 = f"{frame_cnt}-2. 장애물 인지 결과: Right Car {right_car_count}"
                                elif label == 3:
                                    right_truck_count += 1
                                    BSD_OD_result = f"BSD Must Zone OD Result: Right Lorry or Bus {right_truck_count}"
                                    BSD_OD_result2 = f"{frame_cnt}-2. 장애물 인지 결과: Right Lorry or Bus {right_truck_count}"
                                elif label == 4:
                                    right_motorcycle_count += 1
                                    BSD_OD_result = f"BSD Must Zone OD Result: Right Motorcycle {right_motorcycle_count}"
                                    BSD_OD_result2 = f"{frame_cnt}-2. 장애물 인지 결과: Right Motorcycle {right_motorcycle_count}"
                                    # 녹색 마커 (우측)
                                    marker = create_marker(
                                        marker_id=box_num,
                                        position=(box[0], box[1], box[2]),
                                        scale=(box[3], box[4], box[5]),
                                        color=(0.0, 0.6, 0.0)
                                    )
                                    marker_array.markers.append(marker)
                            
                            # 왼쪽 zone OD 검사
                            elif ((LIDAR_TO_BPILLAR <= front_end_object <= LIDAR_TO_XBOUND) and 
                                  (-LIDAR_TO_YBOUND_END <= right_end_object <= -LIDAR_TO_YBOUND_START) and
                                  (((2.5 <= object_yaw <= 3.99)) or ((5.8 <= object_yaw <= 6.99)))):
                                object_count += 1
                                left_object_count += 1
                                is_detected = True
                                if label in [1, 2]:
                                    left_car_count += 1
                                    BSD_OD_result = f"BSD Must Zone OD Result: Left Car {left_car_count}"
                                    BSD_OD_result2 = f"{frame_cnt}-2. 장애물 인지 결과: Left Car {left_car_count}"
                                elif label == 3:
                                    left_truck_count += 1
                                    BSD_OD_result = f"BSD Must Zone OD Result: Left Lorry or Bus {left_truck_count}"
                                    BSD_OD_result2 = f"{frame_cnt}-2. 장애물 인지 결과: Left Lorry or Bus {left_truck_count}"
                                elif label == 4:
                                    left_motorcycle_count += 1
                                    BSD_OD_result = f"BSD Must Zone OD Result: Left Motorcycle {left_motorcycle_count}"
                                    BSD_OD_result2 = f"{frame_cnt}-2. 장애물 인지 결과: Left Motorcycle {left_motorcycle_count}"
                                    # 녹색 마커 (좌측)
                                    marker = create_marker(
                                        marker_id=box_num,
                                        position=(box[0], box[1], box[2]),
                                        scale=(box[3], box[4], box[5]),
                                        color=(0.0, 0.6, 0.0)
                                    )
                                    marker_array.markers.append(marker)
                            
                            # BSD OD 결과 출력 및 발행
                            if BSD_OD_result:
                                print(BSD_OD_result2)
                                pub_BSD_OD_result.publish(String(data=BSD_OD_result))
                            
                            # 객체 박스 마커 (공통)
                            box_marker = Marker()
                            box_marker.header.stamp = node.get_clock().now().to_msg()
                            box_marker.header.frame_id = 'os_sensor'
                            box_marker.id = box_num
                            # 박스 중심 및 회전
                            box_marker.pose.position.x = box[0]
                            box_marker.pose.position.y = box[1]
                            box_marker.pose.position.z = box[2]
                            yaw = box[6].item() + 1.57
                            quat = euler_to_quaternion(yaw, 0, 0)
                            box_marker.pose.orientation.x, box_marker.pose.orientation.y, box_marker.pose.orientation.z, box_marker.pose.orientation.w = quat
                            # 박스 크기 설정 (주의: x, y 축 위치 변경)
                            box_marker.scale.x = box[4]
                            box_marker.scale.y = box[3]
                            box_marker.scale.z = box[5]
                            BBox_array.markers.append(box_marker)

                            # 객체별 마커 색상 설정 (Car: 파란색, Lorry/Bus: 보라색)
                            if label in [1, 2]:
                                color = (0.0, 0.0, 1.0)
                            elif label == 3:
                                color = (0.4, 0.0, 1.0)
                            else:
                                color = (1.0, 1.0, 1.0)
                            marker = create_marker(
                                marker_id=box_num,
                                position=(box[0], box[1], box[2]),
                                scale=(box[3], box[4], box[5]),
                                color=color
                            )
                            marker_array.markers.append(marker)

                    # 발행: 박스와 마커가 있을 경우
                    if BBox_array.markers:
                        pub_BBox.publish(BBox_array)
                        pub_marker.publish(marker_array)
                    else:
                        BBox_array = MarkerArray()
                        marker_array = MarkerArray()

                # 시각화 클라우드 발행
                pub_visual_cloud.publish(visual_cloud)

                if is_detected:
                    perception_time = (time.time() - start_time) * 1000
                    print(f"{frame_cnt}-3. 장애물 인지 완료 시간: {perception_time:.2f} ms")
                    print(f"{frame_cnt}-4. 장애물 인지 속도: {perception_time - input_time:.2f} ms")
                    print("-----------------------------------------------------------------------------------")
                else:
                    print(f"{frame_cnt}-2. {frame_cnt}번째 LiDAR 데이터는 빨간색 영역에 장애물 없음")
                    print("-----------------------------------------------------------------------------------")

            # BSD zone 마커 발행 (별도 처리)
            publish_zone_markers(node, marker_array)
            pub_marker.publish(marker_array)
            marker_array = MarkerArray()  # 매 반복마다 초기화
        
        rclpy.spin_once(node, timeout_sec=0.001)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

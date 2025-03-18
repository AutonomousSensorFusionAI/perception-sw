#!/usr/bin/env python3.8
import argparse
import glob
from pathlib import Path
from pickle import TRUE

# import mayavi.mlab as mlab
import numpy as np
import torch
import time
import sys
import rclpy # ROS2 
from rclpy.node import Node # ROS2 
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2 # ROS2
# from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
# from visual_utils import visualize_utils as V

# np.set_printoptions(threshold=sys.maxsize)


# variables
c_points = None
c_convert_flag = False
visual_cloud = None
object_count = 0
right_object_count = 0
right_car_count = 0
right_van_count = 0
right_truck_count = 0
right_motorcyle_count = 0
left_object_count = 0
left_car_count = 0
left_van_count = 0
left_truck_count = 0
left_motorcyle_count = 0
x_axis_tolerance = 0.1
y_axis_tolerance = 0.1

def save_points(msg):
    global c_points
    global c_convert_flag
    global visual_cloud
    c_points = ros_numpy.numpify(msg)
    c_convert_flag = True
    visual_cloud = msg

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class DemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None, ext='.bin'):
        """
        Args:
            root_path:
            dataset_cfg:
            class_names:
            training:
            logger:
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        self.root_path = root_path
        self.ext = ext
        data_file_list = glob.glob(str(root_path / f'*{self.ext}')) if self.root_path.is_dir() else [self.root_path]

        data_file_list.sort()
        self.sample_file_list = data_file_list

    def __len__(self):
        return len(self.sample_file_list)

    def __getitem__(self, index):
        if self.ext == '.bin':
            # KITTI dataset config
            points = np.fromfile(self.sample_file_list[index], dtype=np.float32).reshape(-1, 4)
            # Nuscenes dataset config (not working)
            # points = np.fromfile(self.sample_file_list[index], dtype=np.float32).reshape(-1, 5)
        elif self.ext == '.npy':
            points = np.load(self.sample_file_list[index])
        else:
            raise NotImplementedError
        
        input_dict = {
            'points': points,
            'frame_id': index,
        }

        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default='cfgs/kitti_models/pv_rcnn.yaml',
                        help='specify the config for demo')
    parser.add_argument('--data_path', type=str, default='demo_data',
                        help='specify the point cloud data file or directory')
    parser.add_argument('--ckpt', type=str, default=None, help='specify the pretrained model')
    parser.add_argument('--ext', type=str, default='.bin', help='specify the extension of your point cloud data file')

    args = parser.parse_args()

    cfg_from_yaml_file(args.cfg_file, cfg)

    return args, cfg

def main():
    args, cfg = parse_config()
    
    logger = common_utils.create_logger()
    logger.info('-----------------Quick Demo of OpenPCDet-------------------------')
    
    demo_dataset = DemoDataset(
        dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
        root_path=Path(args.data_path), ext=args.ext, logger=logger
    )

    model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=demo_dataset)
    model.load_params_from_file(filename=args.ckpt, logger=logger, to_cpu=True)
    model.cuda()
    model.eval()
    
    rclpy.init() # ROS2 
    node = Node('roscom')
    
    node.create_subscription(PointCloud2, '/os_cloud_node/points', save_points, 10) # ROS2 
    # rclpy.create_subscription('/os_cloud_node/points', PointCloud2, save_points) # ROS2 
    # It is good to use a queue size of 1 to 3 at 10 Hz.
    pub_visual_cloud = node.create_publisher(PointCloud2, 'sinked_roi_cloud', 1) # ROS2 
    pub_BBox = node.create_publisher(MarkerArray, 'box_array', 1) # ROS2 
    pub_marker = node.create_publisher(MarkerArray, 'marker_array', 2) # ROS2
    pub_BSD_OD_result = node.create_publisher(String, 'BSD_OD_result', 2) # ROS2 
    
    global c_points
    global c_convert_flag
    global visual_cloud
    c_points = None
    c_convert_flag = False
    c_change_xy = np.ones((65536*2,4))
    BBox_array = MarkerArray()
    marker_array = MarkerArray()
    global object_count
    global right_object_count
    global right_car_count
    global right_van_count
    global right_truck_count
    global right_motorcyle_count
    global left_object_count
    global left_car_count
    global left_van_count
    global left_truck_count
    global left_motorcyle_count
    start_time = time.time()
    frame_cnt = 0
    
    for i in range(65536*2):
        c_change_xy[i][0] = 1
        c_change_xy[i][1] = 1
        c_change_xy[i][3] = 0.001

    ''' 
    * BSD zone OD test 
    LiDAR installation state: Connector is installed in the vehicle driving direction
        - x value is negative in the connector direction
        - y value is positive in the right direction
    scale value is always positive
    * Info for testing BSD zone OD
    Car list
        - Grandeur HG: height 4.99m, width 1.875m
    current BSD zone info: Grandeur HG
    '''
    lidar_to_BPillar = 0.11
    BPillar_to_bumper = 2.24
    bumper_to_xBound = 3
    lidar_to_xBound = lidar_to_BPillar + BPillar_to_bumper + bumper_to_xBound
    lidar_to_carSide = 0.9375
    carSide_to_yBoundStart = 0.5
    yBoundStart_to_yBoundEnd = 2.5
    lidar_to_yBoundStart = lidar_to_carSide + carSide_to_yBoundStart
    lidar_to_yBoundEnd = lidar_to_yBoundStart + yBoundStart_to_yBoundEnd
    BSD_OD_result = ""
    
    while rclpy.ok(): # ROS2
        if (c_points is not None) and (c_points.size != 0):
            if c_convert_flag:
                # 새로운 PCD 프레임이 input으로 입력될 때의 시간
                frame_cnt += 1
                input_time = (time.time() - start_time) * 1000
                print("%d-1. 프로그램 시작 후 %d번째 LiDAR 데이터 프레임이 입력된 시간: %.2f ms" % (frame_cnt, frame_cnt, input_time))
                
                flatten_points = c_points.flatten()
                flatten_points_xyzi = flatten_points[['x', 'y', 'z', 'intensity']]

                # should be moved to last
                c_points = None
                c_convert_flag = False

                flatten_points_xyzi_list =  flatten_points_xyzi.tolist()
                points_pre = np.array(flatten_points_xyzi_list)
                points = points_pre * c_change_xy[points_pre.shape[0]-1,:]

                input_dict = {
                'points': points,
                'frame_id': 0,
                }
                # print("prepre time = {}".format(time.time()-start_time)) # test
                
                is_detected = False
                
                with torch.no_grad():
                    data_dict = demo_dataset.prepare_data(data_dict=input_dict)
                    data_dict = demo_dataset.collate_batch([data_dict])
                    load_data_to_gpu(data_dict)
                    pred_dicts, _ = model.forward(data_dict)

                    for box_num in range(pred_dicts[0]['pred_boxes'].shape[0]):
                        # tradoff: score(confidence level) 낮으면 오인시 증가, 미인식 감소 / score 높으면 오인식 감소, 미인식 증가
                        if ((pred_dicts[0]['pred_labels'][box_num] == 1 and pred_dicts[0]['pred_scores'][box_num] >= 0.7) or 
                           (pred_dicts[0]['pred_labels'][box_num] == 2 and pred_dicts[0]['pred_scores'][box_num] >= 0.7) or 
                           (pred_dicts[0]['pred_labels'][box_num] == 3 and pred_dicts[0]['pred_scores'][box_num] >= 0.4) or
                           (pred_dicts[0]['pred_labels'][box_num] == 4 and pred_dicts[0]['pred_scores'][box_num] >= 0.1)):

                            
                            ''' object poistion and yaw '''
                            front_end_object = pred_dicts[0]['pred_boxes'][box_num][0] - (pred_dicts[0]['pred_boxes'][box_num][3]/2)
                            rear_end_object = pred_dicts[0]['pred_boxes'][box_num][0] + (pred_dicts[0]['pred_boxes'][box_num][3]/2)
                      
                            if pred_dicts[0]['pred_boxes'][box_num][1] >= 0:
                                right_end_object = pred_dicts[0]['pred_boxes'][box_num][1] + (pred_dicts[0]['pred_boxes'][box_num][4]/2)
                                left_end_object = pred_dicts[0]['pred_boxes'][box_num][1] - (pred_dicts[0]['pred_boxes'][box_num][4]/2)

                            # left position
                            if pred_dicts[0]['pred_boxes'][box_num][1] < 0:
                                right_end_object = pred_dicts[0]['pred_boxes'][box_num][1] + (pred_dicts[0]['pred_boxes'][box_num][4]/2)
                                left_end_object = pred_dicts[0]['pred_boxes'][box_num][1] - (pred_dicts[0]['pred_boxes'][box_num][4]/2)

                            object_yaw = pred_dicts[0]['pred_boxes'][box_num][6].item()
                            
                            ''' get BSD must zone OD result'''
                            boxboxbox = Marker()
                            markermarker = Marker()
                            now = node.get_clock().now() # ROS2 
                            boxboxbox.label = pred_dicts[0]['pred_labels'][box_num]
                            boxboxbox.header.stamp = now.to_msg() # ROS2
                            BBox_array.header.stamp = now.to_msg() # ROS2 
                            markermarker.header.stamp = now.to_msg # ROS2
                            boxboxbox.header.frame_id = 'os_sensor'
                            BBox_array.header.frame_id = 'os_sensor'
                            markermarker.header.frame_id = 'os_sensor'
                            
                            # right zone OD
                            if (((lidar_to_BPillar <= front_end_object) and (front_end_object <= lidar_to_xBound)) and 
                                ((lidar_to_yBoundStart <= left_end_object) and (left_end_object <= lidar_to_yBoundEnd)) and
                                (((2.5 <= object_yaw) and (object_yaw <= 3.99)) or ((5.8 <= object_yaw) and (object_yaw <= 6.99)))):
                                object_type = None
                                object_count += 1
                                right_object_count += 1
                                is_detected = True
                                if pred_dicts[0]['pred_labels'][box_num] == 1 or pred_dicts[0]['pred_labels'][box_num] == 2:
                                    object_type = "Car"
                                    right_car_count += 1
                                    BSD_OD_result = "BSD Must Zone OD Reuslt: Right " + object_type + " " + str(right_car_count)
                                    BSD_OD_result2 = "%d-2. 장애물 인지 결과: Right " % (frame_cnt) + object_type + " " + str(right_car_count)
                                    # print(pred_dicts[0]['pred_boxes'][box_num][6].item()) # yaw test
                                elif pred_dicts[0]['pred_labels'][box_num] == 3:
                                    object_type = "Lorry or Bus"
                                    right_truck_count += 1
                                    BSD_OD_result = "BSD Must Zone OD Reuslt: Right " + object_type + " " + str(right_truck_count)
                                    BSD_OD_result2 = "%d-2. 장애물 인지 결과: Right " % (frame_cnt) + object_type + " " + str(right_truck_count)
                                    # print("!!! 우측 BSD Must 존 Lorry 또는 Bus 인지함. confidence level: %f !!!" % (pred_dicts[0]['pred_scores'][box_num]))
                                elif pred_dicts[0]['pred_labels'][box_num] == 4:
                                    object_type = "Motorcyle"
                                    right_motorcyle_count += 1
                                    BSD_OD_result = "BSD Must Zone OD Reuslt: Right " + object_type + " " + str(right_motorcyle_count)
                                    BSD_OD_result2 = "%d-2. 장애물 인지 결과: Right " % (frame_cnt) + object_type + " " + str(right_motorcyle_count)
                                    # print("### 우측 BSD Must 존 Motorcyle 또는 cyclist 인지함. confidence level: %f ###" % (pred_dicts[0]['pred_scores'][box_num]))
                                    markermarker.type = markermarker.CUBE
                                    markermarker.action = markermarker.ADD
                                    markermarker.id = box_num
                                    markermarker.pose.position.x = pred_dicts[0]['pred_boxes'][box_num][0]
                                    markermarker.pose.position.y = pred_dicts[0]['pred_boxes'][box_num][1]
                                    markermarker.pose.position.z = pred_dicts[0]['pred_boxes'][box_num][2]
                                    markermarker.pose.orientation.w = 1.0
                                    markermarker.scale.x = pred_dicts[0]['pred_boxes'][box_num][3]
                                    markermarker.scale.y = pred_dicts[0]['pred_boxes'][box_num][4]
                                    markermarker.scale.z = pred_dicts[0]['pred_boxes'][box_num][5]
                                    markermarker.color.a = 0.4
                                    # 초록색 marker
                                    markermarker.color.r = 0.0
                                    markermarker.color.g = 0.6
                                    markermarker.color.b = 0.0
                                    markermarker.lifetime = rclpy.duration.Duration(seconds=0.2) # ROS2 
                                    marker_array.markers.append(markermarker)
                            # left zone OD
                            elif (((lidar_to_BPillar <= front_end_object) and (front_end_object <= lidar_to_xBound)) and 
                                  ((-lidar_to_yBoundEnd <= right_end_object) and (right_end_object <= -lidar_to_yBoundStart)) and
                                (((2.5 <= object_yaw) and (object_yaw <= 3.99)) or ((5.8 <= object_yaw) and (object_yaw <= 6.99)))):
                                object_type = None
                                object_count += 1
                                left_object_count += 1
                                is_detected = True
                                if pred_dicts[0]['pred_labels'][box_num] == 1 or pred_dicts[0]['pred_labels'][box_num] == 2:
                                    object_type = "Car"
                                    left_car_count += 1
                                    BSD_OD_result = "BSD Must Zone OD Reuslt: Left " + object_type + " " + str(left_car_count)
                                    BSD_OD_result2 = "%d-2. 장애물 인지 결과: Left " % (frame_cnt) + object_type + " " + str(left_car_count)
                                    # print(pred_dicts[0]['pred_boxes'][box_num][6].item()) # yaw test
                                elif pred_dicts[0]['pred_labels'][box_num] == 3:
                                    object_type = "Lorry or Bus"
                                    left_truck_count += 1
                                    BSD_OD_result = "BSD Must Zone OD Reuslt: Left " + object_type + " " + str(left_truck_count)
                                    BSD_OD_result2 = "%d-2. 장애물 인지 결과: Left " % (frame_cnt) + object_type + " " + str(left_truck_count)
                                    # print("!!! 좌측 BSD Must 존 Lorry 또는 Bus 인지함. confidence level: %f !!!" % (pred_dicts[0]['pred_scores'][box_num]))
                                elif pred_dicts[0]['pred_labels'][box_num] == 4:
                                    object_type = "Motorcyle"
                                    left_motorcyle_count += 1
                                    BSD_OD_result = "BSD Must Zone OD Reuslt: Left " + object_type + " " + str(left_motorcyle_count)
                                    BSD_OD_result2 = "%d-2. 장애물 인지 결과: Left " % (frame_cnt) + object_type + " " + str(left_motorcyle_count)
                                    # print("### 좌측 BSD Must 존 Motorcyle 또는 cyclist 인지함. confidence level: %f ###" % (pred_dicts[0]['pred_scores'][box_num]))
                                    markermarker.type = markermarker.CUBE
                                    markermarker.action = markermarker.ADD
                                    markermarker.id = box_num
                                    markermarker.pose.position.x = pred_dicts[0]['pred_boxes'][box_num][0]
                                    markermarker.pose.position.y = pred_dicts[0]['pred_boxes'][box_num][1]
                                    markermarker.pose.position.z = pred_dicts[0]['pred_boxes'][box_num][2]
                                    markermarker.pose.orientation.w = 1.0
                                    markermarker.scale.x = pred_dicts[0]['pred_boxes'][box_num][3]
                                    markermarker.scale.y = pred_dicts[0]['pred_boxes'][box_num][4]
                                    markermarker.scale.z = pred_dicts[0]['pred_boxes'][box_num][5]
                                    markermarker.color.a = 0.4
                                    # 초록색 marker
                                    markermarker.color.r = 0.0
                                    markermarker.color.g = 0.6
                                    markermarker.color.b = 0.0
                                    markermarker.lifetime = rclpy.duration.Duration(seconds=0.2) # ROS2 
                                    marker_array.markers.append(markermarker)

                                   
                            ''' print BSD zone OD result'''
                            if len(BSD_OD_result) != 0:
                                print(BSD_OD_result2)
                                pub_BSD_OD_result.publish(BSD_OD_result)
                                BSD_OD_result = ""
                                BSD_OD_result2 = ""
                                
                            # box
                            boxboxbox.pose.position.x = pred_dicts[0]['pred_boxes'][box_num][0]
                            boxboxbox.pose.position.y = pred_dicts[0]['pred_boxes'][box_num][1]
                            boxboxbox.pose.position.z = pred_dicts[0]['pred_boxes'][box_num][2]
                            yaw = pred_dicts[0]['pred_boxes'][box_num][6].item() + 1.57
                            quat = euler_to_quaternion(yaw, 0, 0)
                            boxboxbox.pose.orientation.x = quat[0]
                            boxboxbox.pose.orientation.y = quat[1]
                            boxboxbox.pose.orientation.z = quat[2]
                            boxboxbox.pose.orientation.w = quat[3]
                            boxboxbox.dimensions.x = pred_dicts[0]['pred_boxes'][box_num][4]
                            boxboxbox.dimensions.y = pred_dicts[0]['pred_boxes'][box_num][3]
                            boxboxbox.dimensions.z = pred_dicts[0]['pred_boxes'][box_num][5]
                            BBox_array.boxes.append(boxboxbox)

                            # marker
                            # Car(sedan, SUV, van, pickup truck etc)
                            if pred_dicts[0]['pred_labels'][box_num] == 1 or pred_dicts[0]['pred_labels'][box_num] == 2: 
                                markermarker.type = markermarker.CUBE
                                markermarker.action = markermarker.ADD
                                markermarker.id = box_num
                                markermarker.pose.position.x = pred_dicts[0]['pred_boxes'][box_num][0]
                                markermarker.pose.position.y = pred_dicts[0]['pred_boxes'][box_num][1]
                                markermarker.pose.position.z = pred_dicts[0]['pred_boxes'][box_num][2]
                                markermarker.pose.orientation.w = 1.0
                                markermarker.scale.x = pred_dicts[0]['pred_boxes'][box_num][3]
                                markermarker.scale.y = pred_dicts[0]['pred_boxes'][box_num][4]
                                markermarker.scale.z = pred_dicts[0]['pred_boxes'][box_num][5]
                                markermarker.color.a = 0.4
                                # 파란색 marker
                                markermarker.color.r = 0.0
                                markermarker.color.g = 0.0
                                markermarker.color.b = 1.0
                                markermarker.lifetime = rclpy.duration.Duration(seconds=0.2) # ROS2 
                                marker_array.markers.append(markermarker)
                            # Lorry or bus
                            elif pred_dicts[0]['pred_labels'][box_num] == 3:
                                markermarker.type = markermarker.CUBE
                                markermarker.action = markermarker.ADD
                                markermarker.id = box_num
                                markermarker.pose.position.x = pred_dicts[0]['pred_boxes'][box_num][0]
                                markermarker.pose.position.y = pred_dicts[0]['pred_boxes'][box_num][1]
                                markermarker.pose.position.z = pred_dicts[0]['pred_boxes'][box_num][2]
                                markermarker.pose.orientation.w = 1.0
                                markermarker.scale.x = pred_dicts[0]['pred_boxes'][box_num][3]
                                markermarker.scale.y = pred_dicts[0]['pred_boxes'][box_num][4]
                                markermarker.scale.z = pred_dicts[0]['pred_boxes'][box_num][5]
                                markermarker.color.a = 0.4
                                # 보라색 marker
                                markermarker.color.r = 0.4
                                markermarker.color.g = 0.0
                                markermarker.color.b = 1.0
                                markermarker.lifetime = rclpy.duration.Duration(seconds=0.2) # ROS2 
                                marker_array.markers.append(markermarker)
                        
                    if len(BBox_array.boxes) != 0:
                        pub_BBox.publish(BBox_array)
                        pub_marker.publish(marker_array)
                    else:
                        BBox_array = MarkerArray()
                        marker_array = MarkerArray()
                        # pub_BBox.publish(BBox_array)
                        # pub_marker.publish(marker_array)

                # clear
                BBox_array = MarkerArray()
                marker_array = MarkerArray()
                BSD_OD_result = ""
                pub_visual_cloud.publish(visual_cloud)
                
                if is_detected == True:
                    # 장애물을 인지한 시간과 장애물 인지 속도 계산
                    perception_time = (time.time() - start_time) * 1000
                    print("%d-3. 장애물 인지를 마친 시간: %.2f ms" % (frame_cnt, perception_time))
                    print("%d-4. 장애물 인지 속도: %.2f ms" % (frame_cnt, perception_time - input_time))
                    print("-----------------------------------------------------------------------------------")
                else:
                    print("%d-2. %d번째 LiDAR 데이터는 빨간색 영역에 장애물 없음" % (frame_cnt, frame_cnt))
                    print("-----------------------------------------------------------------------------------")
                    
                # time confirm
                # print(f"pv-rcnn-my processing time per frame: {(time.time() - start_time) * 1000:.3f} ms")

            pub_visual_cloud.publish(visual_cloud)
            
        ''' BSD must zone display # TODO: Threading '''
        now = rclpy.time.Time
        # right zone
        markermarker2 = Marker()
        markermarker2.header.stamp = now
        markermarker2.header.frame_id = 'os_sensor'      
        markermarker2.type = markermarker2.CUBE
        markermarker2.action = markermarker2.ADD
        markermarker2.id = 12345
        markermarker2.pose.position.x = lidar_to_BPillar + ((lidar_to_xBound - lidar_to_BPillar) / 2)
        markermarker2.pose.position.y = lidar_to_yBoundStart + ((lidar_to_yBoundEnd - lidar_to_yBoundStart) / 2)
        markermarker2.pose.position.z = -1.0
        markermarker2.pose.orientation.w = 0.0
        markermarker2.scale.x = lidar_to_xBound - lidar_to_BPillar
        markermarker2.scale.y = lidar_to_yBoundEnd - lidar_to_yBoundStart
        markermarker2.scale.z = 0.2
        markermarker2.color.a = 0.2
        markermarker2.color.r = 1.0
        markermarker2.color.g = 0.0
        markermarker2.color.b = 0.0
        markermarker2.lifetime = rospy.Duration.from_sec(1)
        marker_array.markers.append(markermarker2)
        # left zone
        markermarker3 = Marker()
        markermarker3.header.stamp = now
        markermarker3.header.frame_id = 'os_sensor'      
        markermarker3.type = markermarker3.CUBE
        markermarker3.action = markermarker3.ADD
        markermarker3.id = 12346
        markermarker3.pose.position.x = lidar_to_BPillar + ((lidar_to_xBound - lidar_to_BPillar) / 2)
        markermarker3.pose.position.y = -lidar_to_yBoundStart - ((-lidar_to_yBoundStart - (-lidar_to_yBoundEnd)) / 2)
        markermarker3.pose.position.z = -1.0
        markermarker3.pose.orientation.w = 1.0
        markermarker3.scale.x = lidar_to_xBound - lidar_to_BPillar
        markermarker3.scale.y = -lidar_to_yBoundStart - (-lidar_to_yBoundEnd)
        markermarker3.scale.z = 0.2
        markermarker3.color.a = 0.2
        markermarker3.color.r = 1.0
        markermarker3.color.g = 0.0
        markermarker3.color.b = 0.0
        markermarker3.lifetime = rospy.Duration.from_sec(1)
        marker_array.markers.append(markermarker3)
        pub_marker.publish(marker_array)
        marker_array = MarkerArray() # clear
    rclpy.spin(node)
    rclpy.shutdown()
            
if __name__ == '__main__':
    main()

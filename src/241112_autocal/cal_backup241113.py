'''
241112, sjy
camera, lidar, radar calibration
- 일단은 cam, lid, rad 데이터/검출 결과는 수동으로
- data/camera : 이미지, yolov5 검출결과
  data/lidar  : 라이다 pcd, 객체검출결과
  data/radar  : 레이더 객체검출결과

'''

import os
import glob
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#import open3d
import torch
from pathlib import Path


# filename = '0000000000'

# cam_path = os.path.join(root, filename+'.png')
# lid_path = os.path.join(root, filename+'.bin')
# rad_path = os.path.join(root, filename+'.txt')

# def parse_filepath(root):

#     cam_list = []  
#     lid_list = []  
#     rad_list = []  

#     # 주어진 디렉토리 내의 모든 파일을 파싱
#     for file in Path(root).rglob('*'):
#         if file.is_file():  # 파일인지 확인
#             if file.suffix.lower() == '.png':
#                 cam_list.append(file)
#             elif file.suffix.lower() in ['.bin', '.pcd']:
#                 lid_list.append(file)
#             elif file.suffix.lower() == '.txt':
#                 rad_list.append(file)

#     # return cam_list, lid_list, rad_list
#     return cam_list[0], lid_list[0], rad_list[0]


# def load_CLR_data(cam_path, lid_path, rad_path):
def load_CLR_data(root):

    cam_path = os.path.join(root, 'camera')
    lid_path = os.path.join(root, 'lidar')
    rad_path = os.path.join(root, 'radar')

    # camera image
    img_file = glob.glob(os.path.join(cam_path, '*.png'))[0]
    img = cv2.imread(img_file)

    # image bboxes
    cam_detections_file = glob.glob(os.path.join(cam_path, '*.txt'))[0]
    cam_detections = []
    with open(cam_detections_file, 'r') as f:
        for line in f:
            parsed_line = line.strip().split()
            parsed_line = [float(value) if i > 0 else int(value) for i, value in enumerate(parsed_line)]
            cam_detections.append(parsed_line)

    # lidar point cloud(bin)
    pcd_file = glob.glob(os.path.join(lid_path, '*.bin'))[0]
    pcd = np.fromfile(pcd_file, dtype=np.float32)
    pcd = pcd.reshape(-1, 4)  # (N, 4) 형태로 변환

    # lidar detection results
    lid_detections = 0

    # radar positions
    rad_file = glob.glob(os.path.join(rad_path, '*.txt'))[0]
    df = pd.read_csv(rad_file, delim_whitespace=True)
    rad_xy = df[['/position_x', '/position_y']].to_numpy() # KATRI G80 data


    # return img, pcd[:, :3], positions
    return img, cam_detections, pcd, lid_detections, rad_xy


# def visualize_lidar_points_2d(points):
#     plt.figure(figsize=(10, 8))
#     plt.scatter(points[:, 0], points[:, 1], s=1)  # x, y 포인트를 2D로 시각화
#     plt.xlabel('X Position')
#     plt.ylabel('Y Position')
#     plt.title('2D Visualization of Lidar Points')
#     plt.axis('equal')  # 비율 유지
#     plt.grid(True)
#     plt.show()


# def visualize_lidar_points_3d(points):
#     fig = plt.figure(figsize=(10, 8))
#     ax = fig.add_subplot(111, projection='3d')

#     # 포인트를 scatter로 시각화
#     ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1)  # x, y, z 포인트를 3D로 시각화
#     ax.set_xlabel('X Position')
#     ax.set_ylabel('Y Position')
#     ax.set_zlabel('Z Position')
#     ax.set_title('3D Visualization of Lidar Points')
#     ax.view_init(elev=20, azim=30)  # 시점 조정

#     plt.show()


# ---



# ---

## Data loading
root = '/home/wise/sjy/241112_autocal/data'

# cam_path, lid_path, rad_path = parse_filepath(root)
# cam, lid, rad = load_CLR_data(cam_path, lid_path, rad_path)
img, cam_detections, pcd, lid_detections, rad_xy = load_CLR_data(root)

img_w = img.shape[1]
img_h = img.shape[0]

# visualize_lidar_points_3d(lid)
# bbox_list = detect_cam_objects(cam, vis=True)

print('data loaded')


## Data association : 일단 수동으로, 카메라 파라미터도 없어서 임의로
focal_length = 800  
center = (1920/2, 1080/2) 
camera_matrix = np.array([[focal_length, 0, center[0]],
                           [0, focal_length, center[1]],
                           [0, 0, 1]])
distortion_coeffs = np.zeros((5, 1))  # 왜곡 계수 (k1, k2, p1, p2)

cam_points = np.array([
    [0.132031, 0.572685, 0.214062, 0.317593],  # 14
    [0.332552, 0.565278, 0.141146, 0.256481],  # 13
    [0.48099, 0.580556, 0.0869792, 0.135185],  # 10
    [0.589323, 0.510185, 0.0838542, 0.155556], # 9
    [0.431771, 0.541667, 0.040625, 0.0777778], # 8
])

cam_points_xyxy = []
for p in cam_points:
    cx, cy, w, h = p
    
    pic_x = int(cx * img_w)
    pic_y = int((cy+(h/2))*img_h)
    cam_points_xyxy.append([pic_x, pic_y])
cam_points_xyxy = np.array(cam_points_xyxy, dtype=np.float32)

# bbox 점 시각화
for po in cam_points_xyxy:
    img = cv2.circle(img, (int(po[0]), int(po[1])), 5, (0, 0, 255), -1)
cv2.imshow('bbox foot points', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

rad_points = np.array([
    [8.0, 3.3],    # 4
    [12.2, 2.1],   # 14
    [14.4, -0.1],  # 18
    [19.2, -2.3],  # 28
    [17.6, -6.3],  # 26
])
rad_z0 = np.zeros((rad_points.shape[0], 1))
rad_points = np.hstack((rad_points, rad_z0))


print('points')


## calibration

# 1) C->R
success, rvec, tvec = cv2.solvePnP(rad_points, cam_points_xyxy, camera_matrix, distortion_coeffs)
R_, _ = cv2.Rodrigues(rvec)
print('R:', R_)
print('t:', tvec)


# rad_point_camera = R_ @ rad_points + tvec.flatten()  # (3,)
rad_point_camera = R_ @ rad_points[0] + tvec.flatten()  # (3,)

height = 3.0  # 높이 3m
top_point = np.array([rad_point_camera[0], rad_point_camera[1], height], dtype=np.float32)
image_base, _ = cv2.projectPoints(rad_point_camera.reshape(1, 1, 3), np.zeros(3), np.zeros(3), camera_matrix, distortion_coeffs)
image_top, _ = cv2.projectPoints(top_point.reshape(1, 1, 3), np.zeros(3), np.zeros(3), camera_matrix, distortion_coeffs)

# 4. 이미지 위에 수직선 그리기
image_base_x, image_base_y = int(image_base[0][0][0]), int(image_base[0][0][1])
image_top_x, image_top_y = int(image_top[0][0][0]), int(image_top[0][0][1])

# 수직선을 그리기
cv2.line(img, (image_base_x, image_base_y), (image_top_x, image_top_y), color=(255, 0, 0), thickness=2)

# rx, ry = int(rad2img[0][0][0]), int(rad2img[0][0][1])
# cv2.circle(img, (rx,ry), radius=5, color=(255,0,0), thickness=2)
cv2.imshow('Projected radar point', img)
cv2.waitKey(0)
cv2.destroyAllWindows()


## Visualization
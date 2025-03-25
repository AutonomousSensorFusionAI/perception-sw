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
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
#import open3d
# import torch
# from pathlib import Path
# from utils.utils import load_CLR_data
from utils import utils


# ---



# ---

## Data loading
root = '/home/wise/sjy/241112_autocal/data'

img, cam_detections, pcd, lid_detections, rad_xy = utils.load_CLR_data(root)

img_w = img.shape[1]
img_h = img.shape[0]

print('data loaded')


## Data association : 일단 수동으로, 카메라 파라미터도 없어서 임의로
fx = 1000  # =fy
cx = 960   
cy = 540   
camera_matrix = np.array([[fx, 0, cx],
                           [0, fx, cy],
                           [0, 0, 1]],dtype=np.float32)

distortion_coeffs = np.zeros((5, 1))  # 왜곡 계수 (k1, k2, p1, p2)

#1
# cam_points = np.array([
#     [0.132031, 0.572685, 0.214062, 0.317593],  # 14
#     [0.332552, 0.565278, 0.141146, 0.256481],  # 13
#     [0.48099, 0.580556, 0.0869792, 0.135185],  # 10
#     [0.589323, 0.510185, 0.0838542, 0.155556], # 9
#     [0.698438 ,0.56713 ,0.0729167 ,0.0824074], # 11
# ])

# lid_points = np.array([
#     [7.3, 5.8, 0],
#     [10.2, 2.8, 0],
#     [13.2, 0.2, 0],
#     [20.0, -3.5, 0],
#     [18.3, -6.8, 0],
# ], dtype=np.float32)

# rad_points = np.array([
#     [8.0, 3.3],    # 4
#     [12.2, 2.1],   # 14
#     [14.4, -0.1],  # 18
#     [19.2, -2.3],  # 28
#     [20.2, -9.9],  # 33
# ])
# rad_z0 = np.zeros((rad_points.shape[0], 1))
# rad_points = np.hstack((rad_points, rad_z0))


cam_points = np.array([
[0.350781 ,0.546296 ,0.0682292 ,0.0740741],
[0.423177 ,0.539352 ,0.0390625 ,0.0583333],
[0.483854 ,0.631481 ,0.128125 ,0.187037],
[0.603906 ,0.580093 ,0.102604 ,0.110185],
[0.752865 ,0.659722 ,0.213021 ,0.239815],
[0.0705729 ,0.529167 ,0.0838542 ,0.0805556],
[0.792969 ,0.575 ,0.133854 ,0.103704],
])

lid_points = np.array([
    [23.8378, 5.4797],
    [28.1516, 2.9522],
    [10.6583, 0.0823],
    [17.7076, -3.1054],
    [9.0397, -3.5654],
    [25.2591, 18.2836],
    [12.9065, -6.3471],
], dtype=np.float32)

# nearest_rad = utils.find_nearest_points(lid_points, rad_xy)
# print(nearest_rad)
lid_z0 = np.zeros((lid_points.shape[0], 1))
lid_points_0 = np.hstack((lid_points, lid_z0))


rad_points = np.array([
    [25. ,  5.9],
    [24.2,  4.1],
    [10.4,  0.1],
    [16.2, -2.7],
    [ 8.6, -3.1],
    [25. , 18.1],
    [14.4, -4.3],
])
rad_z0 = np.zeros((rad_points.shape[0], 1))
rad_points = np.hstack((rad_points, rad_z0))


boxed = utils.draw_bounding_boxes(img, cam_points)
cv2.imshow('boxes', boxed)
cv2.waitKey(0)
cv2.destroyAllWindows()


cam_points_xyxy = []
for p in cam_points:
    cx, cy, w, h = p
    
    pic_x = int(cx * img_w)
    pic_y = int((cy+(h/2))*img_h)
    cam_points_xyxy.append([pic_x, pic_y])
cam_points_xyxy = np.array(cam_points_xyxy, dtype=np.float32)


## calibration

# 1) R->C
_, RC_r, RC_t = cv2.solvePnP(rad_points, cam_points_xyxy, camera_matrix, distortion_coeffs)
RC_R, _ = cv2.Rodrigues(RC_r)
print('RC_R:', RC_R)
print('RC_t:', RC_t)

# 2) L->C
_, LC_r, LC_t = cv2.solvePnP(lid_points_0, cam_points_xyxy, camera_matrix, distortion_coeffs)
LC_R, _ = cv2.Rodrigues(LC_r)
print('LC_R:', LC_R)
print('LC_t:', LC_t)

# 3) R->L




## Visualization

# lidar
lid_points_homogeneous = np.hstack((lid_points, np.ones((lid_points.shape[0], 1))))  # 3D 포인트를 동차 좌표로 변환
lid2cam_coordinates = (LC_R @ lid_points_homogeneous[:, :3].T).T + LC_t.T  # 회전 후 변환
lid2cam_homogeneous = camera_matrix @ lid2cam_coordinates.T  # 카메라 매트릭스와 곱하기
lid2cam_points = lid2cam_homogeneous[:2, :] / lid2cam_homogeneous[2, :]  # 동차 좌표를 정규화

img_li2cam = utils.draw_points('lid2cam', img, lid2cam_points.T)


pcd = pcd[pcd[:, 0] >= 0] # x>0
pcd = pcd[pcd[:, 2] >= -1.4] # 지면
pcd = pcd[pcd[:, 2] <= 1.2] # 
pcd[:, 2] += 1.6 # z값 수정
pcd[:, 0] += 0.9 # x값 수정

lid_points_homogeneous = np.hstack((pcd, np.ones((pcd.shape[0], 1))))  # 3D 포인트를 동차 좌표로 변환
lid2cam_coordinates = (LC_R @ lid_points_homogeneous[:, :3].T).T + LC_t.T  # 회전 후 변환
lid2cam_homogeneous = camera_matrix @ lid2cam_coordinates.T  # 카메라 매트릭스와 곱하기
lid2cam_points = lid2cam_homogeneous[:2, :] / lid2cam_homogeneous[2, :]  # 동차 좌표를 정규화


img_lid2cam = utils.lid2cam_projection2(img, lid2cam_points.T)


# radar

# rad_point_camera = RC_R @ rad_points[0] + RC_t.flatten()  # (3,)
for point in rad_points:
    # 시작점 (z=0) 투영
    point_3d_start = np.array([point[0], point[1], 0]).reshape(1, -1)
    point_cam_start = (RC_R @ point_3d_start.T + RC_t).T
    point_2d_start, _ = cv2.projectPoints(point_cam_start, np.zeros(3), np.zeros(3), camera_matrix, distortion_coeffs)
    start_x, start_y = int(point_2d_start[0][0][0]), int(point_2d_start[0][0][1])

    # 끝점 (z=3) 투영
    point_3d_end = np.array([point[0], point[1], 3]).reshape(1, -1)
    point_cam_end = (RC_R @ point_3d_end.T + RC_t).T
    point_2d_end, _ = cv2.projectPoints(point_cam_end, np.zeros(3), np.zeros(3), camera_matrix, distortion_coeffs)
    end_x, end_y = int(point_2d_end[0][0][0]), int(point_2d_end[0][0][1])

    # 이미지 크기 내에 있는지 확인 후 선 그리기
    if (0 <= start_x < img.shape[1] and 0 <= start_y < img.shape[0] and
        0 <= end_x < img.shape[1] and 0 <= end_y < img.shape[0]):
        cv2.line(img_lid2cam, (start_x, start_y), (end_x, end_y), color=(0, 255, 255), thickness=2)  # 노란색 선

# 결과 이미지 표시
cv2.imshow('Projected Radar Points', img_lid2cam)

cv2.waitKey(0)
cv2.destroyAllWindows()
print('done')
'''
일단 알고리즘만 생각해 보기

- 파일에서 데이터 파싱 : img, pcd, rad, imgdet, pcddet, org_ext
- cal 포인트 세팅 : cam, lid, rad
- solvePnP : C->R, C->L, R->L, C->지면
- 이미지 위에 projection : R, L -> img
- 센서 간 extrinsic 터미널에 출력

'''
'''
241114, sjy

nuscenes 데이터로 projcetion, calibration 

'''

import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#import open3d
import torch
from pathlib import Path
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
fx = 800  # =fy
cx = 960   
cy = 540   
camera_matrix = np.array([[fx, 0, cx],
                           [0, fx, cy],
                           [0, 0, 1]],dtype=np.float32)

distortion_coeffs = np.zeros((5, 1))  # 왜곡 계수 (k1, k2, p1, p2)

cam_points = np.array([
    [0.132031, 0.572685, 0.214062, 0.317593],  # 14
    [0.332552, 0.565278, 0.141146, 0.256481],  # 13
    [0.48099, 0.580556, 0.0869792, 0.135185],  # 10
    [0.589323, 0.510185, 0.0838542, 0.155556], # 9
    [0.698438 ,0.56713 ,0.0729167 ,0.0824074], # 11
])
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

lid_points = np.array([
    [7.3, 5.8, 0],
    [10.2, 2.8, 0],
    [13.2, 0.2, 0],
    [20.0, -3.5, 0],
    [18.3, -6.8, 0],
], dtype=np.float32)

rad_points = np.array([
    [8.0, 3.3],    # 4
    [12.2, 2.1],   # 14
    [14.4, -0.1],  # 18
    [19.2, -2.3],  # 28
    [20.2, -9.9],  # 33
])
rad_z0 = np.zeros((rad_points.shape[0], 1))
rad_points = np.hstack((rad_points, rad_z0))


## calibration

# 1) R->C
_, RC_r, RC_t = cv2.solvePnP(rad_points, cam_points_xyxy, camera_matrix, distortion_coeffs)
RC_R, _ = cv2.Rodrigues(RC_r)
print('RC_R:', RC_R)
print('RC_t:', RC_t)

# 2) L->C
_, LC_r, LC_t = cv2.solvePnP(lid_points, cam_points_xyxy, camera_matrix, distortion_coeffs)
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



lid_points_homogeneous = np.hstack((pcd, np.ones((pcd.shape[0], 1))))  # 3D 포인트를 동차 좌표로 변환
lid2cam_coordinates = (LC_R @ lid_points_homogeneous[:, :3].T).T + LC_t.T  # 회전 후 변환
lid2cam_homogeneous = camera_matrix @ lid2cam_coordinates.T  # 카메라 매트릭스와 곱하기
lid2cam_points = lid2cam_homogeneous[:2, :] / lid2cam_homogeneous[2, :]  # 동차 좌표를 정규화

utils.lid2cam_projection1(img, lid2cam_points.T)
# utils.draw_points('lid2cam', img, lid2cam_points.T)


# radar

# rad_point_camera = RC_R @ rad_points[0] + RC_t.flatten()  # (3,)

# height = 3.0  # 높이 3m
# top_point = np.array([rad_point_camera[0], rad_point_camera[1], height], dtype=np.float32)
# # image_base, _ = cv2.projectPoints(rad_point_camera.reshape(1, 1, 3), np.zeros(3), np.zeros(3), camera_matrix, distortion_coeffs)
# # image_top, _ = cv2.projectPoints(top_point.reshape(1, 1, 3), np.zeros(3), np.zeros(3), camera_matrix, distortion_coeffs)
# image_base, _ = cv2.projectPoints(rad_point_camera.reshape(1, 1, 3), rvec, tvec, camera_matrix, distortion_coeffs)
# image_top, _ = cv2.projectPoints(top_point.reshape(1, 1, 3), rvec, tvec, camera_matrix, distortion_coeffs)
# print(rad_point_camera, top_point, image_base, image_top)

# # 수직선
# image_base_x, image_base_y = int(image_base[0][0][0]), int(image_base[0][0][1])
# image_top_x, image_top_y = int(image_top[0][0][0]), int(image_top[0][0][1])

# cv2.line(img, (image_base_x, image_base_y), (image_top_x, image_top_y), color=(255, 0, 0), thickness=2)

# # rx, ry = int(rad2img[0][0][0]), int(rad2img[0][0][1])
# # cv2.circle(img, (rx,ry), radius=5, color=(255,0,0), thickness=2)
# cv2.imshow('Projected radar point', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


print('done')
'''
일단 알고리즘만 생각해 보기

- 파일에서 데이터 파싱 : img, pcd, rad, imgdet, pcddet, org_ext
- cal 포인트 세팅 : cam, lid, rad
- solvePnP : C->R, C->L, R->L, C->지면
- 이미지 위에 projection : R, L -> img
- 센서 간 extrinsic 터미널에 출력

'''
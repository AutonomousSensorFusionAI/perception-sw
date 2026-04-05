'''
241113, sjy
gathered utility functions here
'''

import os
import glob
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

# from mpl_toolkits.mplot3d import Axes3D
# import torch
# from pathlib import Path

def load_CLR_data1(root):

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
    return img, cam_detections, pcd[:, :3], lid_detections, rad_xy


def load_CLR_data(root):

    # cam_path = os.path.join(root, 'camera')
    # lid_path = os.path.join(root, 'lidar')
    # rad_path = os.path.join(root, 'radar')

    # camera image
    img_file = glob.glob(os.path.join(root, '*.png'))[0]
    img = cv2.imread(img_file)

    # image bboxes
    # cam_detections_file = glob.glob(os.path.join(root, '*.txt'))[0]
    # cam_detections = []
    # with open(cam_detections_file, 'r') as f:
    #     for line in f:
    #         parsed_line = line.strip().split()
    #         parsed_line = [float(value) if i > 0 else int(value) for i, value in enumerate(parsed_line)]
    #         cam_detections.append(parsed_line)
    cam_detections=0

    # lidar point cloud(bin)
    pcd_file = glob.glob(os.path.join(root, '*.bin'))[0]
    pcd = np.fromfile(pcd_file, dtype=np.float32)
    pcd = pcd.reshape(-1, 4)  # (N, 4) 형태로 변환

    # lidar detection results
    lid_detections = 0

    # radar positions
    rad_file = glob.glob(os.path.join(root, '*.txt'))[1]
    df = pd.read_csv(rad_file, delim_whitespace=True)
    rad_xy = df[['/position_x', '/position_y']].to_numpy() # KATRI G80 data


    return img, cam_detections, pcd[:, :3], lid_detections, rad_xy

def draw_bounding_boxes(image, boxes):
    """
    Draws bounding boxes on the given image.

    Parameters:
        image (np.array): The image on which to draw.
        boxes (list): List of bounding boxes, each box is [x_center, y_center, width, height] (relative coordinates).
        color (tuple): Color of the bounding box in BGR format.

    Returns:
        np.array: The image with drawn bounding boxes.
    """
    color=(0, 255, 0)

    h, w, _ = image.shape  # 이미지의 높이와 너비

    for box in boxes:
        x_center, y_center, width, height = box

        x1 = int((x_center - width / 2) * w)  
        y1 = int((y_center - height / 2) * h)  
        x2 = int((x_center + width / 2) * w)  
        y2 = int((y_center + height / 2) * h)  

        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

    return image

def draw_points(windowname, image, points, color=(0,0,255)):
        
    for po in points:
        image = cv2.circle(image, (int(po[0]), int(po[1])), 5, color, -1)

    cv2.imshow(windowname, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def lid2cam_projection1(img, pcd):
    # img = cv2.imread(image_file)
    img_mapped = img.copy()
    img_h, img_w = img.shape[:2]

    # 거리 계산 (z 값 사용)
    distances = np.linalg.norm(pcd[:, :3], axis=1)  # 각 포인트의 거리 계산
    x_normalized = (distances - np.min(distances)) / (np.max(distances) - np.min(distances))  # 정규화
    colors = plt.cm.magma(x_normalized)  # 색상 맵핑

    # 프로젝션된 포인트 (여기서는 간단히 x, y를 사용)
    x = pcd[:, 0]
    y = pcd[:, 1]
    # z = pcd[:, 2]
    angles = np.arctan2(y, x)  # y/x의 아크탄젠트로 각도 계산 (라디안 단위)

    # 전방 범위에 해당하는 포인트 필터링
    front_angle_range=(-np.pi/4, np.pi/4)
    front_mask = (angles >= front_angle_range[0]) & (angles <= front_angle_range[1])
    x_front = x[front_mask]
    y_front = y[front_mask]
    colors_front = colors[front_mask]

    # 이미지에 포인트 그리기
    # for i, (ix, iy) in enumerate(zip(x, y)):
    for i, (ix, iy) in enumerate(zip(x_front, y_front)):
        ix = int(ix)  # x 좌표 정수 변환
        iy = int(iy)  # y 좌표 정수 변환
        if 0 <= ix < img_w and 0 <= iy < img_h:
            # color = (colors[i] * 255).astype(np.uint8)[:3]
            color = (colors_front[i] * 255).astype(np.uint8)[:3]
            color = (int(color[2]), int(color[1]), int(color[0]))  # BGR 형식으로 변환
            img_mapped = cv2.circle(img_mapped, (ix, iy), radius=1, color=color, thickness=2)

    img_mapped_rgb = cv2.cvtColor(img_mapped, cv2.COLOR_BGR2RGB)

    cv2.imshow('Projected LiDAR Points', img_mapped_rgb)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

import cv2
import numpy as np

def lid2cam_projection2(img, pcd):
    img_mapped = img.copy()
    img_h, img_w = img.shape[:2]

    # 2D projection된 pcd의 x, y 좌표 가져오기
    x = pcd[:, 0]
    y = pcd[:, 1]

    # 색상 설정 (기본값으로 흰색 사용)
    color = (255, 255, 255)  # 흰색 (BGR 형식)

    # 이미지에 포인트 그리기
    for ix, iy in zip(x, y):
        ix = int(ix)  # x 좌표 정수 변환
        iy = int(iy)  # y 좌표 정수 변환
        if 0 <= ix < img_w and 0 <= iy < img_h:
            img_mapped = cv2.circle(img_mapped, (ix, iy), radius=1, color=color, thickness=2)

    return img_mapped

    # # 결과 이미지 출력
    # cv2.imshow('Projected LiDAR Points', img_mapped)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()



def find_nearest_points(coords, reference_coords):
    # coords: (N, 2) shape ndarray
    # reference_coords: (M, 2) shape ndarray
    
    # 두 배열 사이의 모든 유클리드 거리를 계산 (N, M 형태의 거리 행렬 생성)
    distances = cdist(coords, reference_coords, metric='euclidean')
    
    # 각 좌표에 대해 가장 가까운 레퍼런스 좌표의 인덱스를 찾음
    nearest_indices = np.argmin(distances, axis=1)
    
    # 가장 가까운 레퍼런스 좌표들을 반환
    nearest_points = reference_coords[nearest_indices]
    return nearest_points

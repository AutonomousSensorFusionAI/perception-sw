'''
250124, sjy3

command:
python project_pcd_on_img.py --rt_yaml 241106_calibration_w_makefile/241030_센서캘리브레이션/src/test_out1.yaml --pcd_file 241106_calibration_w_makefile/241030_센서캘리브레이션/250124_test-data/322-514952000.pcd --image_file 241106_calibration_w_makefile/241030_센서캘리브레이션/250124_test-data/4703.jpg --output_yaml calibrated_output_modified.yaml

'''

import numpy as np
import open3d as o3d
import cv2
import argparse
import yaml
from pyquaternion import Quaternion

# 회전 행렬과 변환 벡터를 쿼터니언으로 변환하는 함수
def rotation_matrix_to_quaternion(rotation_matrix):
    # 회전 행렬을 쿼터니언으로 변환
    q = Quaternion(matrix=rotation_matrix)
    return q

# YAML 파일에서 회전 행렬, 변환 벡터 로드
def load_extrinsic_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)

    rotation_matrix = np.array(data['rotation_matrix'], dtype=np.float32)
    translation_vector = np.array(data['translation_vector'], dtype=np.float32)

    return rotation_matrix, translation_vector

# 라이다 포인트 클라우드를 2D 이미지 좌표로 투영
def project_lidar_to_image(pcd_file, rotation_matrix, translation_vector, camera_matrix):
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)
    
    # 라이다 포인트의 x축 기준 -y쪽 180도 범위 필터링
    filtered_points = points[(points[:, 1] < 0) & (points[:, 0] > 0)]
    
    # 회전 및 변환 적용
    transformed_points = np.dot(filtered_points, rotation_matrix.T) + translation_vector.T
    
    # 3D -> 2D 투영 (카메라 매트릭스 이용)
    projected_points = camera_matrix @ transformed_points.T
    projected_points /= projected_points[2, :]  # Homogeneous 좌표에서 스케일링
    
    # 2D 이미지 좌표로 변환
    image_points = projected_points[:2, :].T
    
    return image_points

# 이미지 위에 투영된 포인트 표시
def display_projected_points(image_file, projected_points):
    image = cv2.imread(image_file)
    
    for point in projected_points:
        x, y = int(point[0]), int(point[1])
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            cv2.circle(image, (x, y), 2, (0, 0, 255), -1)
    
    cv2.imshow("Projected Points", image)
    cv2.waitKey(1)  # 키 입력 대기 없이 바로 업데이트

# 키보드 방향키로 회전 값 수정
def update_rotation(rotation_matrix, key):
    rotation = Quaternion(matrix=rotation_matrix)
    
    delta = 0.25
    if key == 81:  # 왼쪽 방향키 (yaw+)
        rotation = rotation * Quaternion(axis=[0, 0, 1], angle=np.radians(delta))
    elif key == 83:  # 오른쪽 방향키 (yaw-)
        rotation = rotation * Quaternion(axis=[0, 0, 1], angle=-np.radians(delta))
    elif key == 82:  # 위쪽 방향키 (pitch+)
        rotation = rotation * Quaternion(axis=[1, 0, 0], angle=np.radians(delta))
    elif key == 84:  # 아래쪽 방향키 (pitch-)
        rotation = rotation * Quaternion(axis=[1, 0, 0], angle=-np.radians(delta))
    elif key == 2424832:  # Shift+왼쪽 방향키 (roll+)
        rotation = rotation * Quaternion(axis=[0, 1, 0], angle=np.radians(delta))
    elif key == 2555904:  # Shift+오른쪽 방향키 (roll-)
        rotation = rotation * Quaternion(axis=[0, 1, 0], angle=-np.radians(delta))
    
    return rotation.rotation_matrix

# 최종 회전 값을 YAML 파일로 저장
def save_extrinsic_to_yaml(output_yaml, rotation_matrix, translation_vector):
    data = {
        'rotation_matrix': rotation_matrix.tolist(),
        'translation_vector': translation_vector.tolist()
    }
    with open(output_yaml, 'w') as file:
        yaml.dump(data, file)

# 메인 함수
def main():
    # argparse를 사용하여 입력 및 출력 파일명 CLI로 받기
    parser = argparse.ArgumentParser(description="Project lidar points onto an image and modify extrinsics")
    parser.add_argument('--rt_yaml', type=str, required=True, help='Path to the YAML file containing R|t (calibrated_output.yaml)')
    parser.add_argument('--pcd_file', type=str, required=True, help='Path to the lidar PCD file (lidar_data.pcd)')
    parser.add_argument('--image_file', type=str, required=True, help='Path to the image file to project lidar points onto (image.png)')
    # parser.add_argument('--output_yaml', type=str, required=True, help='Output YAML file to save modified extrinsic parameters')

    args = parser.parse_args()

    # YAML에서 회전 행렬과 변환 벡터 로드
    rotation_matrix, translation_vector = load_extrinsic_from_yaml(args.rt_yaml)
    
    # 카메라 내부 파라미터 예시
    fx, fy, cx, cy = 800, 800, 320, 240
    camera_matrix = np.array([[fx, 0, cx], 
                              [0, fy, cy], 
                              [0, 0, 1]], dtype=np.float32)

    # 초기 PCD 투영
    projected_points = project_lidar_to_image(args.pcd_file, rotation_matrix, translation_vector, camera_matrix)
    display_projected_points(args.image_file, projected_points)

    while True:
        key = cv2.waitKey(0)  # 키 입력 대기
        
        if key == ord('q'):  # 'q'를 누르면 종료
            save_extrinsic_to_yaml(args.output_yaml, rotation_matrix, translation_vector)
            cv2.destroyAllWindows()
            break
        
        # 키보드 입력에 따라 회전 값 수정
        if key in [81, 83, 82, 84, 2424832, 2555904]:  # 방향키
            rotation_matrix = update_rotation(rotation_matrix, key)
            # PCD 투영 업데이트
            projected_points = project_lidar_to_image(args.pcd_file, rotation_matrix, translation_vector, camera_matrix)
            display_projected_points(args.image_file, projected_points)

if __name__ == "__main__":
    main()

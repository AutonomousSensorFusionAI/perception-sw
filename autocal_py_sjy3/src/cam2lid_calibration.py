'''
250124, sjy3
2d, 3d points calibration

command:
python3 cam2lid_calibration.py \
    --input_yaml test_in.yaml \
    --output_yaml calibrated_output.yaml

'''
import cv2
import numpy as np
import yaml
import argparse
from datetime import datetime

# YAML 파일에서 2D 및 3D 포인트와 카메라 intrinsic 파라미터를 파싱하는 함수
def load_points_and_intrinsics_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    
    # 2D 포인트와 3D 포인트 파싱
    points_2d = np.array(data['2d_points'], dtype=np.float32)
    points_3d = np.array(data['3d_points'], dtype=np.float32)
    
    # 카메라의 intrinsic 파라미터 파싱
    intrinsic = data['camera_intrinsics']
    fx = intrinsic['fx']
    fy = intrinsic['fy']
    cx = intrinsic['cx']
    cy = intrinsic['cy']
    
    # 카메라 매트릭스 (3x3 행렬)
    camera_matrix = np.array([[fx, 0, cx], 
                              [0, fy, cy], 
                              [0, 0, 1]], dtype=np.float32)

    return points_2d, points_3d, camera_matrix

# 2D-3D 대응점을 이용하여 R|t 계산하는 함수
def compute_rt_matrix(points_2d, points_3d, camera_matrix):
    # 2D, 3D 대응점이 최소 4개 이상이어야 합니다.
    if len(points_2d) < 4 or len(points_3d) < 4:
        raise ValueError("At least 4 points are needed for the transformation.")

    # R|t 계산 (PnP 문제 해결)
    _, rvec, tvec = cv2.solvePnP(points_3d, points_2d, camera_matrix, distCoeffs=None)

    # 회전 벡터를 회전 행렬로 변환
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    
    # R|t 결과 반환
    return rotation_matrix, tvec

# 계산된 R|t를 YAML 파일로 저장하는 함수
def save_rt_to_yaml(rotation_matrix, translation_vector, filename):
    data = {
        'rotation_matrix': rotation_matrix.tolist(),
        'translation_vector': translation_vector.tolist()
    }
    with open(filename, 'w') as file:
        yaml.dump(data, file)

# 메인 함수
def main():
    # argparse를 사용하여 입력 및 출력 파일명 CLI로 받기
    parser = argparse.ArgumentParser(description="Compute R|t from 2D-3D point correspondences")
    parser.add_argument('--input_yaml', type=str, required=True, help='Input YAML file containing 2D/3D points and camera intrinsics')
    parser.add_argument('--output_yaml', type=str, required=True, help='Output YAML file to save the calibrated R|t')

    args = parser.parse_args()

    # 2D, 3D 포인트와 카메라 내부 파라미터 로딩
    points_2d, points_3d, camera_matrix = load_points_and_intrinsics_from_yaml(args.input_yaml)

    # 두 센서 간 R|t 계산
    rotation_matrix, translation_vector = compute_rt_matrix(points_2d, points_3d, camera_matrix)

    # 계산된 R|t 저장
    save_rt_to_yaml(rotation_matrix, translation_vector, args.output_yaml)

    print(f"Calibration saved to {args.output_yaml}")

if __name__ == "__main__":
    main()

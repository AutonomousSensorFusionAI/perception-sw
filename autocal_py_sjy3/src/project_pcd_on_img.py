'''
250124, sjy3
project pcd on image

command:
python3 project_pcd_on_img.py \
    --rt_yaml calibrated_output.yaml \
    --pcd_file ../250124_test-data/322-514952000.pcd \
    --image_file ../250124_test-data/4735.jpg 

'''
import cv2
import numpy as np
import open3d as o3d
import yaml
import argparse

# YAML 파일에서 R|t를 파싱하는 함수
def load_rt_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    
    rotation_matrix = np.array(data['rotation_matrix'], dtype=np.float32)
    translation_vector = np.array(data['translation_vector'], dtype=np.float32)
    
    return rotation_matrix, translation_vector

def project_lidar_to_image(pcd_file, rotation_matrix, translation_vector, camera_matrix):
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)  # (N, 3) 형태의 라이다 포인트들
    
    # 라이다 포인트의 x축 기준 -y쪽 180도 범위 필터링
    # filtered_points = points[(points[:, 1] < 0) & (points[:, 0] > 0)]  # x>0, y<0 필터링
    filtered_points = points[(points[:, 1] < 0)]  # x>0, y<0 필터링
    # filtered_points = points
    
    # translation_vector를 (1, 3) 형태로 변환
    transformed_points = np.dot(filtered_points, rotation_matrix.T) + translation_vector.T  # (78404, 3) + (1, 3)
    
    projected_points = camera_matrix @ transformed_points.T  # (3, N) 형태로 변환
    projected_points /= projected_points[2, :]  # Homogeneous 좌표에서 스케일링
    
    image_points = projected_points[:2, :].T  # (N, 2)
    
    return image_points

def display_projected_points(image_file, projected_points):
    image = cv2.imread(image_file)
    
    for point in projected_points:
        x, y = int(point[0]), int(point[1])
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            cv2.circle(image, (x, y), 2, (0, 0, 255), -1)  # 빨간색 점
    
    # 이미지를 창에 표시
    cv2.imshow("Projected Points", image)
    cv2.waitKey(0)  # 키 입력 대기
    cv2.destroyAllWindows()  # 창 닫기

# 메인 함수
def main():
    # argparse를 사용하여 입력 및 출력 파일명 CLI로 받기
    parser = argparse.ArgumentParser(description="Project lidar points onto an image using R|t transformation")
    parser.add_argument('--rt_yaml', type=str, required=True, help='Path to the YAML file containing R|t (calibrated_output.yaml)')
    parser.add_argument('--pcd_file', type=str, required=True, help='Path to the lidar PCD file (lidar_data.pcd)')
    parser.add_argument('--image_file', type=str, required=True, help='Path to the image file to project lidar points onto (image.png)')

    args = parser.parse_args()

    # R|t 로드
    rotation_matrix, translation_vector = load_rt_from_yaml(args.rt_yaml)

    # 카메라 내/외부 파라미터 (예시, 수정 필요)
    fx, fy, cx, cy = 800, 800, 320, 240  # 카메라 내부 파라미터 (예시)
    camera_matrix = np.array([[fx, 0, cx], 
                              [0, fy, cy], 
                              [0, 0, 1]], dtype=np.float32)

    projected_points = project_lidar_to_image(args.pcd_file, rotation_matrix, translation_vector, camera_matrix)
    
    display_projected_points(args.image_file, projected_points)

if __name__ == "__main__":
    main()

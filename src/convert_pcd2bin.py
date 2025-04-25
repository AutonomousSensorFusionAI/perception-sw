'''
250122, sjy

'''
import open3d as o3d
import numpy as np


def pcd_to_bin(pcd_file, bin_file):

    pcd = o3d.io.read_point_cloud(pcd_file)
    
    points = np.asarray(pcd.points)

    if len(pcd.colors) > 0:  # colors 배열이 존재할 때
        # intensity 정보 (color를 intensity로 간주)
        intensity = np.asarray(pcd.colors)[:, 0]  # R값만 사용한다고 가정 (혹은 다른 채널 선택 가능)
    else:
        # intensity가 없다면 기본적으로 0으로 설정
        intensity = np.zeros(len(points))

    # x, y, z, intensity 값을 하나의 배열로 합치기
    data = np.hstack((points, intensity.reshape(-1, 1)))

    # bin 파일로 변환하여 저장 (float32 타입으로 저장)
    data.astype(np.float32).tofile(bin_file)
    print(f"Saved binary file: {bin_file}")



# 예시 사용법
pcd_file = '/home/wise/workspace/ws_sensors/369-895235000.pcd'  
bin_file = '/home/wise/Documents/sjy/250122_from_bag/output_images_and_pcds/4735.bin'  

print(f'source: {pcd_file}')
print(f'target: {bin_file}')
print(f'-------')
pcd_to_bin(pcd_file, bin_file)

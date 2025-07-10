'''
250122, sjy

'''
import open3d as o3d
import numpy as np


def pcd_to_bin(pcd_file, bin_file):

    pcd = o3d.io.read_point_cloud(pcd_file)
    
    points = np.asarray(pcd.points)

    if len(pcd.colors) > 0:  # colors 배열이 존재할 때
        intensity = np.asarray(pcd.colors)[:, 0]  # R값만 사용한다고 가정 (혹은 다른 채널 선택 가능)
    else:
        intensity = np.zeros(len(points))

    data = np.hstack((points, intensity.reshape(-1, 1)))

    data.astype(np.float32).tofile(bin_file)
    print(f"Saved binary file: {bin_file}")



# 예시
pcd_file = '/home/wise/workspace/ws_sensors/369-895235000.pcd'  
bin_file = '/home/wise/Documents/sjy/250122_from_bag/output_images_and_pcds/4735.bin'  

print(f'source: {pcd_file}')
print(f'target: {bin_file}')
print(f'-------')
pcd_to_bin(pcd_file, bin_file)

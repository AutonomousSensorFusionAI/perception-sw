'''
250124, sjy3

for parse calibration point manually
'''

import open3d as o3d
import numpy as np

def visualize_point_cloud_with_customization(pcd_file):
    # PCD 파일 읽기
    pcd = o3d.io.read_point_cloud(pcd_file)
    print(pcd)
    print("Point cloud dimensions:", len(pcd.points))
    
    # 뷰어 설정
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Customized Point Cloud Viewer", width=800, height=600)

    # 포인트 클라우드 추가
    vis.add_geometry(pcd)

    # 뷰어 옵션 커스터마이징
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.1, 0.1, 0.1])  # 어두운 배경
    opt.point_size = 2.0  # 포인트 크기
    opt.show_coordinate_frame = True  # 축 표시 (x: red, y: green, z: blue)

    # 그리드 생성
    grid = create_grid(1.0, 100, 10.0)  # 1m 간격, 10m마다 두꺼운 선
    vis.add_geometry(grid)

    # 원점 표시
    origin_marker = create_origin_marker(radius=0.1, resolution=20)
    vis.add_geometry(origin_marker)

    # 렌더링
    vis.run()
    vis.destroy_window()


def create_grid(step, size, bold_every):
    """그리드 생성 함수
    step: 그리드 간격 (m)
    size: 그리드의 총 범위 (m)
    bold_every: 두꺼운 선의 간격 (m)
    """
    lines = []
    colors = []
    half_size = size / 2

    # x, y 방향으로 그리드 생성
    for i in range(-int(half_size / step), int(half_size / step) + 1):
        x = i * step
        color = [0.5, 0.5, 0.5]  # 기본 그리드 색상 (회색)
        if i % int(bold_every / step) == 0:
            color = [1.0, 1.0, 1.0]  # 두꺼운 그리드 색상 (흰색)
        # 수직선 (y 고정)
        lines.append(([x, -half_size, 0], [x, half_size, 0]))
        colors.append(color)
        # 수평선 (x 고정)
        lines.append(([-half_size, x, 0], [half_size, x, 0]))
        colors.append(color)

    # Open3D의 LineSet으로 변환
    points = []
    for line in lines:
        points.extend(line)
    points = np.array(points)
    lines = [[i, i + 1] for i in range(0, len(points), 2)]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def create_origin_marker(radius=0.1, resolution=20):
    """원점에 구체를 생성하여 표시"""
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=resolution)
    sphere.translate((0, 0, 0))  # 원점에 배치
    sphere.paint_uniform_color([1.0, 0.0, 0.0])  # 빨간색으로 표시
    return sphere

# Example usage
if __name__ == '__main__':
    # Replace with your file path and type
    file_path = '/home/wise/Documents/sjy/241106_calibration_w_makefile/241030_센서캘리브레이션/250124_test-data/322-514952000.pcd'
    file_type = file_path.split('.')[-1] # pcd or bin

    # Visualize with grid and axis information
    visualize_point_cloud_with_customization(file_path)
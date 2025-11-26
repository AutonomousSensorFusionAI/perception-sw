'''
250124, sjy3
click and get pixel coord
'''

import cv2
import matplotlib.pyplot as plt

# 이미지 파일 경로
image_path = '/home/wise/Documents/sjy/241106_calibration_w_makefile/241030_센서캘리브레이션/250124_test-data/4703.jpg'  


# 이미지 읽기
image = cv2.imread(image_path)
print("To close the window, press 'q'.")

# 클릭된 좌표를 저장할 변수
clicked_points = []  # 클릭된 좌표들 저장
counter = 1  # 클릭 번호

# 마우스 클릭 이벤트 콜백 함수
def click_event(event, x, y, flags, param):
    global counter
    if event == cv2.EVENT_LBUTTONDOWN:
        # 빨간 점 찍기
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
        
        # 번호 텍스트 추가
        cv2.putText(image, str(counter), (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 클릭된 좌표와 번호 저장
        clicked_points.append((counter, x, y))
        
        # 터미널에 좌표 출력
        print(f"Point {counter}: Coordinates: {x}, {y}")
        
        # 결과 창 다시 띄우기
        cv2.imshow("Click to Find Pixel Coordinates", image)
        
        # 번호 증가
        counter += 1

# 창 이름과 이미지를 띄우기
cv2.imshow("Click to Find Pixel Coordinates", image)

# 마우스 이벤트 처리
cv2.setMouseCallback("Click to Find Pixel Coordinates", click_event)


# 'q'를 눌러 창을 닫기
while True:
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):  # 'q' 눌러 종료
        break

# 창 닫기
cv2.destroyAllWindows()
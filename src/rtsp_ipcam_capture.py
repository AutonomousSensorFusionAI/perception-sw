'''
for intrinsic calibration
capture rtsp camera stream
'''

import cv2
import time

def get_current_timestamp():
    return time.strftime("%y%m%d_%H%M%S") + ".jpg"


def main():
    # rtsp_url = "rtsp://admin:1234@192.168.0.223:554/stream protocols=udp latency=0"  
    #rtsp_url = "rtsp://admin:tlJwpbo6@192.168.0.220:554/stream protocols=udp latency=0"
    rtsp_url = "rtsp://admin:tlJwpbo6@192.168.0.161:554/stream protocols=udp latency=0"
    cap = cv2.VideoCapture(rtsp_url)
    
    if not cap.isOpened():
        print("Error: Cannot open RTSP stream!")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Blank frame received!")
            break
        
        height, width, _ = frame.shape
        window_name = f"Camera Stream - {width}x{height}"
        cv2.imshow(window_name, frame)
        
        key = cv2.waitKey(30) & 0xFF
        if key == 13:  # Enter 키 입력 시 캡처
            filename = get_current_timestamp()
            cv2.imwrite(filename, frame)
            print(f"Saved frame as {filename}")
        elif key == 27:  # ESC 키 입력 시 종료
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
import numpy as np
from time import sleep

# --- 카메라 초기화 ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()
sleep(2)

# --- YOLO 모델 로드 ---
model = YOLO("yolov8n.pt")

# --- 메인 루프 ---
try:
    while True:
        # 이미지 캡처 (numpy 배열)
        frame = picam2.capture_array()

        # 이미지 회전 (180도)
        frame_rotated = cv2.rotate(frame, cv2.ROTATE_180)

        # YOLO 객체 인식
        results = model.predict(frame_rotated, verbose=False)

        # 결과 프레임에 박스 그리기
        annotated_frame = results[0].plot()

        # 결과 이미지를 파일로 저장
        cv2.imwrite("/home/pi/yolo_output.jpg", annotated_frame)

        sleep(0.5)

except KeyboardInterrupt:
    print("종료 중...")

finally:
    picam2.stop()
    cv2.destroyAllWindows()

#!/usr/bin/env python3
import os
import time
import cv2
from picamera2 import Picamera2

def main():
    # 실행 파일 폴더 경로
    current_dir = os.path.dirname(os.path.abspath(__file__))
    video_path = os.path.join(current_dir, "test_record.avi")
    print(f"Recording video to: {video_path}")

    # Picamera2 초기화
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(camera_config)
    picam2.start()

    # 프레임 크기 가져오기
    frame = picam2.capture_array()
    height, width = frame.shape[:2]

    # VideoWriter 초기화
    video_writer = cv2.VideoWriter(
        video_path,
        cv2.VideoWriter_fourcc(*"MJPG"),
        30.0,           # FPS
        (width, height)
    )

    print("🎬 Recording started. Press Ctrl+C to stop.")
    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            # 180도 회전
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            video_writer.write(frame)
            time.sleep(0.03)  # 약 30 FPS
    except KeyboardInterrupt:
        print("\n🛑 Recording stopped.")
    finally:
        video_writer.release()
        picam2.stop()

if __name__ == "__main__":
    main()

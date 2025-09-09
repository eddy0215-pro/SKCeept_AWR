#!/usr/bin/env python3
import time
import threading
import RPi.GPIO as GPIO
import move
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import numpy as np


class DSM_Autonomous:
    def __init__(self, speed=40):
        # YOLOv8n 모델 로드
        self.model = YOLO("yolov8n.pt")

        # 카메라 초기화
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(main={"size": (640, 480)})
        self.picam2.configure(camera_config)
        self.picam2.start()

        # 실행 플래그
        self.running = True
        self.started = False
        self.use_ultrasonic = True
        self.speed = speed

        # 초음파 핀
        self.Trig = 11
        self.Echo = 8

        # 출발 관련 설정
        self.start_threshold = 30.0     # cm, 가림막 없으면 출발
        self.stable_time = 3.0          # 3초 이상 안정적이면 출발

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Trig, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.Echo, GPIO.IN)
        move.setup()

        # 비디오 저장 초기화
        self.out = None
        self.init_video_writer("/home/pi/drive_record.avi")

        # 출발 대기 스레드
        threading.Thread(target=self.sensor_start_wait_loop, daemon=True).start()

    # ───── 비디오 저장 초기화 ─────
    def init_video_writer(self, filename, fps=20.0, size=(640, 480)):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(filename, fourcc, fps, size)
        print(f"🎥 Recording started: {filename}")

    # ───── 프레임 저장 ─────
    def write_frame(self, frame):
        if self.out is not None:
            self.out.write(frame)

    # ───── 비디오 저장 종료 ─────
    def release_video_writer(self):
        if self.out is not None:
            self.out.release()
            self.out = None
            print("💾 Video recording stopped and file saved.")

    # ───── 프레임 캡처 + 전처리 ─────
    def capture_frame(self):
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    # ───── YOLO 객체 인식 ─────
    def detect_objects(self, frame):
        results = self.model(frame, verbose=False)
        names = results[0].names
        boxes = results[0].boxes
        detected = [names[int(cls)] for cls in boxes.cls]
        return detected, results[0].plot()  # 박스가 그려진 annotated_frame 반환

    # ───── 차선 인식 (실선/점선 구분) ─────
    def detect_lane(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=40, maxLineGap=10)

        solid_lines = []
        dashed_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = np.hypot(x2-x1, y2-y1)
                # 길이가 길면 실선, 짧으면 점선
                if length > 60:
                    solid_lines.append(line)
                else:
                    dashed_lines.append(line)

        return solid_lines, dashed_lines

    # ───── 차선 주석 그리기 ─────
    def draw_lanes(self, frame, solid_lines, dashed_lines):
        if solid_lines:
            for line in solid_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)  # 초록색 = 실선
        if dashed_lines:
            for line in dashed_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # 파랑 = 점선
        return frame

    # ───── 초음파 거리 읽기 ─────
    def read_distance(self):
        try:
            GPIO.output(self.Trig, GPIO.LOW)
            time.sleep(0.00005)
            GPIO.output(self.Trig, GPIO.HIGH)
            time.sleep(0.000015)
            GPIO.output(self.Trig, GPIO.LOW)

            timeout = time.time() + 0.02
            while not GPIO.input(self.Echo):
                if time.time() > timeout:
                    return float('inf')
            t1 = time.time()

            timeout = time.time() + 0.02
            while GPIO.input(self.Echo):
                if time.time() > timeout:
                    return float('inf')
            t2 = time.time()
            return (t2 - t1) * 34000 / 2
        except:
            return float('inf')

    # ───── 출발 대기 루프 ─────
    def sensor_start_wait_loop(self):
        print("🔎 Waiting for start condition (remove obstacle for 3s)...")
        stable_start = None
        while self.running and not self.started:
            if not self.use_ultrasonic:
                break
            d = self.read_distance()
            if d != float('inf') and d > self.start_threshold:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start >= self.stable_time:
                    self.started = True
                    self.use_ultrasonic = False
                    print("✅ Start condition met — starting YOLO thread")
                    threading.Thread(target=self.yolo_lane_loop, daemon=True).start()
                    break
            else:
                stable_start = None
            time.sleep(0.1)

    # ───── YOLO + 차선 기반 주행 루프 ─────
    def yolo_lane_loop(self):
        frame_count = 0
        start_time = time.time()

        while self.running:
            frame = self.capture_frame()
            frame_count += 1

            if frame_count % 3 == 0:
                try:
                    detected, annotated_frame = self.detect_objects(frame)
                    solid_lines, dashed_lines = self.detect_lane(frame)

                    # 차선 주석 추가
                    annotated_frame = self.draw_lanes(annotated_frame, solid_lines, dashed_lines)

                    # YOLO 장애물 회피
                    if "person" in detected:
                        print("👀 Person detected → stop 1s")
                        move.motorStop()
                        time.sleep(1.0)
                        self.write_frame(annotated_frame)
                        continue

                    # 차선 변경 판단
                    lane_move = 'forward'
                    if dashed_lines:
                        avg_x = np.mean([(line[0][0]+line[0][2])/2 for line in dashed_lines])
                        if avg_x < 320 - 30:
                            lane_move = 'left'
                        elif avg_x > 320 + 30:
                            lane_move = 'right'

                    move.move(self.speed,
                              lane_move if lane_move in ['left','right'] else 'forward',
                              'no', 0)

                    print(f"Detected: {detected} | Solid:{len(solid_lines)} Dashed:{len(dashed_lines)} | Move:{lane_move}")

                    # 프레임 저장
                    self.write_frame(annotated_frame)

                except Exception as e:
                    print("Error:", e)

            # FPS 출력 최적화
            if frame_count % 30 == 0:
                fps = frame_count / (time.time() - start_time)
                print(f"📷 FPS: {fps:.2f}")
                frame_count = 0
                start_time = time.time()

            time.sleep(0.05)

    # ───── 실행 / 종료 ─────
    def run(self):
        print("🚗 DSM Autonomous Driving Initialized — waiting for start")
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        self.running = False
        move.motorStop()
        self.picam2.stop()
        self.release_video_writer()
        GPIO.cleanup()
        print("🛑 DSM Autonomous Driving Stopped")


if __name__ == '__main__':
    auto = DSM_Autonomous(speed=40)
    auto.run()

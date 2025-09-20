#!/usr/bin/env python3
import time
import threading
import RPi.GPIO as GPIO
import move
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import numpy as np
import os
import datetime  # datetime 모듈 추가
import ultra

class DSM_Autonomous:
    def __init__(self, speed=40):
        # YOLOv8n 모델 로드
        self.model = YOLO("yolov8n.pt")

        # 카메라 초기화
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(main={"size": (320, 240)})
        self.picam2.configure(camera_config)
        self.picam2.set_controls({"AwbEnable": True})
        self.picam2.start()

        # 실행 플래그
        self.running = True
        self.started = False
        self.use_ultrasonic = True
        self.speed = speed

        # 출력 모드 설정 ("fb", "video", "both")
        self.outputmode = "video"

        # 초음파 핀
        self.Trig = 11
        self.Echo = 8

        # 출발 관련 설정
        self.start_threshold = 10.0     # cm, 가림막 없으면 출발
        self.stable_time = 3.0          # 3초 이상 안정적이면 출발

        # 기본 진행 방향
        self.current_direction = "no"  

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Trig, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.Echo, GPIO.IN)
        move.setup()

        # 비디오 저장 초기화
        if self.outputmode == "video":
            current_dir = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(current_dir)
            date_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.out = None
            self.init_video_writer(os.path.join(parent_dir, f"drive_record_{date_str}.avi"))

        # 주석된 프레임 저장용
        self.annotated_frame = None

        # 출발 대기 스레드
        threading.Thread(target=self.sensor_start_wait_loop, daemon=True).start()

    # ───── 비디오 저장 초기화 ─────
    def init_video_writer(self, filename, fps=30.0, size=(320, 240)):
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.out = cv2.VideoWriter(filename, fourcc, fps, size)
        print(f"🎥 Recording started: {filename}")

    def write_frame(self, frame):
        if self.out is not None:
            self.out.write(frame)

    def release_video_writer(self):
        if self.out is not None:
            self.out.release()
            self.out = None
            print("💾 Video recording stopped and file saved.")

    # ───── 프레임 캡처 + FB 출력 (320x240, RGB565) ─────
    def capture_frame(self):
        # 1. 프레임 캡처
        frame = self.picam2.capture_array()
        if frame is None:
            print("Warning: frame capture failed")
            return None

        # 2. BGRA → BGR
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

        # 3. 180도 회전
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    # ───── YOLO 객체 인식 ─────
    def detect_objects(self, frame):
        results = self.model(frame, verbose=False)
        names = results[0].names
        boxes = results[0].boxes
        detected = [names[int(cls)] for cls in boxes.cls]

        if detected:
            print(f"🟥 Detected objects: {detected}")
            
        return detected, results[0].plot()  # 박스가 그려진 annotated_frame 반환

    # ───── 왼쪽 노란 실선만 인식 ─────
    def detect_left_yellow_lane(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 노란색 범위
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # ROI: 상단 절반 + 왼쪽 절반
        height, width = frame.shape[:2]
        roi = mask[:int(height/2), :int(width/2)]

        # 모폴로지
        kernel = np.ones((5, 5), np.uint8)
        roi_clean = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)

        # 에지 검출
        edges = cv2.Canny(roi_clean, 50, 150)

        # 허프 변환 → 긴 선분만
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
                                minLineLength=60, maxLineGap=20)

        solid_lines = []
        points = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = np.hypot(x2-x1, y2-y1)
                if length > 60:
                    solid_lines.append(line)
                    points.append((x1, y1))
                    points.append((x2, y2))

        # Polynomial Fitting
        curve = None
        if len(points) > 0:
            pts = np.array(points)
            x = pts[:, 0]
            y = pts[:, 1]
            sort_idx = np.argsort(y)
            x = x[sort_idx]
            y = y[sort_idx]

            coeffs = np.polyfit(y, x, 2)
            poly = np.poly1d(coeffs)

            y_new = np.linspace(min(y), max(y), num=50, dtype=int)
            x_new = poly(y_new).astype(int)

            curve = list(zip(x_new, y_new))

        return solid_lines, curve, roi_clean, edges

    # ───── 차선 주석 ─────
    def draw_left_lane(self, frame, solid_lines, curve):
        for line in solid_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        if curve is not None:
            for i in range(len(curve)-1):
                cv2.line(frame, curve[i], curve[i+1], (0, 0, 255), 2)
        return frame

    # ───── 초음파 거리 ─────
    def read_distance(self):
        try:
            return ultra.checkdist()
        except:
            return float('inf')

    # ───── 출발 대기 루프 ─────
    def sensor_start_wait_loop(self):
        print("🔎 Waiting for start condition (remove obstacle for 3s)...")
        stable_start = None
        while self.running and not self.started:
            if not self.use_ultrasonic:
                break
            d = self.read_distance() * 100
            print(d)
            if d != float('inf') and d > self.start_threshold:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start >= self.stable_time:
                    self.started = True
                    self.use_ultrasonic = False
                    print("✅ Start condition met — starting YOLO thread")
                    threading.Thread(target=self.camera_record_loop, daemon=True).start()
                    threading.Thread(target=self.yolo_lane_loop, daemon=True).start()
                    threading.Thread(target=self.motor_loop, daemon=True).start()
                    break
            else:
                stable_start = None
            time.sleep(0.5)

    # ───── 주행녹화 루프 ─────
    def camera_record_loop(self):
        while self.running:
            frame = self.capture_frame()

            # 오버레이: YOLO 결과가 있으면 합성
            if self.annotated_frame is not None:
                frame = cv2.addWeighted(frame, 0.7, self.annotated_frame, 0.3, 0)
            
            if self.outputmode == "video":
                self.write_frame(frame)

            elif self.outputmode == "fb":
                small_w, small_h = 320, 240
                frame_small = cv2.resize(frame, (small_w, small_h))
                frame_rgb565 = cv2.cvtColor(frame_small, cv2.COLOR_BGR2BGR565)

                fb_width, fb_height = 1920, 1080
                bpp = 2
                line_length = fb_width * bpp

                x_offset = (fb_width - small_w) // 2
                y_offset = (fb_height - small_h) // 2

                try:
                    with open("/dev/fb0", "r+b") as f:
                        for row in range(small_h):
                            offset = ((y_offset + row) * line_length) + (x_offset * bpp)
                            f.seek(offset)
                            f.write(frame_rgb565[row].tobytes())
                except Exception as e:
                    print(f"FB 출력 실패: {e}")

            time.sleep(1/30)  # 30fps 유지

    # ───── 모터 루프 ─────
    def motor_loop(self):
        while self.running:
            move.move(self.speed, self.current_direction, "no", 0)
            time.sleep(0.05)

        # ───── YOLO + 차선 주행 ─────
    def yolo_lane_loop(self):
        while self.running:
            frame = self.capture_frame()

            # YOLO 객체 감지
            detected, annotated = self.detect_objects(frame)

            # 차선 검출 → annotated 프레임 위에 덧그리기
            solid_lines, curve, roi_clean, edges = self.detect_left_yellow_lane(annotated)
            final_frame = self.draw_left_lane(annotated, solid_lines, curve)

            # 최신 주석 프레임 저장 (video나 fb 출력 루프에서 사용)
            self.annotated_frame = final_frame

            # 객체 감지 → 보행자 멈춤
            if "person" in detected:
                print("👀 Person detected → stop 1s")
                self.current_direction = "no"
                continue

            # 기본 주행: 전진
            lane_move = 'forward'
            self.current_direction = lane_move

            print(f"Detected:{detected} | Solid:{len(solid_lines)} | Move:{lane_move}")

            time.sleep(1/3)

    # ───── 실행 / 종료 ─────
    def run(self):
        print("🚗 DSM Autonomous Driving Initialized — waiting for start")
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n🛑 Ctrl+C detected! Stopping DSM Autonomous...")
            self.stop()
        except Exception as e:
            print(f"⚠️ Unexpected error: {e}")
            self.stop()
        finally:
            print("✅ Program exited cleanly.")

    def stop(self):
        self.running = False
        move.motorStop()
        if self.picam2:
            try:
                self.picam2.stop()
            except Exception as e:
                print(f"⚠️ Error stopping camera: {e}")

        if self.outputmode == "video":
            self.release_video_writer()
        GPIO.cleanup()
        print("🛑 DSM Autonomous Driving Stopped")

if __name__ == '__main__':
    auto = DSM_Autonomous(speed=40)
    auto.run()

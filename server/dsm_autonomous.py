#!/usr/bin/env python3
# File name   : dsm_autonomous_threaded.py
# Description : Threaded DSM Autonomous Driving with Line, YOLO & Ultrasonic Sensor
# Author      : seonkeun cho
# Date        : 2025/09/05

import time
import threading
import RPi.GPIO as GPIO
import move
from ultralytics import YOLO
from picamera2 import Picamera2  # 상단에 import 추가

# ───────────────────────────────────────
# DSM Autonomous Driving (Threaded)
# ───────────────────────────────────────
class DSM_Autonomous:
    def __init__(self, speed=40):
        # Picamera2 초기화 (저해상도로 속도 향상)
        picam2 = Picamera2()
        camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
        picam2.configure(camera_config)
        picam2.start()

        # 시작/센서 플래그
        self.started = False            # 출발 조건 만족 여부
        self.use_ultrasonic = True      # 출발 후 초음파 사용 중지

        # 출발 관련 설정
        self.start_threshold = 40.0     # cm, 이 값보다 멀면 '가림막 없음'으로 간주
        self.required_stable = 5        # 연속 측정 횟수

        # 센서값 저장소
        self.distance = 100.0
        self.line_left = 0
        self.line_middle = 0
        self.line_right = 0

        # 라인 센서 핀
        self.line_pin_right = 19
        self.line_pin_middle = 16
        self.line_pin_left = 20

        # 초음파 센서 핀
        self.Trig = 11
        self.Echo = 8

        # GPIO 초기화
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.line_pin_right, GPIO.IN)
        GPIO.setup(self.line_pin_middle, GPIO.IN)
        GPIO.setup(self.line_pin_left, GPIO.IN)
        GPIO.setup(self.Trig, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.Echo, GPIO.IN)
        move.setup()

        # 출발 대기 스레드만 시작 — 출발 후 라인/YOLO 스레드 시작
        threading.Thread(target=self.sensor_start_wait_loop, daemon=True).start()

    # ────────────────────────────────
    # 초음파 센서 거리 읽기 (타임아웃 추가, cm 반환)
    # ────────────────────────────────
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

            distance_cm = (t2 - t1) * 34000 / 2  # cm
            self.distance = distance_cm
            # print("Distance: %.1f cm" % distance_cm)
            return distance_cm
        except Exception as e:
            # 예외시 매우 큰 값으로 처리(가림막 없음으로 판단 방지)
            print("read_distance error:", e)
            return float('inf')

    # ────────────────────────────────
    # 라인 센서 상태 읽기
    # ────────────────────────────────
    def read_line_status(self):
        left = GPIO.input(self.line_pin_left)
        middle = GPIO.input(self.line_pin_middle)
        right = GPIO.input(self.line_pin_right)
        return left, middle, right

    # ────────────────────────────────
    # 출발 대기 루프: 초음파로 가림막 연속 판정되면 started=True로 바꾸고 라인/YOLO 시작
    # ────────────────────────────────
    def sensor_start_wait_loop(self):
        stable_count = 0
        print("🔎 Waiting for start condition (remove obstacle in front)...")
        while self.running and not self.started:
            if not self.use_ultrasonic:
                break
            d = self.read_distance()
            # 초음파가 정상적으로 읽히면 판별
            if d != float('inf') and d > self.start_threshold:
                stable_count += 1
                print(f"  start wait: distance={d:.1f}cm ({stable_count}/{self.required_stable})")
            else:
                stable_count = 0
            if stable_count >= self.required_stable:
                self.started = True
                self.use_ultrasonic = False
                print("✅ Start condition met — starting line & YOLO threads")
                # 출발 시 라인과 YOLO 루프 시작
                threading.Thread(target=self.line_loop, daemon=True).start()
                threading.Thread(target=self.yolo_loop, daemon=True).start()
                break
            time.sleep(0.1)

    # ────────────────────────────────
    # 라인 센서 기반 주행 루프 (초음파는 출발 후 사용하지 않음)
    # ────────────────────────────────
    def line_loop(self):
        while self.running:
            left, middle, right = self.read_line_status()
            self.line_left = left
            self.line_middle = middle
            self.line_right = right

            print('Line Sensor - L:%d M:%d R:%d | started:%s' %
                  (left, middle, right, str(self.started)))

            # 순수 라인 센서 기반 주행
            if left != 1:
                move.move(self.speed, 'no', 'right', 0)
            elif right != 1:
                move.move(self.speed, 'no', 'left', 0)
            else:
                move.move(self.speed, 'forward', 'no', 0)

            time.sleep(0.05)

    # ────────────────────────────────
    # YOLO 기반 장애물 감지 (proximity 플래그로 라인 루프에 알림)
    # ────────────────────────────────
    def yolo_loop(self):
        frame_count = 0
        start_time = time.time()

        while self.running:
            frame = self.picam2.capture_array()  # 미리 초기화된 카메라 사용
            frame_count += 1

            # YOLO 추론 (3프레임마다 1번씩 → 속도 ↑)
            if frame_count % 3 == 0:
                try:
                    results = self.model(frame, verbose=False)
                    names = results[0].names
                    boxes = results[0].boxes

                    detected = [names[int(cls)] for cls in boxes.cls]
                    print("Detected:", detected)

                    # 감지된 객체에 따른 동작
                    if "person" in detected:
                        print("👀 Person detected → stop for 1s")
                        move.motorStop()
                        time.sleep(1.0)
                    elif "car" in detected:
                        print("🚗 Car detected → turn left for 1s")
                        move.move(self.speed, 'no', 'left', 0)
                        time.sleep(1.0)

                except Exception as e:
                    print("YOLO error:", e)
                    time.sleep(0.1)
                    continue

            # FPS 출력
            if frame_count % 30 == 0:
                end_time = time.time()
                fps = frame_count / (end_time - start_time)
                print(f"📷 FPS: {fps:.2f}")
                frame_count = 0
                start_time = time.time()

            time.sleep(0.05)

    # ────────────────────────────────
    # 실행
    # ────────────────────────────────
    def run(self):
        print("🚗 DSM Autonomous Driving Initialized — waiting for start")
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n⛔ Interrupted by User")
            self.stop()

    # ────────────────────────────────
    # 종료
    # ────────────────────────────────
    def stop(self):
        self.running = False
        move.motorStop()
        self.picam2.stop()  # 카메라 정리 추가
        GPIO.cleanup()
        print("🛑 DSM Autonomous Driving Stopped")


# ───────────────────────────────────────
# 실행 진입점
# ───────────────────────────────────────
if __name__ == '__main__':
    auto = DSM_Autonomous(speed=40)
    auto.run()

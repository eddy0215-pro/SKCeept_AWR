#!/usr/bin/env python3
import time
import threading
import RPi.GPIO as GPIO
import move, PID
from ultralytics import YOLO
from picamera2 import Picamera2
from kalman_2d import kalman_update
import cv2
import numpy as np
import os
import datetime
import ultra
from clcd import I2CLCD

class DSM_Autonomous:
    def __init__(self, speed=90):
        self.debug = True
        self.model = YOLO("yolov8n.pt")

        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(main={"size": (320, 240)})
        self.picam2.configure(camera_config)
        self.picam2.set_controls({"AwbEnable": True})
        self.picam2.start()

        self.pid = PID.PID()
        self.pid.SetKp(0.5)
        self.pid.SetKd(0)
        self.pid.SetKi(0)

        self.running = True
        self.started = False
        self.speed = speed
        self.outputmode = "video"

        self.stop_distance = 10.0
        self.start_threshold = 10.0
        self.stable_time = 4
        self.alpha = 0.6  # annotated_frame 투명도
        self.beta = 0.4   # 원본 frame 투명도

        self.first_turn = True
        self.last_known_direction = 'forward'

        self.out = None
        self.annotated_frame = None
        self.frame_center = None
        self.center_x = None

        self.current_frame = None

        self.lcd_support = False
        self.lcd_msg = "Future of Korea"
        if self.lcd_support:
            self.lcd = I2CLCD(i2c_addr=0x27, width=16)
        
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        

        move.setup()

        if self.outputmode == "video":
            current_dir = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(current_dir)
            date_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.init_video_writer(os.path.join(parent_dir, f"drive_record_{date_str}.avi"))

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
        print("💾 Video recording stopped and saved.")

    def camera_capture_thread(self):
        while self.running:
            frame = self.picam2.capture_array()
            if self.frame_center is None:
                self.frame_center = frame.shape[1] // 2
            if frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                self.current_frame = frame

                # 두 프레임 크기 같아야 함
                if self.annotated_frame is not None:
                    combined = cv2.addWeighted(self.annotated_frame, self.alpha, frame, self.beta, 0)
                    self.write_frame(combined)
                else:
                    self.write_frame(frame)

            time.sleep(0.01)

    def clcd_task(self):
        last_msg = ""  # 루프 밖에서 이전 메시지 초기화
        while True:
            # 1번째 줄은 고정
            self.lcd.cursor_pos = (0, 0)
            self.lcd.write("DGSM".ljust(16),1)  # 16자리로 맞춤

            # 2번째 줄 메시지 갱신
            if self.lcd_msg != last_msg:
                self.lcd.cursor_pos = (1, 0)
                msg_to_display = self.lcd_msg.ljust(16)[:16]  # 16자리 맞춤
                self.lcd.write(msg_to_display,2)
                last_msg = self.lcd_msg

            time.sleep(0.1)  # 갱신 속도

    def preprocess(self, frame):
        """노란색 영역을 강조하는 전처리 함수"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 노란색 범위 설정
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 노란색 부분만 추출
        yellow_area = cv2.bitwise_and(frame, frame, mask=mask)

        # 채도(S), 명도(V) 강화
        hsv_yellow = cv2.cvtColor(yellow_area, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_yellow)

        s = cv2.add(s, 50)   # 채도 강화
        v = cv2.add(v, 30)   # 밝기 강화
        s = np.clip(s, 0, 255)
        v = np.clip(v, 0, 255)

        hsv_yellow = cv2.merge([h, s, v])
        enhanced = cv2.cvtColor(hsv_yellow, cv2.COLOR_HSV2BGR)

        return enhanced
        
    def apply_roi(self, frame):
        """입력 프레임에서 ROI 마스킹"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        height, width = mask.shape[:2]
        roi_mask = mask[int(height * 0.5):, :]   # 하단부만 ROI 적용
        return roi_mask

    def detect_lanes(self, roi_mask):
        """ROI 내에서 좌우 차선(노란색 영역) 각각 검출"""
        height, width = roi_mask.shape[:2]

        # 화면을 좌우로 나누기
        left_roi = roi_mask[:, :width // 2]
        right_roi = roi_mask[:, width // 2:]

        # 좌측 컨투어 검출
        left_contours, _ = cv2.findContours(left_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        left_contour = max(left_contours, key=cv2.contourArea) if left_contours else None

        # 우측 컨투어 검출
        right_contours, _ = cv2.findContours(right_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        right_contour = max(right_contours, key=cv2.contourArea) if right_contours else None

        # 오른쪽 컨투어 좌표 보정
        if right_contour is not None:
            for point in right_contour:
                point[0][0] += width // 2

        return left_contour, right_contour

    def get_lane_center(self, lane_contour):
        """차선 컨투어로부터 중심 좌표 계산 (X, Y)"""
        if lane_contour is None:
            return -1, -1
        moments = cv2.moments(lane_contour)
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            return cx, cy
        return -1, -1

    def pid_control(self, fx, frame_center, pid, pwm_limit=100):
        """
        fx : 칼만 필터 적용 후 차선 중심 X좌표
        frame_center : 화면 중앙 X좌표
        pid : PID 클래스 인스턴스
        base_speed : 직진 기본 속도
        pwm_limit : 모터 최대값
        """
        # 1. 오차 계산
        error_x = (fx - frame_center) / frame_center

        # 2. PID 제어값 계산 및 제한
        steering = max(min(pid.GenOut(error_x), pwm_limit), -pwm_limit)

        # 3. 좌우 모터 속도 계산
        left_speed  = max(min(self.speed + steering, pwm_limit), 0)
        right_speed = max(min(self.speed - steering, pwm_limit), 0)

        return left_speed, right_speed

    def read_distance(self):
        try:
            return ultra.checkdist()
        except Exception as e:
            print(f"⚠️ 초음파 센서 오류: {e}")
            return float('inf')

    def sensor_start_wait_loop(self):
        """
        초음파 센서로 장애물 제거 확인 후 주행 시작
        """
        print("🔎 Waiting for obstacle removal to start...")
        stable_start_time = None
        last_display = None  # 이전에 표시한 값
        print(f"Running: {self.running}, Started: {self.started}")

        while self.running and not self.started:
            distance = self.read_distance() * 100

            if self.debug:
                print(f"Distance: {distance:.2f} cm")

            if distance == float('inf'):
                stable_start_time = None
                print("⚠️ Sensor read failed, waiting...")

            elif distance < self.start_threshold:
                stable_start_time = None
                self.lcd_msg = "Obstacle Ahead..."
                print(f"🚧 Obstacle detected ({distance:.2f} cm), waiting...")

            else:
                # 장애물 제거됨
                if stable_start_time is None:
                    stable_start_time = time.time()
                    print(f"✅ Obstacle removed detected ({distance:.2f} cm), confirming...")
                else:
                    elapsed = time.time() - stable_start_time
                    remaining = max(0, int(self.stable_time - elapsed))

                    if remaining != last_display:
                        self.lcd_msg = f"Stable: {remaining}s"
                        last_display = remaining

                    if elapsed >= self.stable_time:
                        self.started = True
                        self.lcd_msg = "Start confirmed!"
                        print("▶️ Start confirmed! Starting driving loop.")
                        threading.Thread(target=self.yolo_lane_loop, daemon=True).start()
                        break

            time.sleep(0.1)  # 0.3 → 0.1초로 갱신 속도 빠르게

    def display_debug(self, cx, cy, fx, fy, left_speed, right_speed, left_dir, right_dir):
        if self.current_frame is None:
            return None  # None 반환

        # 원본 프레임 복사
        frame = self.current_frame.copy()
        h, w = frame.shape[:2]

        # 차선 측정 좌표 표시 (노란색)
        if cx >= 0 and cy >= 0:
            cv2.circle(frame, (cx, cy), 5, (255, 0, 255), -1)
            cv2.putText(frame, f"Measured: ({cx},{cy})", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 255), 1)

        # 칼만 필터 좌표 표시 (빨간색)
        cv2.circle(frame, (fx, fy), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"Kalman: ({fx},{fy})", (5, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

        # PID / 모터 정보 표시 (파란색)
        cv2.putText(frame, f"Left spd: {left_speed}, Right spd: {right_speed}", (5, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
        cv2.putText(frame, f"Left dir: {left_dir}, Right dir: {right_dir}", (5, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)

        # annotated_frame 갱신
        self.annotated_frame = frame

    def draw_center_message(self, frame, message, color=(0, 0, 255), font_scale=0.8, thickness=2):
        h, w = frame.shape[:2]
        text_size, _ = cv2.getTextSize(message, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        text_x = (w - text_size[0]) // 2
        text_y = (h // 2)  # 수직 중앙

        cv2.putText(frame, message, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

        # annotated_frame 갱신
        self.annotated_frame = frame

    def yolo_lane_loop(self):
        print("🚦 Starting main driving loop...")
        while self.running:
            frame = None
            if self.current_frame is not None:
                frame  = self.current_frame.copy()

            if frame is None:
                time.sleep(0.01)
                continue

            # 1. 전처리
            edges = self.preprocess(frame)

            # 2. ROI 영역 마스킹
            roi = self.apply_roi(edges)

             # 3. 차선 검출 (허프 or 폴리피팅)
            left_lanes, right_lanes = self.detect_lanes(roi)

            # 4. 차선 중심 좌표 구하기
            left_cx, left_cy = self.get_lane_center(left_lanes)
            right_cx, right_cy = self.get_lane_center(right_lanes)

            left_cx, left_cy = int(left_cx), int(left_cy)  # 정수로 변환
            right_cx, right_cy = int(right_cx), int(right_cy)  # 정수로 변환

            # 5. 칼만 필터 적용 (노이즈 제거 + 이동 예측)
            l_fx, l_fy = kalman_update(left_cx, left_cy)
            r_fx, r_fy = kalman_update(right_cx, right_cy)
            l_fx, l_fy = int(l_fx), int(l_fy)  # 정수로 변환
            r_fx, r_fy = int(r_fx), int(r_fy)  # 정수로 변환

            # 두 중심의 평균으로 도로 중앙 계산
            if left_cx != -1 and right_cx != -1:
                center_x = (l_fx + l_fx) // 2
                center_y = (l_fy + r_fy) // 2
            else:
                self.lcd_msg = "No yellow Lane"
                print("노란선 없음: 차선 인식 실패")
                self.draw_center_message(frame, "Yellow lane not detected.", color=(0, 0, 255))
                move.motor_left(1, 1, 0)
                move.motor_right(1, 1, 0)
                time.sleep(1/30)
                continue

            # 6. 차량 제어 신호 생성
            left_speed, right_speed = self.pid_control(center_x, self.frame_center, self.pid)
            left_speed, right_speed = int(left_speed), int(right_speed)
            
            # 모터 방향 결정
            left_dir  = 1
            right_dir = 1

            # 속도가 음수면 방향 반대로
            if left_speed < 0:
                left_speed = abs(left_speed)
                left_dir = 0

            if right_speed < 0:
                right_speed = abs(right_speed)
                right_dir = 0

            # 3. 모터에 전달
            move.motor_left(1, left_dir, left_speed)
            move.motor_right(1, right_dir, right_speed)

            self.display_debug(left_cx, right_cx, l_fx, r_fx, left_speed, right_speed, left_dir, right_dir)

            if self.debug:
                # 디버그: 측정 좌표 출력
                print(f"Measured left lane center: cx={left_cx}, cy={left_cy}")
                print(f"Measured right lane center: cx={right_cx}, cy={right_cy}")            
                # 디버그: 칼만 필터 적용 후 값
                print(f"Kalman filtered: left fx={l_fx}, fy={l_fy}")
                print(f"Kalman filtered: right fx={r_fx}, fy={r_fy}")
                # 디버그: PID 기반 좌우 속도
                print(f"PID control -> Left speed: {left_speed}, Right speed: {right_speed}")
                # 디버그: 최종 모터 제어 신호
                print(f"Motor command -> Left: ({left_dir}, {left_speed}), Right: ({right_dir}, {right_speed})")

            time.sleep(1/30)

    def run(self):
        print("🚗 DSM Autonomous Driving Initialized — waiting for start")
        threading.Thread(target=self.sensor_start_wait_loop, daemon=True).start()
        threading.Thread(target=self.camera_capture_thread, daemon=True).start()

        if self.lcd_support:
            threading.Thread(target=self.clcd_task, daemon=True).start()

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
                print(f"⚠️ Camera stop error: {e}")
        if self.outputmode == "video":
            self.release_video_writer()

        print("🛑 DSM Autonomous Driving Stopped")

if __name__ == '__main__':
    auto = DSM_Autonomous(speed=100)
    auto.run()
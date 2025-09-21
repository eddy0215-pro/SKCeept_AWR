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
import datetime
import ultra

class DSM_Autonomous:
    def __init__(self, speed=90):
        self.model = YOLO("yolov8n.pt")

        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(main={"size": (320, 240)})
        self.picam2.configure(camera_config)
        self.picam2.set_controls({"AwbEnable": True})
        self.picam2.start()

        self.running = True
        self.started = False
        self.speed = speed
        self.outputmode = "video"

        self.stop_distance = 10.0
        self.start_threshold = 10.0
        self.stable_time = 3.0

        self.first_turn = True
        self.last_known_direction = 'forward'

        self.out = None
        self.mask_out = None
        self.annotated_frame = None

        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        self.avg_cx_value = 0
        self.offset_value = 0
        self.log_lock = threading.Lock()

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        move.setup()

        # 💡 하드 턴 로직을 위한 변수
        self.driving_state = 'normal' # 'normal', 'performing_hard_turn'
        self.right_turn_counter = 0
        self.turn_start_time = 0
        self.turn_duration = 1.0  # 90도 회전 지속 시간 (초), 조정 가능
        self.hard_turn_threshold = -60 # 하드 턴을 시작하는 오프셋 임계값
        self.hard_turn_frame_limit = 10 # 하드 턴을 시작하기 위한 연속 프레임 수

        if self.outputmode == "video":
            current_dir = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(current_dir)
            date_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.init_video_writer(os.path.join(parent_dir, f"drive_record_{date_str}.avi"))
            self.init_mask_writer(os.path.join(parent_dir, f"mask_record_{date_str}.avi"))

    def init_video_writer(self, filename, fps=30.0, size=(320, 240)):
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.out = cv2.VideoWriter(filename, fourcc, fps, size)
        print(f"🎥 Recording started: {filename}")

    def init_mask_writer(self, filename, fps=30.0, size=(320, 240)):
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.mask_out = cv2.VideoWriter(filename, fourcc, fps, size, isColor=False)
        print(f"🎥 Mask recording started: {filename}")

    def write_frame(self, frame):
        if self.out is not None:
            self.out.write(frame)

    def write_mask_frame(self, mask):
        if self.mask_out is not None:
            self.mask_out.write(mask)

    def release_video_writer(self):
        if self.out is not None:
            self.out.release()
        if self.mask_out is not None:
            self.mask_out.release()
        self.out = None
        self.mask_out = None
        print("💾 Video recording stopped and saved.")

    def camera_capture_thread(self):
        while self.running:
            frame = self.picam2.capture_array()
            if frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                with self.frame_lock:
                    self.current_frame = frame
            time.sleep(0.01)

    def log_line_position_thread(self):
        while self.running:
            with self.log_lock:
                print(f"Line Center (avg_cx): {self.avg_cx_value}, Offset: {self.offset_value}")
            time.sleep(1)

    def detect_yellow_lane_single_roi(self, frame):
        """가장 큰 하나의 노란색 선의 중심을 탐지"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        height, width = mask.shape[:2]

        roi_mask = mask[int(height * 0.5):, :]

        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return -1, roi_mask, None

        largest_contour = max(contours, key=cv2.contourArea)
        
        moments = cv2.moments(largest_contour)
        
        cx = -1
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
        
        return cx, roi_mask, largest_contour

    def read_distance(self):
        try:
            return ultra.checkdist()
        except Exception as e:
            print(f"⚠️ 초음파 센서 오류: {e}")
            return float('inf')

    def sensor_start_wait_loop(self):
        print("🔎 Waiting for obstacle removal to start...")
        stable_start = None
        obstacle_present = True

        while self.running and not self.started:
            distance = self.read_distance() * 100
            if distance == float('inf'):
                stable_start = None
                obstacle_present = True
                print("⚠️ Sensor read failed, waiting...")
            else:
                print(f"Distance: {distance:.2f} cm")
                if distance < self.start_threshold:
                    obstacle_present = True
                    stable_start = None
                    print("🚧 Obstacle detected, waiting...")
                else:
                    if obstacle_present:
                        stable_start = time.time()
                        obstacle_present = False
                        print("✅ Obstacle removed detected, confirming...")
                    else:
                        if stable_start and (time.time() - stable_start) >= self.stable_time:
                            self.started = True
                            print("▶️ Start confirmed! Starting driving loop.")
                            threading.Thread(target=self.yolo_lane_loop, daemon=True).start()
                            break
            time.sleep(0.3)

    def yolo_lane_loop(self):
        print("🚦 Starting main driving loop...")

        if self.first_turn:
            print("➡️ 시작 - 즉시 직진")
            # move.set_individualmo_speeds(self.speed, self.speed * 0.75) 
            move.move(self.speed,"forward","no")
            time.sleep(2.0)
            self.first_turn = False

        while self.running:
            distance = self.read_distance() * 100
            if distance < self.stop_distance:
                print(f"🛑 장애물 감지: {distance:.2f} cm, 정지합니다.")
                self.stop()
                break

            frame = None
            with self.frame_lock:
                if self.current_frame is not None:
                    frame = self.current_frame.copy()

            if frame is None:
                time.sleep(0.01)
                continue

            cx, mask, largest_contour = self.detect_yellow_lane_single_roi(frame)
            
            # 💡 하드 턴 상태 확인
            # if self.driving_state == 'performing_hard_turn':
            #     self.handle_hard_turn()
            
            # else: # 💡 일반 주행 모드
            if cx != -1:
                frame_center = frame.shape[1] // 2
                offset = cx - frame_center
                
                with self.log_lock:
                    self.avg_cx_value = cx
                    self.offset_value = offset

                # 💡 하드 우회전 조건 확인
                if offset < self.hard_turn_threshold:
                    self.right_turn_counter += 1
                    print(f"⚠️ 하드 우회전 감지. 카운터: {self.right_turn_counter}")
                    if self.right_turn_counter >= self.hard_turn_frame_limit:
                        print(f"🔄 연속 하드 우회전({self.right_turn_counter} 프레임) 감지, 하드 턴 시작!")
                        move.motorStop()
                        self.driving_state = 'performing_hard_turn'
                        self.turn_start_time = time.time()
                        self.right_turn_counter = 0
                        continue # 다음 프레임으로 바로 넘어가서 하드 턴 실행

                else:
                    self.right_turn_counter = 0

                threshold = 30 
                if offset > threshold:
                    print(f"⬅️ 좌회전 (offset {offset})")
                    # move.set_individual_speeds(-self.speed, self.speed)
                    move.move(self.speed,"forward","left")
                    self.last_known_direction = 'left'
                elif offset < -threshold:
                    print(f"➡️ 우회전 (offset {offset})")
                    # move.set_individual_speeds(self.speed, -self.speed)
                    move.move(self.speed,"forward","right")
                    self.last_known_direction = 'right'
                else:
                    print(f"✅ 직진 (offset {offset})")
                    # move.set_individual_speeds(self.speed, self.speed)
                    move.move(self.speed,"forward","no")
                    self.last_known_direction = 'forward'
            else:
                print("⚠️ 차선 미인식. 직진을 시도합니다.")
                # move.set_individual_speeds(self.speed, self.speed)
                move.move(self.speed,"forward","no")
                self.last_known_direction = 'forward'
                
            self.annotated_frame = frame
            
            if cx != -1:
                cv2.circle(frame, (cx, frame.shape[0] - 20), 10, (0, 255, 0), -1)
                
            frame_center = frame.shape[1] // 2
            cv2.line(frame, (frame_center, 0), (frame_center, frame.shape[0]), (255, 0, 0), 2)
            
            if self.outputmode == "video":
                self.write_frame(frame)
                self.write_mask_frame(mask)

            time.sleep(0.01)

    def handle_hard_turn(self):
        """하드 턴 로직 실행"""
        current_time = time.time()
        if current_time - self.turn_start_time < self.turn_duration:
            print("🔄 90도 오른쪽 회전 중...")
            move.move(self.speed,"forward","right")
        else:
            print("✅ 90도 회전 완료, 일반 주행 복귀")
            move.motorStop()
            self.driving_state = 'normal'
            self.last_known_direction = 'forward' # 상태 초기화

    def run(self):
        print("🚗 DSM Autonomous Driving Initialized — waiting for start")
        threading.Thread(target=self.sensor_start_wait_loop, daemon=True).start()
        threading.Thread(target=self.camera_capture_thread, daemon=True).start()
        threading.Thread(target=self.log_line_position_thread, daemon=True).start()

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

        GPIO.cleanup()
        print("🛑 DSM Autonomous Driving Stopped")

if __name__ == '__main__':
    auto = DSM_Autonomous(speed=60)
    auto.run()
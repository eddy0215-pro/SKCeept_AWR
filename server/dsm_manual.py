#!/usr/bin/env python3
import time
import threading
import RPi.GPIO as GPIO
import move
from picamera2 import Picamera2
import cv2
import os
import datetime
import numpy as np
from ultralytics import YOLO
import sys, termios, tty, select

class DSM_Autonomous:
    def __init__(self, speed=90):
        self.model = YOLO("yolov8n.pt")
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(main={"size": (320, 240)})
        self.picam2.configure(camera_config)
        self.picam2.set_controls({"AwbEnable": True})
        self.picam2.start()

        self.running = True
        self.started = True  # 센서 관련 코드 제거, 바로 시작
        self.speed = speed
        self.current_direction = "no"
        self.current_turn = "no"
        self.outputmode = "video"

        self.out = None
        self.annotated_frame = None
        self.current_frame = None

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        move.setup()

        if self.outputmode == "video":
            current_dir = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(current_dir)
            date_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.init_video_writer(os.path.join(parent_dir, f"drive_record_{date_str}.avi"))

    # ==================== 영상 기록 ====================
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

    # ==================== 카메라 스레드 ====================
    def capture_frame(self):
        frame = self.picam2.capture_array()
        if frame is None:
            print("Warning: frame capture failed")
            return None
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def detect_objects(self, frame):
        results = self.model(frame, verbose=False)
        names = results[0].names
        boxes = results[0].boxes
        detected = [names[int(cls)] for cls in boxes.cls]
        if detected:
            print(f"\r[{datetime.datetime.now().strftime('%H:%M:%S')}] 🟥 Detected objects: {detected}", flush=True)
        return detected, results[0].plot()

    def detect_left_yellow_lane(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 💡 정밀하게 조정된 HSV 색상 범위
        lower_yellow = np.array([26, 140, 200])
        upper_yellow = np.array([30, 210, 235])
        
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        height, width = mask.shape[:2]
        # roi_mask = mask[int(height * 0.6):height, :]
        roi_mask = mask[int(height * 0.8):height, :]
        
        M = cv2.moments(roi_mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + int(height * 0.6)
            return cx, cy, mask
        else:
            return None, None, mask

    def draw_lane_center(self, frame, cx, cy):
        if cx is not None:
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        return frame
    
    def camera_record_loop(self):
        while self.running:
            frame = self.capture_frame()
            if self.annotated_frame is not None:
                frame_to_record = cv2.addWeighted(frame, 0.7, self.annotated_frame, 0.3, 0)
            else:
                frame_to_record = frame
            
            if self.outputmode == "video":
                self.write_frame(frame_to_record)
            
            elif self.outputmode == "fb":
                small_w, small_h = 320, 240
                frame_small = cv2.resize(frame_to_record, (small_w, small_h))
                frame_rgb565 = cv2.cvtColor(frame_small, cv2.COLOR_BGR2BGR565)
                fb_path = "/dev/fb0"
                fb_width, fb_height = 1920, 1080
                bpp = 2
                line_length = fb_width * bpp
                x_offset = (fb_width - small_w) // 2
                y_offset = (fb_height - small_h) // 2
                if os.path.exists(fb_path):
                    try:
                        with open(fb_path, "r+b") as f:
                            for row in range(small_h):
                                offset = ((y_offset + row) * line_length) + (x_offset * bpp)
                                f.seek(offset)
                                f.write(frame_rgb565[row].tobytes())
                    except Exception as e:
                        print(f"FB 출력 실패: {e}")
            
            time.sleep(1/30)

    def motor_loop(self):
        while self.running:
            move.move(self.speed, self.current_direction, self.current_turn, 1)
            time.sleep(0.05)

    # ==================== 주행 루프 ====================
    def yolo_lane_loop(self):
        while self.running:
            frame = self.capture_frame()
            if frame is None:
                print("⚠️ Frame capture failed, skipping loop iteration.")
                time.sleep(1/30)
                continue
            
            detected, annotated = self.detect_objects(frame)
            
            cx, cy, mask = self.detect_left_yellow_lane(annotated)
            final_frame = self.draw_lane_center(annotated, cx, cy)
            self.annotated_frame = final_frame
            
            if cx is None:
                lane_move = 'forward'
                print(f"\r[{datetime.datetime.now().strftime('%H:%M:%S')}] 🚫 No lane detected, keep forward", flush=True)
            else:
                height, width = frame.shape[:2]
                center_offset = cx - (width / 2)

                if center_offset < -5:
                    lane_move = 'right'
                elif center_offset > 5:
                    lane_move = 'left'
                else:
                    lane_move = 'forward'
                
                print(f"Lane center x: {cx}, Offset: {center_offset}, Move: {lane_move}")
            
            self.current_direction = lane_move
            time.sleep(1/3)

    # ==================== 키보드 입력 제어 루프 ====================
    def getch_nonblock(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)  # 10ms 대기
            if rlist:
                return sys.stdin.read(1)
            else:
                return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def manual_drive_loop(self):
        print("🚦 Starting keyboard-controlled driving loop...")
        print("Controls: w=forward, s=backward, a=left, d=right, q=quit")
        
        last_input_time = time.time()

        while self.running:
            key = self.getch_nonblock()

            if key == 'w':
                print("w:forward")
                self.current_direction, self.current_turn = "forward", "no"
                last_input_time = time.time()
            elif key == 's':
                print("s:backward")
                self.current_direction, self.current_turn = "backward", "no"
                last_input_time = time.time()
            elif key == 'a':
                print("a:left")
                self.current_direction, self.current_turn = "forward", "left"
                last_input_time = time.time()
            elif key == 'd':
                print("d:right")
                self.current_direction, self.current_turn = "forward", "right"
                last_input_time = time.time()
            elif key == 'q':
                print("q:Quit")
                print("🛑 Quit command received")
                self.stop()
                break

            # 입력이 일정 시간 이상 없으면 멈춤
            if time.time() - last_input_time > 0.2:
                self.current_direction, self.current_turn = "no", "no"

            time.sleep(0.005)  # 짧은 주기로 빠르게 체크

    def run(self):
        print("🚗 DSM Autonomous Driving Initialized — starting driving loop")
        threading.Thread(target=self.camera_record_loop, daemon=True).start()
        threading.Thread(target=self.yolo_lane_loop, daemon=True).start()
        threading.Thread(target=self.manual_drive_loop, daemon=True).start()
        threading.Thread(target=self.motor_loop, daemon=True).start()

        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n🛑 Ctrl+C detected! Stopping DSM Autonomous...")
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
    auto = DSM_Autonomous(speed=60)
    auto.run()
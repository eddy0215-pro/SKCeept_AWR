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
    def __init__(self, speed=40):
        self.model = YOLO("yolov8n.pt")
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(main={"size": (320, 240)})
        self.picam2.configure(camera_config)
        self.picam2.set_controls({"AwbEnable": True})
        self.picam2.start()

        self.running = True
        self.started = False
        self.use_ultrasonic = True
        self.speed = speed
        self.outputmode = "video"
        
        self.stop_distance = 10.0
        self.Trig = 11
        self.Echo = 8

        self.start_threshold = 10.0
        self.stable_time = 3.0
        self.current_direction = "no"  

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Trig, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.Echo, GPIO.IN)
        move.setup()

        threading.Thread(target=self.sensor_start_wait_loop, daemon=True).start()

        if self.outputmode == "video":
            current_dir = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(current_dir)
            date_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.out = None
            self.init_video_writer(os.path.join(parent_dir, f"drive_record_{date_str}.avi"))

        self.annotated_frame = None

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
            print(f"🟥 Detected objects: {detected}")
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

    def read_distance(self):
        try:
            return ultra.checkdist()
        except:
            return float('inf')

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
            move.move(self.speed, self.current_direction, "no", 1)
            time.sleep(0.05)

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
                print("🚫 No lane detected, keep forward")
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
    auto = DSM_Autonomous(speed=35)
    auto.run()
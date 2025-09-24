#!/usr/bin/env python3
import time
import threading
import RPi.GPIO as GPIO
import move
from picamera2 import Picamera2
import cv2
import os
import datetime
import sys, termios, tty, select

class DSM_Autonomous:
    def __init__(self, speed=90):
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(main={"size": (320, 240)})
        self.picam2.configure(camera_config)
        self.picam2.set_controls({"AwbEnable": True})
        self.picam2.start()

        self.running = True
        self.started = True  # 센서 관련 코드 제거, 바로 시작
        self.speed = speed
        self.outputmode = "video"

        self.out = None
        self.annotated_frame = None
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.log_lock = threading.Lock()

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
    def camera_capture_thread(self):
        while self.running:
            frame = self.picam2.capture_array()
            if frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                with self.frame_lock:
                    self.current_frame = frame
            time.sleep(0.01)

    # ==================== 주행 루프 ====================
    def yolo_lane_loop(self):
        print("🚦 Starting main driving loop...")

        while self.running:
            frame = None
            with self.frame_lock:
                if self.current_frame is not None:
                    frame = self.current_frame.copy()

            if frame is not None and self.outputmode == "video":
                self.write_frame(frame)
                
            time.sleep(0.05)

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

    def drive_loop(self):
        print("🚦 Starting keyboard-controlled driving loop...")
        print("Controls: w=forward, s=backward, a=left, d=right, q=quit")
        direction = None
        turn = None
        last_input_time = time.time()

        while self.running:
            key = self.getch_nonblock()

            if key == 'w':
                print("w!!")
                direction, turn = "forward", "no"
                last_input_time = time.time()
            elif key == 's':
                print("s!!")
                direction, turn = "backward", "no"
                last_input_time = time.time()
            elif key == 'a':
                print("a!!")
                direction, turn = "forward", "left"
                last_input_time = time.time()
            elif key == 'd':
                print("d!!")
                direction, turn = "forward", "right"
                last_input_time = time.time()
            elif key == 'q':
                print("q!!")
                print("🛑 Quit command received")
                self.stop()
                break

            # 입력이 일정 시간 이상 없으면 멈춤
            if time.time() - last_input_time > 0.2:
                direction, turn = "no", "no"

            move.move(self.speed, direction, turn)
            time.sleep(0.005)  # 짧은 주기로 빠르게 체크

    def run(self):
        print("🚗 DSM Autonomous Driving Initialized — starting driving loop")
        threading.Thread(target=self.camera_capture_thread, daemon=True).start()
        threading.Thread(target=self.yolo_lane_loop, daemon=True).start()
        threading.Thread(target=self.drive_loop, daemon=True).start()

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
        move.destroy()
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
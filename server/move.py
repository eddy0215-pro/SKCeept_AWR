#!/usr/bin/env python3
# File name   : move.py
# Description : Control Motor
# Product     : RaspTank  
# Website     : www.adeept.com
# Author      : William
# Date        : 2019/02/23

import time
import RPi.GPIO as GPIO

# ==================== 핀 설정 ====================
Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

# ==================== 방향 상수 ====================
Dir_forward   = 1
Dir_backward  = 0

left_forward  = 1
left_backward = 0

right_forward = 0
right_backward= 1

# ==================== PWM 객체 ====================
pwm_A = None
pwm_B = None

# ==================== 모터 제어 ====================
def motorStop():
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    GPIO.output(Motor_B_EN, GPIO.LOW)

def setup():
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)

    motorStop()

    try:
        pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        pwm_B = GPIO.PWM(Motor_B_EN, 1000)
    except Exception:
        pass

def motor_left(status, direction, speed):
    if status == 0:
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)
        else:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)

def motor_right(status, direction, speed):
    if status == 0:
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction != Dir_forward:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        else:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)
    return direction

# ==================== 이동 제어 ====================
def move(speed, direction, turn, radius=0.6):
    """
    speed: 0~100
    direction: 'forward', 'backward', 'no'
    turn: 'left', 'right', 'none'
    """
    speed = abs(speed)

    if direction == 'forward':
        left_dir = left_forward
        right_dir = right_forward
    elif direction == 'backward':
        left_dir = left_backward
        right_dir = right_backward
    else:  # no movement
        motorStop()
        return

    if turn == 'left':
        # 제자리 좌회전: 왼쪽 전진, 오른쪽 후진
        motor_left(1, left_forward, speed)
        motor_right(1, right_backward, speed)
    elif turn == 'right':
        # 제자리 우회전: 왼쪽 후진, 오른쪽 전진
        motor_left(1, left_backward, speed)
        motor_right(1, right_forward, speed)
    else:
        # 직진/후진
        motor_left(1, left_dir, speed)
        motor_right(1, right_dir, speed)

# ==================== 정리 ====================
def destroy():
    motorStop()
    if pwm_A is not None:
        pwm_A.stop()
    if pwm_B is not None:
        pwm_B.stop()
    GPIO.cleanup()

# ==================== 테스트 ====================
if __name__ == '__main__':
    try:
        setup()
        while True:
            speed_set = 100
            # move(speed_set, "forward", "none")
            # time.sleep(1)
            # move(speed_set, "backward", "none")
            # time.sleep(1)
            motor_left(1, 1, speed_set)
            motor_right(1, 1, speed_set)
            time.sleep(1)
            motor_left(1, 0, speed_set)
            motor_right(1, 0, speed_set)
            time.sleep(1)
    except KeyboardInterrupt:
        destroy()

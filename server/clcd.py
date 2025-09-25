#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# I2C CLCD Library for Raspberry Pi

import smbus2
import time
import threading

class I2CLCD:
    def __init__(self, i2c_addr=0x27, width=16):
        self.I2C_ADDR = i2c_addr
        self.LCD_WIDTH = width

        self.LCD_CHR = 1  # 문자 모드
        self.LCD_CMD = 0  # 명령 모드

        # 줄 주소
        self.LCD_LINE_1 = 0x80
        self.LCD_LINE_2 = 0xC0
        self.LCD_LINE_3 = 0x94  # 20x4 LCD용
        self.LCD_LINE_4 = 0xD4  # 20x4 LCD용

        self.ENABLE = 0b00000100
        self.BACKLIGHT = 0x08

        self.E_PULSE = 0.0005
        self.E_DELAY = 0.0005

        self.bus = smbus2.SMBus(1)  # I2C-1 사용

        self.lcd_init()

    def lcd_toggle_enable(self, bits):
        time.sleep(self.E_DELAY)
        self.bus.write_byte(self.I2C_ADDR, (bits | self.ENABLE))
        time.sleep(self.E_PULSE)
        self.bus.write_byte(self.I2C_ADDR, (bits & ~self.ENABLE))
        time.sleep(self.E_DELAY)

    def lcd_byte(self, bits, mode):
        """LCD로 바이트 전송"""
        bits_high = mode | (bits & 0xF0) | self.BACKLIGHT
        bits_low = mode | ((bits << 4) & 0xF0) | self.BACKLIGHT

        self.bus.write_byte(self.I2C_ADDR, bits_high)
        self.lcd_toggle_enable(bits_high)

        self.bus.write_byte(self.I2C_ADDR, bits_low)
        self.lcd_toggle_enable(bits_low)

    def lcd_init(self):
        """LCD 초기화"""
        self.lcd_byte(0x33, self.LCD_CMD)  # 초기화
        self.lcd_byte(0x32, self.LCD_CMD)  # 4비트 모드
        self.lcd_byte(0x06, self.LCD_CMD)  # 커서 오른쪽 이동
        self.lcd_byte(0x0C, self.LCD_CMD)  # 디스플레이 켜기, 커서 숨김
        self.lcd_byte(0x28, self.LCD_CMD)  # 2줄, 5x8 폰트
        self.clear()

    def clear(self):
        """화면 지우기"""
        self.lcd_byte(0x01, self.LCD_CMD)
        time.sleep(self.E_DELAY)

    def write(self, message, line=1):
        """특정 라인에 문자열 출력"""
        if line == 1:
            addr = self.LCD_LINE_1
        elif line == 2:
            addr = self.LCD_LINE_2
        elif line == 3:
            addr = self.LCD_LINE_3
        elif line == 4:
            addr = self.LCD_LINE_4
        else:
            raise ValueError("line 은 1~4만 가능합니다.")

        message = message.ljust(self.LCD_WIDTH, " ")
        self.lcd_byte(addr, self.LCD_CMD)
        for char in message:
            self.lcd_byte(ord(char), self.LCD_CHR)

    def backlight_on(self):
        self.BACKLIGHT = 0x08

    def backlight_off(self):
        self.BACKLIGHT = 0x00

    def marquee_text(self, message, line=1, delay=0.2):
        """
        오른쪽에서 왼쪽으로 무한 스크롤.
        문구 끝나도 끊김 없이 이어서 반복.
        """
        msg = message + " " * self.LCD_WIDTH  # 뒤에 공백만 추가
        msg_len = len(msg)
        pos = 0  # 시작 위치

        while True:
            # 현재 화면에 표시할 부분 슬라이싱
            display_text = (msg + msg)[pos:pos + self.LCD_WIDTH]  # 원형 연결
            self.write(display_text, line)
            pos = (pos + 1) % msg_len  # 위치 증가 후 루프
            time.sleep(delay)


# 테스트 실행 (직접 실행했을 때만 동작)
if __name__ == "__main__":
    lcd = I2CLCD(i2c_addr=0x27, width=16)  # 필요 시 주소 변경
    threading.Thread(
        target=lcd.marquee_text,
        args=("Dream Global Smart Minds", 1, 0.2),
        daemon=True
    ).start()
    while True:
        lcd.write("Future of Chips", 2)
        time.sleep(3)
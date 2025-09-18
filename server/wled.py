#!/usr/bin/python3
# File name   : led_strip.py
# Description : WS2812 LED library - all white
# Author      : Based on Tony DiCola

from rpi_ws281x import *
import time

class wled:
    def __init__(self, count=16, pin=12, brightness=255, channel=0):
        self.LED_COUNT = count
        self.LED_PIN = pin
        self.LED_FREQ_HZ = 800000
        self.LED_DMA = 10
        self.LED_BRIGHTNESS = brightness
        self.LED_INVERT = False
        self.LED_CHANNEL = channel

        # NeoPixel 객체 생성
        self.strip = Adafruit_NeoPixel(
            self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, 
            self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, 
            self.LED_CHANNEL
        )
        self.strip.begin()

    def all_white(self):
        """모든 LED를 흰색으로 켬"""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, Color(255, 255, 255))
        self.strip.show()

    def all_off(self):
        """모든 LED 끔"""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, Color(0, 0, 0))
        self.strip.show()

    def set_color(self, r, g, b):
        """모든 LED를 원하는 색으로 설정"""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, Color(r, g, b))
        self.strip.show()

    def color_wipe(self, r, g, b, wait_ms=50):
        """한 픽셀씩 색을 채우는 효과"""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, Color(r, g, b))
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

# 간단 테스트
if __name__ == '__main__':
    led = wled()
    try:
        led.all_white()
        input("Press Enter to turn off...")
        led.all_off()
    except KeyboardInterrupt:
        led.all_off()

import RPi.GPIO as GPIO
import time

class GPIOLedButton:
    def __init__(self, led_pin=23, button_pin=24, debounce_time=0.2):
        self.led_pin = led_pin
        self.button_pin = button_pin
        self.debounce_time = debounce_time
        self.last_state = GPIO.HIGH
        self.last_change_time = time.time()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # LED 테스트
        GPIO.output(self.led_pin, GPIO.LOW)
        time.sleep(1)
        GPIO.output(self.led_pin, GPIO.HIGH)
        time.sleep(1)

    def run(self):
        print("버튼을 눌렀다가 뗄 때 LED가 켜집니다. Ctrl+C로 종료하세요.")
        try:
            while True:
                current_state = GPIO.input(self.button_pin)
                current_time = time.time()

                # 버튼을 뗀 순간 감지 (LOW → HIGH)
                if self.last_state == GPIO.LOW and current_state == GPIO.HIGH:
                    if current_time - self.last_change_time > self.debounce_time:
                        print("버튼을 뗐습니다!")
                        GPIO.output(self.led_pin, GPIO.HIGH)
                        self.last_change_time = current_time

                # 버튼을 누르고 있는 동안 LED OFF
                if current_state == GPIO.LOW:
                    GPIO.output(self.led_pin, GPIO.LOW)

                self.last_state = current_state
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n종료 중...")
        finally:
            self.cleanup()

    def cleanup(self):
        GPIO.output(self.led_pin, GPIO.LOW)
        GPIO.cleanup()

if __name__ == '__main__':
    controller = GPIOLedButton()
    controller.run()
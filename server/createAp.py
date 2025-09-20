import os
import re
import netifaces
from wled import wled  # 모듈에서 클래스 가져오기

def ap_thread():
    # wlan0의 MAC 주소 가져오기
    try:
        mac = netifaces.ifaddresses('wlan0')[netifaces.AF_LINK][0]['addr']
        mac_suffix = re.sub(":", "", mac)[-4:]  # 콜론 제거 후 마지막 4자리
    except Exception:
        mac_suffix = "0000"  # 오류 시 기본값

    ssid_name = f"Groovy{mac_suffix}"
    os.system(f"sudo create_ap --no-virt -n wlan0 {ssid_name} dsm12345")

# 간단 테스트
if __name__ == '__main__':
    led = wled()
    try:
        led.all_white()
        ap_thread()
    except KeyboardInterrupt:
        led.all_off()

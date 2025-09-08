import os
import re
import netifaces
import time

def ap_thread():
    # wlan0의 MAC 주소 가져오기
    try:
        mac = netifaces.ifaddresses('wlan0')[netifaces.AF_LINK][0]['addr']
        mac_suffix = re.sub(":", "", mac)[-4:]  # 콜론 제거 후 마지막 4자리
    except Exception:
        mac_suffix = "0000"  # 오류 시 기본값

    ssid_name = f"Groovy{mac_suffix}"
    os.system(f"sudo create_ap --no-virt wlan0 eth0 {ssid_name} dsm12345")

if __name__ == "__main__":
    ap_thread()

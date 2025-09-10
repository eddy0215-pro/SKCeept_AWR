# Adeept_AWR
Example Code for Adeept Wheeled Robot

# 카메라 보는 명령어
libcamera-hello -t 0 --rotation 180

# 카메라 영상 저장하는 명령어
libcamera-vid -t 10000 --rotation 180 -o test.h264
ffmpeg -i test.h264 -c copy test.mp4
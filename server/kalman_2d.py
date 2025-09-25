import cv2
import numpy as np

# =========================
# 2D 칼만 필터 초기화
# =========================
class Kalman2D:
    def __init__(self):
        # 상태: [x, y, vx, vy], 측정: [x, y]
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1,0,0,0],
                                              [0,1,0,0]], np.float32)
        self.kf.transitionMatrix = np.array([[1,0,1,0],
                                             [0,1,0,1],
                                             [0,0,1,0],
                                             [0,0,0,1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        # 초기 추정값
        self.initialized = False

    def update(self, cx, cy):
        measured = np.array([[np.float32(cx)], [np.float32(cy)]])
        if not self.initialized:
            # 초기 상태를 첫 측정값으로 세팅
            self.kf.statePre = np.array([[np.float32(cx)],
                                         [np.float32(cy)],
                                         [0.],
                                         [0.]], np.float32)
            self.initialized = True
        self.kf.correct(measured)
        predicted = self.kf.predict()
        fx, fy = predicted[0][0], predicted[1][0]
        return fx, fy

# =========================
# 전역 칼만 필터 객체 생성
# =========================
kf2d = Kalman2D()

# =========================
# kalman_update 함수
# =========================
def kalman_update(cx, cy):
    return kf2d.update(cx, cy)
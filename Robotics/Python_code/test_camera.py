import cv2
import numpy as np
import math

class CameraTest:
    """Класс для тестирования камеры с ArUco-маркерами."""

    def __init__(self):
        """Инициализация камеры и параметров ArUco."""
        self._camera_matrix = np.array([[920.0, 0.0, 650.0], [0.0, 930.0, 320.0], [0.0, 0.0, 1.0]])
        self._dist_coeffs = np.array([[0.06], [-0.1], [-0.006], [0.009], [0]])
        self._cap = cv2.VideoCapture(0)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def run(self):
        """Захват и обработка изображений с камеры."""
        try:
            while True:
                ret, img = self._cap.read()
                if not ret:
                    print("Не удалось захватить кадр с камеры")
                    continue

                aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
                parameters = cv2.aruco.DetectorParameters()
                detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
                corners, ids, _ = detector.detectMarkers(img)

                if ids is not None and len(corners) > 0:
                    cv2.aruco.drawDetectedMarkers(img, corners, ids)
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.12, self._camera_matrix, self._dist_coeffs)
                    if tvec is not None and len(tvec) > 0:
                        R, _ = cv2.Rodrigues(rvec[0])
                        gamma = math.atan2(R[1, 0], R[0, 0])
                        print(f"Detected: x={tvec[0][0][0]:.2f}, y={tvec[0][0][1]:.2f}, gamma={gamma:.2f}")
                        cv2.drawFrameAxes(img, self._camera_matrix, self._dist_coeffs, rvec[0], tvec[0], 0.1)

                cv2.imshow('ArUco Detection', img)
                if cv2.waitKey(10) == 27:  # Выход по ESC
                    break
        finally:
            self._cap.release()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    CameraTest().run()

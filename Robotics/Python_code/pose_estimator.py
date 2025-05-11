import rospy
from turtlesim.msg import Pose
import cv2
import numpy as np
import math

class PoseEstimator:
    """Класс для оценки позиции черепахи с помощью ArUco-маркеров."""

    def __init__(self):
        """Инициализация камеры, подписки и публикации."""
        self._pose_pub = rospy.Publisher('/turtle_pose', Pose, queue_size=10)
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_theta = 0.0
        self._camera_matrix = np.array([[920.0, 0.0, 650.0], [0.0, 930.0, 320.0], [0.0, 0.0, 1.0]])
        self._dist_coeffs = np.array([[0.06], [-0.1], [-0.006], [0.009], [0]])
        self._cap = cv2.VideoCapture(0)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self._timer = rospy.Timer(rospy.Duration(0.01), self._timer_callback)

    def _timer_callback(self, event):
        """Захват изображения и оценка позиции с помощью ArUco-маркеров."""
        pose = Pose()
        ret, img = self._cap.read()
        if not ret:
            rospy.logwarn("Не удалось захватить кадр с камеры")
            return

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(img)

        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.12, self._camera_matrix, self._dist_coeffs)
            if tvec is not None and len(tvec) > 0:
                self._current_x = tvec[0][0][0]
                self._current_y = tvec[0][0][1]
                R, _ = cv2.Rodrigues(rvec[0])
                gamma = math.atan2(R[1, 0], R[0, 0])
                self._current_theta = gamma
                cv2.drawFrameAxes(img, self._camera_matrix, self._dist_coeffs, rvec[0], tvec[0], 0.1)
                rospy.loginfo(f"Detected: x={self._current_x:.2f}, y={self._current_y:.2f}, gamma={gamma:.2f}")
            else:
                self._current_x = -100000.0
                self._current_y = -100000.0
                self._current_theta = 0.0
        else:
            self._current_x = -100000.0
            self._current_y = -100000.0
            self._current_theta = 0.0

        pose.x = self._current_x
        pose.y = self._current_y
        pose.theta = self._current_theta
        rospy.loginfo(f"Pose: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}")
        self._pose_pub.publish(pose)

        cv2.imshow('ArUco Detection', img)
        cv2.waitKey(10)

    def run(self):
        """Запуск узла и ожидание сообщений."""
        try:
            rospy.spin()
        finally:
            self._cap.release()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('pose_estimator_node')
    PoseEstimator().run()

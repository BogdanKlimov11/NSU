from turtlesim.msg import Pose
import rospy
from math import *
import cv2
import numpy

mtx = numpy.matrix([[920.0, 0.0, 650.0],[0.0, 930.0, 320.0],[0.0, 0.0, 1.0]])
dist = numpy.matrix([[0.06], [-0.1], [-0.006], [0.009], [0]])

class My_RodrigeZ:
    def __init__(self) -> None:
        # Подписка на топик для получения положения черепахи
        self.pub = rospy.Publisher('my_turtle_pose', Pose, queue_size = 10)
        
        self.cur_x = 0
        self.cur_y = 0
        self.cur_th = 0

        self.vel_lin_x = 0
        self.vel_ang_z = 0
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
    
    def timer_callback(self):
        # Основной цикл для захвата и обработки изображений с камеры
        my_pose = Pose()

        img = self.cap.read()
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        (corners, ids) = cv2.aruco.detectMarkers(img, dictionary=arucoDict)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            rvec, tvec, _ =  cv2.aruco.estimatePoseSingleMarkers(corners, 0.12, mtx, dist)
            print("Detected: ", tvec)
            R = cv2.Rodrigues(rvec, None, None)
            g_angle = atan2(R[1,0], R[0,0])
            if len(tvec)>0:
                my_pose.x = tvec [0][0][0]
                my_pose.y = tvec [0][0][1]
            my_pose.theta = g_angle 
            print("Gamma: ", g_angle)
            cv2.drawFrameAxes(img, mtx, dist, rvec, tvec, length = 0.1)
        else:
            my_pose.x = -100000
            my_pose.y = -100000
            my_pose.theta = 0
        cv2.imshow('output', img)

        key = cv2.waitKey(10)

        print("My pose: ", my_pose.x, '; ', my_pose.y, '; ', my_pose.theta)
        self.pub.publish(my_pose)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('my_turtle_pose')
    My_RodrigeZ().run()

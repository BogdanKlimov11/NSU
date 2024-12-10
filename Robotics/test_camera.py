import cv2
import numpy
from math import *

mtx = numpy.matrix([[920.0, 0.0, 650.0], [0.0, 930.0, 320.0], [0.0, 0.0, 1.0]])
dist = numpy.matrix([[0.06], [-0.1], [-0.006], [0.009], [0]])

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ret, img = cap.read()
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    # parameters = cv2.aruco.DetectorParameters()
    print("1")
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, dictionary=arucoDict)
    # print("2")
    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        # print("3")
        
        rvec, tvec, _ =  cv2.aruco.estimatePoseSingleMarkers(corners, 0.12, mtx, dist)
        print("Detected: ", tvec)

        R, jac = cv2.Rodrigues(rvec, None, None)
        # print("Rodrogues: ", matrix)
        # b_angle = asin(matrix[0][2])
        # g_angle = asin(matrix[0][1] / (-cos(asin2(matrix[0][2]))))
        # g_angle = g_angle + pi / 2

        # beta = -np.arcsin(R[2,0])
        # alpha = np.arctan2(R[2,1] / np.cos(beta),R[2,2] / np.cos(beta))
        g_angle = atan2(R[1,0], R[0,0])

        print("Gamma: ", g_angle, tvec[0][0][0], tvec[0][0][1])
        cv2.drawFrameAxes(img, mtx, dist, rvec, tvec, length=0.1)
    
    cv2.imshow('output', img)

    key = cv2.waitKey(10)
    # if key == 27:
    #     break

cap.release()
cv2.destroyAllWindows()

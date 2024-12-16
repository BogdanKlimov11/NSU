import cv2
import numpy
from math import *

# Матрица камеры (фокусное расстояние и координаты главной точки)
mtx = numpy.matrix([[920.0, 0.0, 650.0], [0.0, 930.0, 320.0], [0.0, 0.0, 1.0]])
# Коэффициенты искажения камеры
dist = numpy.matrix([[0.06], [-0.1], [-0.006], [0.009], [0]])

# Открытие видеопотока с камеры
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Установка ширины изображения
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Установка высоты изображения

while True:
    ret, img = cap.read()  # Захват одного кадра
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)  # Загрузка словаря маркеров ArUco
    print("1")
    
    # Обнаружение маркеров ArUco на изображении
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, dictionary=arucoDict)
    if len(corners) > 0:  # Если маркеры найдены
        # Отображение обнаруженных маркеров
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        
        # Оценка положения и ориентации маркеров
        rvec, tvec, _ =  cv2.aruco.estimatePoseSingleMarkers(corners, 0.12, mtx, dist)
        print("Detected: ", tvec)  # Печать трансляции маркера

        # Преобразование в матрицу вращения с помощью Rodrigues
        R, jac = cv2.Rodrigues(rvec, None, None)

        # Вычисление угла ориентации маркера относительно оси Z
        g_angle = atan2(R[1,0], R[0,0])

        # Печать угла ориентации и координат маркера
        print("Gamma: ", g_angle, tvec[0][0][0], tvec[0][0][1])

        # Отображение осей на изображении для маркера
        cv2.drawFrameAxes(img, mtx, dist, rvec, tvec, length=0.1)

    # Отображение изображения с обнаруженными маркерами
    cv2.imshow('output', img)

    key = cv2.waitKey(10)  # Ожидание 10 мс для нажатия клавиши
    # if key == 27:  # Прерывание цикла по нажатию клавиши ESC
    #     break

# Закрытие видеопотока и окон OpenCV
cap.release()
cv2.destroyAllWindows()

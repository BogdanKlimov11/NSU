#pragma once

#include <QWidget> // Базовый класс для всех виджетов Qt
#include <QSlider> // Для создания ползунков
#include <QLabel> // Для создания текстовых меток
#include <QVBoxLayout> // Для вертикального компоновщика
#include <QHBoxLayout> // Для горизонтального компоновщика

// Подключение пользовательского заголовочного файла для класса SquareWindow
#include "square_window.h"

// Определение класса MainWindow, наследующегося от QWidget
class MainWindow : public QWidget {
    Q_OBJECT // Макрос Qt для поддержки механизма сигналов и слотов

public:
    // Конструктор класса, принимающий родительский виджет
    explicit MainWindow(QWidget *parent = nullptr);

private:
    // Указатель на объект SquareWindow для отображения 3D-графики
    SquareWindow *squareWindow_;
    // Указатель на ползунок для управления скоростью вращения объекта
    QSlider *rotationSpeedSlider_;
    // Указатель на ползунок для управления скоростью движения света
    QSlider *lightSpeedSlider_;
    // Указатель на метку для отображения текста "Скорость вращения"
    QLabel *rotationSpeedLabel_;
    // Указатель на метку для отображения текста "Скорость света"
    QLabel *lightSpeedLabel_;
};

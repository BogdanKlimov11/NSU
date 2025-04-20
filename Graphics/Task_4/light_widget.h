#pragma once

#include <QColorDialog> // Для отображения диалогового окна выбора цвета
#include <QLabel> // Для создания текстовых меток
#include <QSlider> // Для создания ползунков
#include <QSpinBox> // Для создания числового поля ввода
#include <QRadioButton> // Для создания радиокнопок
#include <QVBoxLayout> // Для создания вертикального компоновщика
#include <QWidget> // Базовый класс для всех виджетов Qt
#include <type_traits> // Для работы с типами данных (например, проверка типов)
#include <QPushButton> // Для создания кнопок

// Подключение пользовательского заголовочного файла для класса SquareWindow
#include "square_window.h"

// Определение класса LightWidget, наследующегося от QWidget
class LightWidget : public QWidget {
    Q_OBJECT // Макрос Qt для поддержки механизма сигналов и слотов

public:
    // Конструктор класса, принимающий указатель на SquareWindow и родительский виджет
    LightWidget(SquareWindow *squareWindow, QWidget *parent = nullptr);

private:
    // Указатель на ползунок для управления параметром морфинга (например, для изменения формы объекта)
    QSlider *morphSlider;
    // Указатель на числовое поле ввода для задания значения параметра t (например, плотности сетки)
    QSpinBox *tSpinBox;
    // Указатели на ползунки для управления положением источника света по осям X, Y, Z
    QSlider *sun_Slider_x;
    QSlider *sun_Slider_y;
    QSlider *sun_Slider_z;
    // Указатель на метку для отображения текста "Плотность сетки (n)"
    QLabel *nLabel;
    // Указатели на метки для отображения текста для параметров света по осям X, Y, Z
    QLabel *lightXLabel;
    QLabel *lightYLabel;
    QLabel *lightZLabel;
    // Указатель на метку для отображения текста "Морфинг"
    QLabel *morphLabel;
    // Указатели на радиокнопки для выбора типа источника света
    QRadioButton *directionalLightRadio; // Радиокнопка для направленного света
    QRadioButton *pointLightRadio; // Радиокнопка для точечного света
    QRadioButton *spotlightRadio; // Радиокнопка для прожекторного света
    // Указатель на ползунок для управления интенсивностью света
    QSlider *intensitySlider;
    // Указатель на ползунок для управления затуханием света (для точечного или прожекторного света)
    QSlider *attenuationSlider;
    // Указатель на ползунок для управления углом обрезки света (для прожектора)
    QSlider *cutoffSlider;
    // Указатель на метку для отображения информации о частоте кадров (FPS)
    QLabel *fpsLabel;
};

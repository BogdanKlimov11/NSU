#pragma once

#include <QColorDialog> // Для отображения диалогового окна выбора цвета
#include <QLabel> // Для создания текстовых меток
#include <QSlider> // Для создания ползунков
#include <QSpinBox> // Для создания числового поля ввода
#include <QRadioButton> // Для создания переключателей (радиокнопок)
#include <QVBoxLayout> // Для создания вертикального компоновщика
#include <QWidget> // Базовый класс для всех виджетов Qt
#include <type_traits> // Для работы с типами данных (например, проверка типов)
#include <QPushButton> // Для создания кнопок

// Подключение заголовочного файла пользовательского класса SquareWindow
#include "square_window.h"

// Объявление класса LightWidget, наследующегося от QWidget
class LightWidget : public QWidget {
    Q_OBJECT // Макрос Qt для поддержки сигналов и слотов

public:
    // Конструктор класса, принимающий указатель на SquareWindow и родительский виджет
    explicit LightWidget(SquareWindow *renderWindow, QWidget *parent = nullptr);

private:
    // Указатель на объект SquareWindow, используемый для взаимодействия с окном рендеринга
    SquareWindow *renderWindow_;

    // Указатель на ползунок для управления морфингом (например, для изменения формы объекта)
    QSlider *morphSlider;
    // Указатель на поле ввода для установки значения параметра t (например, для анимации)
    QSpinBox *tSpinBox;
    // Указатели на ползунки для управления положением источника света по осям X, Y, Z
    QSlider *sun_Slider_x;
    QSlider *sun_Slider_y;
    QSlider *sun_Slider_z;
    // Указатель на метку для отображения значения параметра n (например, для количества элементов)
    QLabel *nLabel;
    // Указатели на метки для отображения текущих значений координат источника света (X, Y, Z)
    QLabel *lightXLabel;
    QLabel *lightYLabel;
    QLabel *lightZLabel;
    // Указатель на метку для отображения текущего значения морфинга
    QLabel *morphLabel;

    // Указатели на радиокнопки для выбора типа источника света
    QRadioButton *directionalLightRadio; // Направленный свет
    QRadioButton *pointLightRadio; // Точечный свет
    QRadioButton *spotlightRadio; // Прожекторный свет
    // Указатель на ползунок для управления интенсивностью света
    QSlider *intensitySlider;
    // Указатель на ползунок для управления затуханием света (например, для точечного источника)
    QSlider *attenuationSlider;
    // Указатель на ползунок для управления углом обрезки света (например, для прожектора)
    QSlider *cutoffSlider;

    // Указатели на кнопки для выбора цвета материала
    QPushButton *materialAmbientButton; // Кнопка для выбора фонового цвета материала
    QPushButton *materialDiffuseButton; // Кнопка для выбора диффузного цвета материала
    QPushButton *materialSpecularButton; // Кнопка для выбора зеркального цвета материала
    // Указатель на кнопку для выбора глобального фонового освещения
    QPushButton *globalAmbientButton;
};

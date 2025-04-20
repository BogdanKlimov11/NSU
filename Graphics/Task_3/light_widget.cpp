#include "light_widget.h"

// Шаблонный псевдоним main, который используется для указателей на типы T
// Использует std::enable_if_t для ограничения использования только указателями
template <typename T, typename = std::enable_if_t<std::is_pointer<T>::value>>
using main = T;

// Вспомогательная функция для создания и настройки горизонтального ползунка QSlider
auto slider(int min_value, int max_value, int tick_value) {
    // Создание нового объекта QSlider с горизонтальной ориентацией
    auto *const slider = main<QSlider*>(new QSlider(Qt::Orientation::Horizontal));
    // Установка диапазона значений ползунка (от min_value до max_value)
    slider->setRange(min_value, max_value);
    // Установка шага изменения значения ползунка
    slider->setSingleStep(tick_value);
    // Установка положения делений (справа от ползунка)
    slider->setTickPosition(QSlider::TicksRight);
    // Установка начального значения ползунка равным максимальному
    slider->setValue(max_value);
    // Возврат указателя на созданный ползунок
    return slider;
}

// Конструктор класса LightWidget, принимающий указатель на SquareWindow и родительский виджет
LightWidget::LightWidget(SquareWindow *renderWindow, QWidget *parent)
    : QWidget(parent), renderWindow_(renderWindow) {
    // Создание компоновщика QGridLayout для размещения виджетов в виде сетки
    auto *const container = main<QGridLayout *>(new QGridLayout);
    // Создание основного компоновщика QVBoxLayout для размещения виджетов по вертикали
    auto *const mainLayout = main<QVBoxLayout *>(new QVBoxLayout);
    // Создание контейнерного виджета для размещения сетки
    auto *const w = main<QWidget *>(new QWidget);

    // Создание поля ввода QSpinBox для установки плотности сетки (n)
    tSpinBox = new QSpinBox;
    // Установка диапазона значений для поля ввода (от 2 до 100)
    tSpinBox->setRange(2, 100);
    // Установка начального значения поля ввода
    tSpinBox->setValue(10);
    // Установка шага изменения значения
    tSpinBox->setSingleStep(1);

    // Создание ползунков для управления положением источника света по осям X, Y, Z
    sun_Slider_x = slider(-100, 100, 5); // Ползунок для оси X (диапазон: -100..100, шаг: 5)
    sun_Slider_y = slider(-100, 100, 5); // Ползунок для оси Y (диапазон: -100..100, шаг: 5)
    sun_Slider_z = slider(-100, 100, 5); // Ползунок для оси Z (диапазон: -100..100, шаг: 5)
    // Создание ползунка для управления параметром морфинга
    morphSlider = slider(0, 1000, 10); // Ползунок для морфинга (диапазон: 0..1000, шаг: 10)

    // Создание текстовых меток для отображения подписей к элементам интерфейса
    nLabel = new QLabel("Плотность сетки (n):"); // Метка для поля ввода плотности сетки
    lightXLabel = new QLabel("Свет X:"); // Метка для ползунка положения света по X
    lightYLabel = new QLabel("Свет Y:"); // Метка для ползунка положения света по Y
    lightZLabel = new QLabel("Свет Z:"); // Метка для ползунка положения света по Z
    morphLabel = new QLabel("Морфинг:"); // Метка для ползунка морфинга
    // Создание радиокнопок для выбора типа источника света
    directionalLightRadio = new QRadioButton("Направленный свет"); // Радиокнопка для направленного света
    pointLightRadio = new QRadioButton("Точечный свет"); // Радиокнопка для точечного света
    spotlightRadio = new QRadioButton("Прожектор"); // Радиокнопка для прожекторного света
    // Установка направленного света как выбранного по умолчанию
    directionalLightRadio->setChecked(true);
    // Создание ползунков для управления параметрами света
    intensitySlider = slider(1, 100, 1); // Ползунок для интенсивности света (диапазон: 1..100, шаг: 1)
    attenuationSlider = slider(1, 100, 1); // Ползунок для затухания света (диапазон: 1..100, шаг: 1)
    cutoffSlider = slider(70, 100, 1); // Ползунок для угла обрезки света (диапазон: 70..100, шаг: 1)

    // Создание кнопки для изменения цвета куба
    QPushButton *change_color_cube_button = new QPushButton(tr("&Цвет куба"));
    // Отключение фокуса для кнопки (чтобы она не перехватывала фокус при клике)
    change_color_cube_button->setFocusPolicy(Qt::NoFocus);

    // Создание кнопки для изменения цвета источника света
    QPushButton *change_color_light_button = new QPushButton(tr("&Цвет света"));
    // Отключение фокуса для кнопки
    change_color_light_button->setFocusPolicy(Qt::NoFocus);

    // Создание кнопок для выбора свойств материала
    materialAmbientButton = new QPushButton(tr("&Ambient материал")); // Кнопка для фонового освещения материала
    materialAmbientButton->setFocusPolicy(Qt::NoFocus); // Отключение фокуса
    materialDiffuseButton = new QPushButton(tr("&Diffuse материал")); // Кнопка для диффузного освещения материала
    materialDiffuseButton->setFocusPolicy(Qt::NoFocus); // Отключение фокуса
    materialSpecularButton = new QPushButton(tr("&Specular материал")); // Кнопка для зеркального освещения материала
    materialSpecularButton->setFocusPolicy(Qt::NoFocus); // Отключение фокуса
    globalAmbientButton = new QPushButton(tr("&Фоновое освещение")); // Кнопка для глобального фонового освещения
    globalAmbientButton->setFocusPolicy(Qt::NoFocus); // Отключение фокуса

    // Размещение виджетов в сетке QGridLayout
    int row = 0; // Начальная строка для размещения виджетов
    container->addWidget(nLabel, row, 0); // Добавление метки плотности сетки
    container->addWidget(tSpinBox, row++, 1); // Добавление поля ввода плотности сетки
    container->addWidget(lightXLabel, row, 0); // Добавление метки для света X
    container->addWidget(sun_Slider_x, row++, 1); // Добавление ползунка для света X
    container->addWidget(lightYLabel, row, 0); // Добавление метки для света Y
    container->addWidget(sun_Slider_y, row++, 1); // Добавление ползунка для света Y
    container->addWidget(lightZLabel, row, 0); // Добавление метки для света Z
    container->addWidget(sun_Slider_z, row++, 1); // Добавление ползунка для света Z
    container->addWidget(morphLabel, row, 0); // Добавление метки для морфинга
    container->addWidget(morphSlider, row++, 1); // Добавление ползунка для морфинга
    container->addWidget(change_color_cube_button, row, 0); // Добавление кнопки изменения цвета куба
    container->addWidget(change_color_light_button, row++, 1); // Добавление кнопки изменения цвета света
    container->addWidget(new QLabel("Тип света:"), row, 0); // Добавление метки для типа света
    container->addWidget(directionalLightRadio, row++, 1); // Добавление радиокнопки направленного света
    container->addWidget(pointLightRadio, row, 0); // Добавление радиокнопки точечного света
    container->addWidget(spotlightRadio, row++, 1); // Добавление радиокнопки прожекторного света
    container->addWidget(new QLabel("Интенсивность:"), row, 0); // Добавление метки для интенсивности
    container->addWidget(intensitySlider, row++, 1); // Добавление ползунка для интенсивности
    container->addWidget(new QLabel("Затухание:"), row, 0); // Добавление метки для затухания
    container->addWidget(attenuationSlider, row++, 1); // Добавление ползунка для затухания
    container->addWidget(new QLabel("Угол обреза:"), row, 0); // Добавление метки для угла обрезки
    container->addWidget(cutoffSlider, row++, 1); // Добавление ползунка для угла обрезки
    container->addWidget(materialAmbientButton, row, 0); // Добавление кнопки для фонового освещения материала
    container->addWidget(materialDiffuseButton, row++, 1); // Добавление кнопки для диффузного освещения материала
    container->addWidget(materialSpecularButton, row, 0); // Добавление кнопки для зеркального освещения материала
    container->addWidget(globalAmbientButton, row++, 1); // Добавление кнопки для глобального фонового освещения

    // Установка компоновщика container для виджета w
    w->setLayout(container);
    // Добавление виджета w в основной компоновщик mainLayout
    mainLayout->addWidget(w);
    // Установка основного компоновщика для текущего виджета
    setLayout(mainLayout);

    // Соединение сигнала изменения значения поля ввода tSpinBox с методом changeN объекта renderWindow_
    connect(tSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), renderWindow_, &SquareWindow::changeN);
    // Соединение сигналов изменения значений ползунков света с соответствующими методами renderWindow_
    connect(sun_Slider_x, &QSlider::valueChanged, renderWindow_, &SquareWindow::change_light_x_param);
    connect(sun_Slider_y, &QSlider::valueChanged, renderWindow_, &SquareWindow::change_light_y_param);
    connect(sun_Slider_z, &QSlider::valueChanged, renderWindow_, &SquareWindow::change_light_z_param);
    // Соединение сигнала изменения значения ползунка морфинга с методом change_morph_param
    connect(morphSlider, &QSlider::valueChanged, renderWindow_, &SquareWindow::change_morph_param);
    // Соединение сигналов нажатия кнопок изменения цвета с соответствующими методами renderWindow_
    connect(change_color_cube_button, &QPushButton::clicked, renderWindow_, &SquareWindow::change_cube_color);
    connect(change_color_light_button, &QPushButton::clicked, renderWindow_, &SquareWindow::change_light_color);

    // Соединение сигнала нажатия радиокнопки направленного света с установкой типа света (0)
    connect(directionalLightRadio, &QRadioButton::clicked, [this]() {
        renderWindow_->setLightType(0);
    });

    // Соединение сигнала нажатия радиокнопки точечного света с установкой типа света (1)
    connect(pointLightRadio, &QRadioButton::clicked, [this]() {
        renderWindow_->setLightType(1);
    });

    // Соединение сигнала нажатия радиокнопки прожекторного света с установкой типа света (2)
    connect(spotlightRadio, &QRadioButton::clicked, [this]() {
        renderWindow_->setLightType(2);
    });

    // Соединение сигналов изменения значений ползунков параметров света с соответствующими методами renderWindow_
    connect(intensitySlider, &QSlider::valueChanged, renderWindow_, &SquareWindow::setLightIntensity);
    connect(attenuationSlider, &QSlider::valueChanged, renderWindow_, &SquareWindow::setLightAttenuation);
    connect(cutoffSlider, &QSlider::valueChanged, renderWindow_, &SquareWindow::setLightCutoff);

    // Соединение сигналов нажатия кнопок свойств материала с соответствующими методами renderWindow_
    connect(materialAmbientButton, &QPushButton::clicked, renderWindow_, &SquareWindow::setMaterialAmbient);
    connect(materialDiffuseButton, &QPushButton::clicked, renderWindow_, &SquareWindow::setMaterialDiffuse);
    connect(materialSpecularButton, &QPushButton::clicked, renderWindow_, &SquareWindow::setMaterialSpecular);
    connect(globalAmbientButton, &QPushButton::clicked, renderWindow_, &SquareWindow::setGlobalAmbient);
}

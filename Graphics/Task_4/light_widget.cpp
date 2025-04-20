#include "light_widget.h"

// Шаблонный псевдоним main, используемый для указателей на типы T
// Ограничен только указателями с помощью std::enable_if_t
template <typename T, typename = std::enable_if_t<std::is_pointer<T>::value>>
using main = T;

// Вспомогательная функция для создания и настройки горизонтального ползунка QSlider
auto slider(int min_value, int max_value, int tick_value) {
    // Создание нового объекта QSlider с горизонтальной ориентацией
    auto *const slider = main<QSlider*>(new QSlider(Qt::Orientation::Horizontal));
    // Установка диапазона значений ползунка (от min_value до max_value)
    slider->setRange(min_value, max_value);
    // Установка шага изменения значения
    slider->setSingleStep(tick_value);
    // Установка положения делений (справа от ползунка)
    slider->setTickPosition(QSlider::TicksRight);
    // Установка начального значения ползунка равным максимальному
    slider->setValue(max_value);
    // Возврат указателя на созданный ползунок
    return slider;
}

// Конструктор класса LightWidget, принимающий указатель на SquareWindow и родительский виджет
LightWidget::LightWidget(SquareWindow *my_window, QWidget *parent) : QWidget(parent) {
    // Создание компоновщика QGridLayout для размещения виджетов в виде сетки
    auto *const container = main<QGridLayout *>(new QGridLayout);
    // Создание основного компоновщика QVBoxLayout для вертикального размещения
    auto *const mainLayout = main<QVBoxLayout *>(new QVBoxLayout);
    // Создание контейнерного виджета для размещения сетки
    auto *const w = main<QWidget *>(new QWidget);

    // Создание поля ввода QSpinBox для установки плотности сетки (n)
    tSpinBox = new QSpinBox;
    // Установка диапазона значений (от 2 до 100)
    tSpinBox->setRange(2, 100);
    // Установка начального значения
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
    lightXLabel = new QLabel("Свет X:"); // Метка для ползунка света по оси X
    lightYLabel = new QLabel("Свет Y:"); // Метка для ползунка света по оси Y
    lightZLabel = new QLabel("Свет Z:"); // Метка для ползунка света по оси Z
    morphLabel = new QLabel("Morphing:"); // Метка для ползунка морфинга
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

    // Создание метки для отображения текущей частоты кадров (FPS)
    fpsLabel = new QLabel("FPS: 0");
    // Выравнивание текста метки по левому краю
    fpsLabel->setAlignment(Qt::AlignLeft);

    // Создание кнопки для изменения цвета куба
    QPushButton *change_color_cube_button = new QPushButton(tr("Цвет куба"));
    // Отключение фокуса для кнопки (чтобы она не перехватывала фокус при клике)
    change_color_cube_button->setFocusPolicy(Qt::NoFocus);

    // Создание кнопки для изменения цвета источника света
    QPushButton *change_color_light_button = new QPushButton(tr("Цвет света"));
    // Отключение фокуса для кнопки
    change_color_light_button->setFocusPolicy(Qt::NoFocus);

    // Создание горизонтального компоновщика для размещения метки FPS
    auto *fpsLayout = new QHBoxLayout;
    // Добавление метки FPS в компоновщик с выравниванием по центру
    fpsLayout->addWidget(fpsLabel, 0, Qt::AlignCenter);
    // Добавление компоновщика FPS в сетку (первая строка, занимает 3 столбца)
    container->addLayout(fpsLayout, 0, 0, 1, 3);
    // Размещение остальных виджетов в сетке
    container->addWidget(nLabel, 1, 0); // Метка плотности сетки
    container->addWidget(tSpinBox, 1, 1); // Поле ввода плотности сетки
    container->addWidget(lightXLabel, 2, 0); // Метка для света X
    container->addWidget(sun_Slider_x, 2, 1); // Ползунок для света X
    container->addWidget(lightYLabel, 3, 0); // Метка для света Y
    container->addWidget(sun_Slider_y, 3, 1); // Ползунок для света Y
    container->addWidget(lightZLabel, 4, 0); // Метка для света Z
    container->addWidget(sun_Slider_z, 4, 1); // Ползунок для света Z
    container->addWidget(morphLabel, 5, 0); // Метка для морфинга
    container->addWidget(morphSlider, 5, 1); // Ползунок для морфинга
    container->addWidget(change_color_cube_button, 6, 0); // Кнопка изменения цвета куба
    container->addWidget(change_color_light_button, 6, 1); // Кнопка изменения цвета света
    container->addWidget(new QLabel("Тип света:"), 7, 0); // Метка для типа света
    container->addWidget(directionalLightRadio, 7, 1); // Радиокнопка направленного света
    container->addWidget(pointLightRadio, 8, 0); // Радиокнопка точечного света
    container->addWidget(spotlightRadio, 8, 1); // Радиокнопка прожекторного света
    container->addWidget(new QLabel("Интенсивность:"), 9, 0); // Метка для интенсивности
    container->addWidget(intensitySlider, 9, 1); // Ползунок для интенсивности
    container->addWidget(new QLabel("Затухание:"), 10, 0); // Метка для затухания
    container->addWidget(attenuationSlider, 10, 1); // Ползунок для затухания
    container->addWidget(new QLabel("Угол обреза:"), 11, 0); // Метка для угла обрезки
    container->addWidget(cutoffSlider, 11, 1); // Ползунок для угла обрезки

    // Установка компоновщика container для виджета w
    w->setLayout(container);
    // Добавление виджета w в основной компоновщик mainLayout
    mainLayout->addWidget(w);

    // Установка основного компоновщика для текущего виджета
    setLayout(mainLayout);

    // Соединение сигнала изменения значения поля ввода tSpinBox с методом changeN объекта my_window
    connect(tSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), my_window, &SquareWindow::changeN);
    // Соединение сигналов изменения значений ползунков света с соответствующими методами my_window
    connect(sun_Slider_x, &QSlider::valueChanged, my_window, &SquareWindow::change_light_x_param);
    connect(sun_Slider_y, &QSlider::valueChanged, my_window, &SquareWindow::change_light_y_param);
    connect(sun_Slider_z, &QSlider::valueChanged, my_window, &SquareWindow::change_light_z_param);
    // Соединение сигнала изменения значения ползунка морфинга с методом change_morph_param
    connect(morphSlider, &QSlider::valueChanged, my_window, &SquareWindow::change_morph_param);
    // Соединение сигналов нажатия кнопок изменения цвета с соответствующими методами my_window
    connect(change_color_cube_button, &QPushButton::clicked, my_window, &SquareWindow::change_cube_color);
    connect(change_color_light_button, &QPushButton::clicked, my_window, &SquareWindow::change_light_color);
    // Соединение сигнала нажатия радиокнопки направленного света с установкой типа света (0)
    connect(directionalLightRadio, &QRadioButton::clicked, [my_window]() {
        my_window->setLightType(0);
    });
    // Соединение сигнала нажатия радиокнопки точечного света с установкой типа света (1)
    connect(pointLightRadio, &QRadioButton::clicked, [my_window]() {
        my_window->setLightType(1);
    });
    // Соединение сигнала нажатия радиокнопки прожекторного света с установкой типа света (2)
    connect(spotlightRadio, &QRadioButton::clicked, [my_window]() {
        my_window->setLightType(2);
    });
    // Соединение сигналов изменения значений ползунков параметров света с соответствующими методами my_window
    connect(intensitySlider, &QSlider::valueChanged, my_window, &SquareWindow::setLightIntensity);
    connect(attenuationSlider, &QSlider::valueChanged, my_window, &SquareWindow::setLightAttenuation);
    connect(cutoffSlider, &QSlider::valueChanged, my_window, &SquareWindow::setLightCutoff);

    // Соединение сигнала обновления FPS от my_window с обновлением текста метки fpsLabel
    connect(my_window, &SquareWindow::fpsUpdated, this, [this](int fps) {
        // Вывод значения FPS в консоль для отладки
        qDebug() << "FPS updated in LightWidget:" << fps;
        // Обновление текста метки FPS с новым значением
        fpsLabel->setText(QString("FPS: %1").arg(fps));
    });
}

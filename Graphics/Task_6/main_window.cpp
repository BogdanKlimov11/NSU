#include <QDebug> // Для вывода отладочной информации

#include "main_window.h" // Подключение заголовочного файла класса MainWindow

// Конструктор класса MainWindow
MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
    // Создание OpenGL-виджета для отображения 3D-графики
    squareWindow_ = new SquareWindow(this);

    // Создание вертикального ползунка для управления скоростью вращения
    rotationSpeedSlider_ = new QSlider(Qt::Vertical, this);
    rotationSpeedSlider_->setRange(0, 200); // Диапазон значений: 0–200 (будет умножен на 0.1)
    rotationSpeedSlider_->setValue(100); // Начальное значение: 100 (скорость = 10.0)
    rotationSpeedSlider_->setTickPosition(QSlider::TicksRight); // Деления справа
    rotationSpeedSlider_->setTickInterval(10); // Интервал делений: 10

    // Создание вертикального ползунка для управления скоростью движения света
    lightSpeedSlider_ = new QSlider(Qt::Vertical, this);
    lightSpeedSlider_->setRange(0, 100); // Диапазон значений: 0–100 (будет умножен на 0.01)
    lightSpeedSlider_->setValue(10); // Начальное значение: 10 (скорость = 0.1)
    lightSpeedSlider_->setTickPosition(QSlider::TicksRight); // Деления справа
    lightSpeedSlider_->setTickInterval(10); // Интервал делений: 10

    // Создание текстовых меток для отображения текущих значений скоростей
    rotationSpeedLabel_ = new QLabel("Rotation Speed: 10.0", this);
    lightSpeedLabel_ = new QLabel("Light Speed: 0.10", this);

    // Создание вертикального компоновщика для панели ползунков
    QVBoxLayout *sliderLayout = new QVBoxLayout();
    sliderLayout->addWidget(new QLabel("Вращение:", this)); // Метка для вращения
    sliderLayout->addWidget(rotationSpeedSlider_); // Ползунок вращения
    sliderLayout->addWidget(rotationSpeedLabel_); // Метка текущей скорости вращения
    sliderLayout->addSpacing(10); // Отступ
    sliderLayout->addWidget(new QLabel("Свет:", this)); // Метка для света
    sliderLayout->addWidget(lightSpeedSlider_); // Ползунок света
    sliderLayout->addWidget(lightSpeedLabel_); // Метка текущей скорости света
    sliderLayout->setAlignment(Qt::AlignTop); // Выравнивание содержимого по верхнему краю

    // Создание виджета для панели ползунков
    QWidget *sliderPanel = new QWidget(this);
    sliderPanel->setLayout(sliderLayout);
    sliderPanel->setMinimumWidth(100); // Установка минимальной ширины панели

    // Создание основного горизонтального компоновщика
    QHBoxLayout *mainLayout = new QHBoxLayout(this);
    mainLayout->addWidget(squareWindow_, 1); // OpenGL-виджет растягивается
    mainLayout->addWidget(sliderPanel, 0); // Панель ползунков имеет фиксированный размер
    mainLayout->setContentsMargins(0, 0, 0, 0); // Установка нулевых отступов

    // Установка компоновщика для окна
    setLayout(mainLayout);

    // Подключение сигнала изменения значения ползунка вращения
    connect(rotationSpeedSlider_, &QSlider::valueChanged, this, [=](int value) {
        float speed = value * 0.1f; // Преобразование значения в скорость (0–20)
        squareWindow_->setRotationSpeed(speed); // Установка скорости вращения
        rotationSpeedLabel_->setText(QString("Rotation Speed: %1").arg(speed, 0, 'f', 2)); // Обновление метки
        qDebug() << "Rotation speed set to:" << speed; // Отладочный вывод
    });

    // Подключение сигнала изменения значения ползунка скорости света
    connect(lightSpeedSlider_, &QSlider::valueChanged, this, [=](int value) {
        float speed = value * 0.01f; // Преобразование значения в скорость (0–1)
        squareWindow_->setLightSpeed(speed); // Установка скорости света
        lightSpeedLabel_->setText(QString("Light Speed: %1").arg(speed, 0, 'f', 2)); // Обновление метки
        qDebug() << "Light speed set to:" << speed; // Отладочный вывод
    });

    // Установка начальных значений скоростей
    squareWindow_->setRotationSpeed(10.0f);
    squareWindow_->setLightSpeed(0.1f);

    // Установка заголовка окна
    setWindowTitle("Дисплей");
    // Установка начального размера окна (800x600 пикселей)
    resize(800, 600);
}

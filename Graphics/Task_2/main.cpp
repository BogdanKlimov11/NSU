#include <QVBoxLayout> // Вертикальная компоновка
#include <QHBoxLayout> // Горизонтальная компоновка
#include <QApplication> // Основной класс приложения Qt
#include <QSlider> // Ползунок для управления параметрами
#include <QLineEdit> // Поле ввода текста
#include <QIntValidator> // Валидатор для целочисленных значений
#include <QDoubleValidator> // Валидатор для вещественных чисел
#include <QLabel> // Метка для текста

#include "transform_widget.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv); // Создание объекта приложения Qt

    // Окно для отображения сцены
    auto scene_window = new QWidget; // Создание основного окна сцены
    auto scene_widget = new SceneWidget(scene_window); // Создание виджета OpenGL сцены
    auto scene_layout = new QVBoxLayout(scene_window); // Создание вертикальной компоновки для окна
    scene_layout->addWidget(scene_widget); // Добавление виджета сцены в компоновку
    scene_window->setWindowTitle("Дисплей"); // Установка заголовка окна
    scene_widget->setMinimumSize(640, 640); // Установка минимального размера виджета сцены
    scene_window->resize(640, 640); // Установка начального размера окна

    // Окно для управления параметрами
    auto control_window = new QWidget; // Создание окна управления

    // Создание элементов управления: ползунки и поля ввода
    auto morphing_slider = new QSlider(Qt::Horizontal, control_window); // Ползунок для морфинга
    auto morphing_edit = new QLineEdit(control_window); // Поле ввода для морфинга
    auto split_parameter_slider = new QSlider(Qt::Horizontal, control_window); // Ползунок для параметра разделения
    auto split_parameter_edit = new QLineEdit(control_window); // Поле ввода для параметра разделения
    auto rotation_x_slider = new QSlider(Qt::Horizontal, control_window); // Ползунок для вращения по оси X
    auto rotation_y_slider = new QSlider(Qt::Horizontal, control_window); // Ползунок для вращения по оси Y
    auto rotation_z_slider = new QSlider(Qt::Horizontal, control_window); // Ползунок для вращения по оси Z
    auto rotation_x_edit = new QLineEdit(control_window); // Поле ввода для вращения по X
    auto rotation_y_edit = new QLineEdit(control_window); // Поле ввода для вращения по Y
    auto rotation_z_edit = new QLineEdit(control_window); // Поле ввода для вращения по Z

    // Установка диапазонов для ползунков
    rotation_x_slider->setRange(0, 360); // Диапазон вращения по X: 0–360 градусов
    rotation_y_slider->setRange(0, 360); // Диапазон вращения по Y: 0–360 градусов
    rotation_z_slider->setRange(0, 360); // Диапазон вращения по Z: 0–360 градусов
    morphing_slider->setRange(0, 100); // Диапазон морфинга: 0–100 (будет преобразован в 0–1)
    split_parameter_slider->setRange(1, 99); // Диапазон разделения: 1–99

    // Настройка полей ввода для углов вращения
    rotation_x_edit->setAlignment(Qt::AlignHCenter); // Выравнивание текста по центру
    rotation_y_edit->setAlignment(Qt::AlignHCenter); // Выравнивание текста по центру
    rotation_z_edit->setAlignment(Qt::AlignHCenter); // Выравнивание текста по центру
    rotation_x_edit->setValidator(new QIntValidator(0, 360, rotation_x_edit)); // Ограничение ввода: 0–360
    rotation_y_edit->setValidator(new QIntValidator(0, 360, rotation_y_edit)); // Ограничение ввода: 0–360
    rotation_z_edit->setValidator(new QIntValidator(0, 360, rotation_z_edit)); // Ограничение ввода: 0–360
    rotation_x_edit->setText("0"); // Начальное значение для X
    rotation_y_edit->setText("0"); // Начальное значение для Y
    rotation_z_edit->setText("0"); // Начальное значение для Z

    // Настройка поля ввода и ползунка для морфинга
    morphing_edit->setAlignment(Qt::AlignHCenter); // Выравнивание текста по центру
    morphing_edit->setValidator(new QDoubleValidator(0.0, 1.0, 2, morphing_edit)); // Ограничение ввода: 0.0–1.0 с 2 знаками после запятой
    morphing_edit->setText("0.00"); // Начальное значение для морфинга
    morphing_slider->setValue(0); // Начальное положение ползунка морфинга

    // Настройка поля ввода и ползунка для параметра разделения
    split_parameter_edit->setAlignment(Qt::AlignHCenter); // Выравнивание текста по центру
    split_parameter_edit->setValidator(new QIntValidator(1, 99, split_parameter_edit)); // Ограничение ввода: 1–99
    split_parameter_edit->setText("10"); // Начальное значение для разделения
    split_parameter_slider->setValue(10); // Начальное положение ползунка разделения

    // Подключение ползунков к методам Scene_widget и полям ввода
    QObject::connect(rotation_x_slider, &QAbstractSlider::valueChanged, [=](int value) {
        scene_widget->set_rotation_x(float(value)); // Установка угла X
        rotation_x_edit->setText(QString::number(value)); // Синхронизация с полем ввода
        scene_widget->update(); // Обновление сцены
    });
    QObject::connect(rotation_y_slider, &QAbstractSlider::valueChanged, [=](int value) {
        scene_widget->set_rotation_y(float(value)); // Установка угла Y
        rotation_y_edit->setText(QString::number(value)); // Синхронизация с полем ввода
        scene_widget->update(); // Обновление сцены
    });
    QObject::connect(rotation_z_slider, &QAbstractSlider::valueChanged, [=](int value) {
        scene_widget->set_rotation_z(float(value)); // Установка угла Z
        rotation_z_edit->setText(QString::number(value)); // Синхронизация с полем ввода
        scene_widget->update(); // Обновление сцены
    });
    QObject::connect(morphing_slider, &QAbstractSlider::valueChanged, [=](int value) {
        float morph_value = float(value) / 100.0f; // Преобразование из 0–100 в 0–1
        scene_widget->set_morphing_parameter(morph_value); // Установка морфинга
        morphing_edit->setText(QString::number(morph_value, 'f', 2)); // Синхронизация с полем ввода (2 знака после запятой)
        scene_widget->update(); // Обновление сцены
    });
    QObject::connect(split_parameter_slider, &QAbstractSlider::valueChanged, [=](int value) {
        scene_widget->set_split_parameter(size_t(value)); // Установка параметра разделения
        split_parameter_edit->setText(QString::number(value)); // Синхронизация с полем ввода
        scene_widget->update(); // Обновление сцены
    });

    // Подключение полей ввода к методам Scene_widget и ползункам
    QObject::connect(rotation_x_edit, &QLineEdit::textChanged, [=](const QString &text) {
        int value = text.toInt(); // Получение значения из поля
        scene_widget->set_rotation_x(float(value)); // Установка угла X
        rotation_x_slider->setValue(value); // Синхронизация с ползунком
        scene_widget->update(); // Обновление сцены
    });
    QObject::connect(rotation_y_edit, &QLineEdit::textChanged, [=](const QString &text) {
        int value = text.toInt(); // Получение значения из поля
        scene_widget->set_rotation_y(float(value)); // Установка угла Y
        rotation_y_slider->setValue(value); // Синхронизация с ползунком
        scene_widget->update(); // Обновление сцены
    });
    QObject::connect(rotation_z_edit, &QLineEdit::textChanged, [=](const QString &text) {
        int value = text.toInt(); // Получение значения из поля
        scene_widget->set_rotation_z(float(value)); // Установка угла Z
        rotation_z_slider->setValue(value); // Синхронизация с ползунком
        scene_widget->update(); // Обновление сцены
    });
    QObject::connect(morphing_edit, &QLineEdit::textChanged, [=](const QString &text) {
        float value = text.toFloat(); // Получение значения из поля
        if (value >= 0.0f && value <= 1.0f) { // Проверка диапазона
            scene_widget->set_morphing_parameter(value); // Установка морфинга
            morphing_slider->setValue(int(value * 100)); // Синхронизация с ползунком (0–100)
            scene_widget->update(); // Обновление сцены
        }
    });
    QObject::connect(split_parameter_edit, &QLineEdit::textChanged, [=](const QString &text) {
        int value = text.toInt(); // Получение значения из поля
        // Проверка диапазона
        if (value >= 1 && value <= 99) {
            scene_widget->set_split_parameter(size_t(value)); // Установка параметра разделения
            split_parameter_slider->setValue(value); // Синхронизация с ползунком
            scene_widget->update(); // Обновление сцены
        }
    });

    // Метки для элементов управления
    auto rotation_x_label = new QLabel("Ось X:", control_window); // Метка для вращения по X
    auto rotation_y_label = new QLabel("Ось Y:", control_window); // Метка для вращения по Y
    auto rotation_z_label = new QLabel("Ось Z:", control_window); // Метка для вращения по Z
    auto morphing_label = new QLabel("Морфинг:", control_window); // Метка для морфинга
    auto split_label = new QLabel("Параметр разделения:", control_window); // Метка для параметра разделения

    // Компоновка для окна управления
    auto control_layout = new QVBoxLayout(control_window); // Создание вертикальной компоновки
    control_layout->addWidget(rotation_x_label); // Добавление метки X
    control_layout->addWidget(rotation_x_slider); // Добавление ползунка X
    control_layout->addWidget(rotation_x_edit); // Добавление поля ввода X
    control_layout->addWidget(rotation_y_label); // Добавление метки Y
    control_layout->addWidget(rotation_y_slider); // Добавление ползунка Y
    control_layout->addWidget(rotation_y_edit); // Добавление поля ввода Y
    control_layout->addWidget(rotation_z_label); // Добавление метки Z
    control_layout->addWidget(rotation_z_slider); // Добавление ползунка Z
    control_layout->addWidget(rotation_z_edit); // Добавление поля ввода Z
    control_layout->addWidget(morphing_label); // Добавление метки морфинга
    control_layout->addWidget(morphing_slider); // Добавление ползунка морфинга
    control_layout->addWidget(morphing_edit); // Добавление поля ввода морфинга
    control_layout->addWidget(split_label); // Добавление метки параметра разделения
    control_layout->addWidget(split_parameter_slider); // Добавление ползунка разделения
    control_layout->addWidget(split_parameter_edit); // Добавление поля ввода разделения
    control_layout->addStretch(); // Добавление растяжения для компактного размещения

    control_window->setWindowTitle("Панель управления"); // Установка заголовка окна управления
    control_window->resize(300, 420); // Установка начального размера окна управления

    // Показываем оба окна
    scene_window->show(); // Показ окна сцены
    control_window->show(); // Показ окна управления

    return a.exec(); // Запуск основного цикла приложения
}

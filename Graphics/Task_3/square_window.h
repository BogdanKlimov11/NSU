#pragma once

#include <QBasicTimer> // Для создания таймера, управляющего обновлением анимации
#include <QColor> // Для работы с цветами
#include <QColorDialog> // Для отображения диалогового окна выбора цвета
#include <QKeyEvent> // Для обработки событий клавиатуры
#include <QOpenGLBuffer> // Для работы с буферами OpenGL (вершинные и индексные)
#include <QOpenGLShaderProgram> // Для работы с шейдерными программами OpenGL
#include <QOpenGLWidget> // Базовый класс для виджетов, использующих OpenGL
#include <QVector3D> // Для работы с 3D-векторами
#include <QOpenGLFunctions> // Для доступа к функциям OpenGL
#include <memory> // Для использования умных указателей (std::unique_ptr)

// Структура для хранения данных вершины
struct VertexData {
    // Конструктор по умолчанию
    VertexData() {}
    // Конструктор, инициализирующий позицию и нормаль вершины
    VertexData(QVector3D p, QVector3D n) : position(p), normal(n) {}
    QVector3D position; // Позиция вершины в 3D-пространстве
    QVector3D normal; // Нормаль вершины для расчета освещения
};

// Объявление класса SquareWindow, наследующегося от QOpenGLWidget и QOpenGLFunctions
class SquareWindow : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT // Макрос Qt для поддержки сигналов и слотов

public:
    // Конструктор класса, принимающий родительский виджет
    explicit SquareWindow(QWidget *parent = nullptr);
    // Деструктор, определенный как виртуальный по умолчанию
    ~SquareWindow() override = default;

    // Переопределение метода для инициализации OpenGL (настройка шейдеров, буферов и т.д.)
    void initializeGL() override;
    // Переопределение метода для обработки изменения размера окна
    void resizeGL(int w, int h) override;
    // Переопределение метода для отрисовки сцены
    void paintGL() override;

    // Метод для изменения параметра морфинга объекта
    void change_morph_param(float value);
    // Переопределение метода для обработки событий нажатия клавиш
    void keyPressEvent(QKeyEvent *e) override;

    // Методы для изменения координат источника света по осям X, Y, Z
    void change_light_x_param(float value);
    void change_light_y_param(float value);
    void change_light_z_param(float value);

    // Метод для изменения цвета куба
    void change_cube_color();
    // Метод для изменения цвета источника света
    void change_light_color();
    // Метод для изменения параметра n (например, количества сегментов объекта)
    void changeN(int new_n);

public slots:
    // Слот для установки типа источника света (направленный, точечный, прожектор)
    void setLightType(int type);
    // Слот для установки интенсивности света
    void setLightIntensity(float intensity);
    // Слот для установки затухания света (для точечного источника)
    void setLightAttenuation(float attenuation);
    // Слот для установки угла обрезки света (для прожектора)
    void setLightCutoff(float cutoff);
    // Слот для установки направления света
    void setLightDirection(QVector3D direction);

    // Слоты для установки свойств материала
    void setMaterialAmbient(); // Фоновое освещение материала
    void setMaterialDiffuse(); // Диффузное освещение материала
    void setMaterialSpecular(); // Зеркальное освещение материала
    void setGlobalAmbient(); // Глобальное фоновое освещение сцены

private:
    // Переопределение метода для обработки нажатия мыши
    void mousePressEvent(QMouseEvent *e) override;
    // Переопределение метода для обработки отпускания мыши
    void mouseReleaseEvent(QMouseEvent *e) override;
    // Переопределение метода для обработки событий таймера (например, для анимации)
    void timerEvent(QTimerEvent *e) override;
    // Метод для инициализации геометрии куба с заданной шириной и количеством сегментов
    void init_cube(float width, int N);

    // Идентификаторы атрибутов шейдера для позиции и нормали
    GLint posAttr_ = 0; // Атрибут позиции вершины
    GLint normAttr_ = 0; // Атрибут нормали вершины

    // Умный указатель на шейдерную программу
    std::unique_ptr<QOpenGLShaderProgram> program_ = nullptr;

    // Буферы OpenGL для хранения вершин и индексов
    QOpenGLBuffer vertexBuffer; // Буфер для данных вершин
    QOpenGLBuffer indexBuffer{QOpenGLBuffer::IndexBuffer}; // Буфер для индексов

    // Параметры освещения
    int lightType = 0; // Тип источника света (0 - направленный, 1 - точечный, 2 - прожектор)
    float lightIntensity = 1.0f; // Интенсивность света
    float lightAttenuation = 0.1f; // Коэффициент затухания света
    QVector3D lightDirection{0.0f, 0.0f, -1.0f}; // Направление света
    float lightCutoff = 0.9f; // Угол обрезки для прожектора
    float light_x_param = 1.0f; // Позиция света по оси X
    float light_y_param = 1.0f; // Позиция света по оси Y
    float light_z_param = 1.0f; // Позиция света по оси Z
    QMatrix4x4 projection_matrix; // Матрица проекции для рендеринга
    QVector3D lightColor{1.0f, 1.0f, 1.0f}; // Цвет источника света (RGB)

    // Параметры объекта
    QVector3D cube_color{1.0f, 0.0f, 0.0f}; // Цвет куба (RGB)
    float morph_param = 1.0f; // Параметр морфинга (для изменения формы)
    int n = 2; // Количество сегментов объекта

    // Параметры материала
    QVector3D materialAmbient{1.0f, 0.0f, 0.0f}; // Фоновое освещение материала
    QVector3D materialDiffuse{1.0f, 0.0f, 0.0f}; // Диффузное освещение материала
    QVector3D materialSpecular{1.0f, 1.0f, 1.0f}; // Зеркальное освещение материала
    QVector3D globalAmbient{0.5f, 0.3f, 0.3f}; // Глобальное фоновое освещение

    // Переменные для управления анимацией и взаимодействием
    int frame_ = 0; // Текущий кадр анимации
    QVector2D mousePressPosition{0., 0.}; // Позиция мыши при нажатии
    QVector3D rotationAxis{0., 0., 1.}; // Ось вращения объекта
    QBasicTimer timer; // Таймер для периодического обновления сцены
};

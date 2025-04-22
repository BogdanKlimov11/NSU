#pragma once

#include <QBasicTimer> // Для создания таймера, управляющего периодическими обновлениями
#include <QMatrix4x4> // Для работы с 4x4 матрицами (трансформации)
#include <QOpenGLBuffer> // Для работы с буферами OpenGL (вершинные и индексные)
#include <QOpenGLFunctions> // Для доступа к функциям OpenGL
#include <QOpenGLShaderProgram> // Для работы с шейдерными программами OpenGL
#include <QOpenGLWidget> // Базовый класс для виджетов, использующих OpenGL
#include <QVector3D> // Для работы с 3D-векторами
#include <memory> // Для использования умных указателей (std::unique_ptr)

#include "material_earth.h" // Подключение заголовочного файла для класса MaterialEarth

// Структура для хранения данных вершины
struct VertexData {
    // Конструктор, инициализирующий все компоненты вершины
    VertexData(QVector3D p, QVector3D n, QVector2D t, QVector3D tng, QVector3D btg)
        : position(p), normal(n), textcoord(t), tangent(tng), bitangent(btg) {}
    QVector3D position; // Позиция вершины в 3D-пространстве
    QVector3D normal; // Нормаль вершины для освещения
    QVector2D textcoord; // Текстурные координаты (UV)
    QVector3D tangent; // Касательный вектор для карты нормалей
    QVector3D bitangent; // Бинормальный вектор для карты нормалей
};

// Определение класса SquareWindow, наследующегося от QOpenGLWidget и QOpenGLFunctions
class SquareWindow : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT // Макрос Qt для поддержки сигналов и слотов

public:
    // Конструктор класса, принимающий родительский виджет
    explicit SquareWindow(QWidget *parent = nullptr);
    // Деструктор, определенный как виртуальный по умолчанию
    ~SquareWindow() override = default;

    // Методы для установки скорости вращения сферы и движения света
    void setRotationSpeed(float speed) { rotationSpeed = speed; }
    void setLightSpeed(float speed) { lightSpeed = speed; }

protected:
    // Переопределение метода для инициализации OpenGL (настройка шейдеров, буферов и т.д.)
    void initializeGL() override;
    // Переопределение метода для обработки изменения размера окна
    void resizeGL(int w, int h) override;
    // Переопределение метода для отрисовки сцены
    void paintGL() override;

private:
    // Переопределение методов для обработки событий мыши
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    // Переопределение метода для обработки событий таймера (для анимации)
    void timerEvent(QTimerEvent *e) override;
    // Метод для инициализации геометрии сферы с заданным радиусом
    void init_sphere(float radius);

    // Идентификаторы атрибутов шейдера
    GLint posAttr_ = -1; // Атрибут позиции вершины
    GLint normAttr_ = -1; // Атрибут нормали вершины
    GLint textureAttr_ = -1; // Атрибут текстурных координат
    GLint tangentAttr_ = -1; // Атрибут касательного вектора
    GLint bitangentAttr_ = -1; // Атрибут бинормального вектора

    // Умный указатель на шейдерную программу
    std::unique_ptr<QOpenGLShaderProgram> program_;
    // Умный указатель на объект материала (текстуры Земли)
    std::unique_ptr<MaterialEarth> material_;

    // Буферы OpenGL для хранения вершин и индексов
    QOpenGLBuffer vertexBuffer; // Буфер для данных вершин
    QOpenGLBuffer indexBuffer{QOpenGLBuffer::IndexBuffer}; // Буфер для индексов

    // Матрица проекции для рендеринга
    QMatrix4x4 projection_matrix;

    // Переменные для управления анимацией и взаимодействием
    int frame_ = 0; // Текущий кадр анимации
    float rotationSpeed = 10.0f; // Скорость вращения сферы (градусы/кадр)
    float lightSpeed = 0.1f; // Скорость движения света (радианы/кадр)
    QVector2D mousePressPosition{0., 0.}; // Позиция мыши при нажатии
    QVector3D rotationAxis{0., 1., 0.}; // Ось вращения сферы (по умолчанию вдоль оси Y)
    QBasicTimer timer; // Таймер для периодического обновления сцены
};

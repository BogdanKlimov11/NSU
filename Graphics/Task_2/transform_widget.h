#ifndef TRANSFORM_WIDGET_H
#define TRANSFORM_WIDGET_H

#include <QOpenGLWidget> // Базовый класс для виджета OpenGL
#include <QOpenGLFunctions> // Функции OpenGL для упрощённого доступа
#include <QOpenGLShaderProgram> // Класс для работы с шейдерными программами
#include <QTimer> // Таймер для анимации или обновлений

class SceneWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT // Макрос Qt для поддержки сигналов и слотов

public:
    explicit SceneWidget(QWidget *parent = nullptr); // Конструктор с опциональным родительским виджетом
    ~SceneWidget() = default; // Деструктор по умолчанию

    void set_split_parameter(size_t parameter); // Метод для установки параметра разделения геометрии
    void set_morphing_parameter(float parameter); // Метод для установки параметра морфинга
    // Новые методы для установки углов вращения объекта
    void set_rotation_x(float angle); // Установка угла вращения вокруг оси X
    void set_rotation_y(float angle); // Установка угла вращения вокруг оси Y
    void set_rotation_z(float angle); // Установка угла вращения вокруг оси Z

protected:
    void initializeGL() override; // Инициализация OpenGL контекста
    void resizeGL(int w, int h) override; // Обработка изменения размеров виджета
    void paintGL() override; // Отрисовка сцены

private:
    void update_geometry(); // Обновление геометрических данных (вершины, цвета, индексы)

    QMatrix4x4 m_view_projection; // Матрица проекции и вида для рендеринга
    std::vector<QMatrix4x4> m_rotations; // Вектор матриц вращения для объектов

    std::vector<GLfloat> m_vertices; // Вектор координат вершин
    std::vector<GLfloat> m_colors; // Вектор цветов вершин
    std::vector<GLuint> m_indices; // Вектор индексов для построения примитивов

    // Углы вращения объекта по трём осям (заменяют единый угол)
    float m_rotation_x { 0 }; // Угол вращения вокруг оси X (инициализирован 0)
    float m_rotation_y { 0 }; // Угол вращения вокруг оси Y (инициализирован 0)
    float m_rotation_z { 0 }; // Угол вращения вокруг оси Z (инициализирован 0)

    GLint m_pos_attr { 0 }; // Атрибут положения вершин в шейдере
    GLint m_col_attr { 0 }; // Атрибут цвета вершин в шейдере
    GLint m_matrix_uniform { 0 }; // Униформ-переменная для матрицы в шейдере
    GLint m_morphing_uniform { 0 }; // Униформ-переменная для параметра морфинга в шейдере
    QOpenGLShaderProgram *m_shader_program { nullptr }; // Указатель на шейдерную программу

    float m_morphing_parameter { 0 }; // Параметр морфинга (инициализирован 0)
    size_t m_split_parameter { 10 }; // Параметр разделения геометрии (инициализирован 10)
};

#endif // TRANSFORM_WIDGET_H

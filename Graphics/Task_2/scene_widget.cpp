#include "scene_widget.h"

Scene_widget::Scene_widget(QWidget *parent) : QOpenGLWidget(parent) {
    update_geometry(); // Инициализация геометрии при создании объекта

    QMatrix4x4 view; // Матрица вида
    QMatrix4x4 projection; // Матрица проекции

    projection.perspective(60.0f, 1.0f, 0.1f, 100.0f); // Установка перспективной проекции: угол 60°, соотношение 1:1, ближняя плоскость 0.1, дальняя 100
    view.lookAt({0, 0, -2}, {0, 0, 0}, {1, 0, 0}); // Установка камеры: позиция (0,0,-2), смотрит в (0,0,0), верх вдоль оси X

    m_view_projection = projection * view; // Комбинированная матрица вида и проекции

    for (size_t i = 0; i < 6; ++i) { // Инициализация вектора для 6 матриц вращения (6 граней)
        m_rotations.push_back({}); // Добавление пустой матрицы
    }

    // Установка фиксированных вращений для каждой грани
    m_rotations[1].rotate(90, 1, 0, 0); // Поворот на 90° вокруг оси X
    m_rotations[2].rotate(180, 1, 0, 0); // Поворот на 180° вокруг оси X
    m_rotations[3].rotate(270, 1, 0, 0); // Поворот на 270° вокруг оси X
    m_rotations[4].rotate(90, 0, 1, 0); // Поворот на 90° вокруг оси Y
    m_rotations[5].rotate(270, 0, 1, 0); // Поворот на 270° вокруг оси Y
}

// Установка параметра разделения
void Scene_widget::set_split_parameter(size_t parameter) {
    m_split_parameter = parameter; // Обновление значения параметра
    update_geometry(); // Пересчёт геометрии с новым параметром
    update(); // Запрос перерисовки сцены
}

// Установка параметра морфинга
void Scene_widget::set_morphing_parameter(float parameter) {
    m_morphing_parameter = parameter; // Обновление значения морфинга
    update(); // Запрос перерисовки сцены
}

// Реализация методов для установки углов вращения
// Установка вращения вокруг оси X
void Scene_widget::set_rotation_x(float angle) {
    m_rotation_x = angle; // Обновление угла вращения по X
    update(); // Запрос перерисовки сцены
}

// Установка вращения вокруг оси Y
void Scene_widget::set_rotation_y(float angle) {
    m_rotation_y = angle; // Обновление угла вращения по Y
    update(); // Запрос перерисовки сцены
}

// Установка вращения вокруг оси Z
void Scene_widget::set_rotation_z(float angle) {
    m_rotation_z = angle; // Обновление угла вращения по Z
    update(); // Запрос перерисовки сцены
}

void Scene_widget::initializeGL() { // Инициализация OpenGL
    initializeOpenGLFunctions(); // Инициализация функций OpenGL
    glClearColor(0, 0, 0, 1); // Установка цвета очистки (чёрный фон)

    glEnable(GL_DEPTH_TEST); // Включение теста глубины
    glEnable(GL_CULL_FACE); // Включение отсечения граней
    glCullFace(GL_BACK); // Отсечение задних граней

    m_shader_program = new QOpenGLShaderProgram(this); // Создание объекта шейдерной программы
    m_shader_program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":vertex_shader.vsh"); // Добавление вершинного шейдера
    m_shader_program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":fragment_shader.fsh"); // Добавление фрагментного шейдера
    m_shader_program->link();       // Связывание шейдеров в программу
    m_pos_attr = m_shader_program->attributeLocation("pos_attr"); // Получение позиции атрибута вершин
    m_col_attr = m_shader_program->attributeLocation("col_attr"); // Получение позиции атрибута цвета
    m_matrix_uniform = m_shader_program->uniformLocation("mvp_matrix"); // Получение позиции униформ-переменной матрицы
    m_morphing_uniform = m_shader_program->uniformLocation("morphing_param"); // Получение позиции униформ-переменной морфинга
}

void Scene_widget::resizeGL(int w, int h) { // Обработка изменения размеров окна
    glViewport(0, 0, w, h);  // Установка области просмотра под новые размеры окна
}

void Scene_widget::paintGL() { // Отрисовка сцены
    const qreal retinaScale = devicePixelRatio(); // Получение масштаба для Retina-дисплеев
    glViewport(0, 0, width() * retinaScale, height() * retinaScale); // Установка области просмотра с учётом масштаба
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Очистка буферов цвета и глубины

    m_shader_program->bind(); // Привязка шейдерной программы

    GLuint EBO; // Переменная для буфера элементов
    glGenBuffers(1, &EBO); // Генерация одного буфера
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO); // Привязка буфера элементов
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(GLuint), m_indices.data(), GL_STATIC_DRAW); // Передача индексов в буфер

    glVertexAttribPointer(m_pos_attr, 3, GL_FLOAT, GL_FALSE, 0, m_vertices.data()); // Настройка атрибута положения вершин
    glVertexAttribPointer(m_col_attr, 3, GL_FLOAT, GL_FALSE, 0, m_colors.data()); // Настройка атрибута цвета вершин

    glEnableVertexAttribArray(m_pos_attr); // Включение атрибута положения
    glEnableVertexAttribArray(m_col_attr); // Включение атрибута цвета

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO); // Повторная привязка буфера элементов

    QMatrix4x4 rotation; // Матрица для хранения вращения объекта
    rotation.rotate(m_rotation_x, 1, 0, 0); // Применение вращения вокруг оси X
    rotation.rotate(m_rotation_y, 0, 1, 0); // Применение вращения вокруг оси Y
    rotation.rotate(m_rotation_z, 0, 0, 1); // Применение вращения вокруг оси Z

    for (const auto &model : m_rotations) { // Цикл по всем матрицам вращения (граням)
        m_shader_program->setUniformValue(m_matrix_uniform, m_view_projection * rotation * model); // Установка итоговой матрицы
        m_shader_program->setUniformValue(m_morphing_uniform, m_morphing_parameter); // Установка параметра морфинга
        glDrawElements(GL_LINES, m_indices.size(), GL_UNSIGNED_INT, 0); // Отрисовка линий по индексам
    }

    glDisableVertexAttribArray(m_col_attr); // Отключение атрибута цвета
    glDisableVertexAttribArray(m_pos_attr); // Отключение атрибута положения

    glDeleteBuffers(1, &EBO); // Удаление буфера элементов после отрисовки
    m_shader_program->release(); // Освобождение шейдерной программы
}

void Scene_widget::update_geometry() { // Обновление геометрических данных
    m_vertices.clear(); // Очистка вектора вершин
    m_colors.clear(); // Очистка вектора цветов
    m_indices.clear(); // Очистка вектора индексов

    // Генерация вершин в плоскости XY (Z = -0.5)
    for (size_t j = 0; j < m_split_parameter + 1; ++j) {
        for (size_t i = 0; i < m_split_parameter + 1; ++i) {
            m_vertices.push_back(-0.5f + GLfloat(i) / m_split_parameter); // Координата X
            m_vertices.push_back(-0.5f + GLfloat(j) / m_split_parameter); // Координата Y
            m_vertices.push_back(-0.5f); // Координата Z (фиксированная глубина)

            m_colors.push_back(1.0f); // Красный компонент цвета (белый)
            m_colors.push_back(1.0f); // Зелёный компонент цвета (белый)
            m_colors.push_back(1.0f); // Синий компонент цвета (белый)
        }
    }

    // Генерация индексов для построения линий сетки
    for (size_t j = 0; j < m_split_parameter; ++j) {
        for (size_t i = 0; i < m_split_parameter; ++i) {
            // Горизонтальные линии
            m_indices.push_back(j * (m_split_parameter + 1) + i); // Начало линии
            m_indices.push_back(j * (m_split_parameter + 1) + i + 1); // Конец линии

            // Вертикальные линии
            m_indices.push_back((j + 1) * (m_split_parameter + 1) + i); // Начало линии
            m_indices.push_back((j + 1) * (m_split_parameter + 1) + i + 1); // Конец линии

            // Диагонали (дополнительные линии)
            m_indices.push_back(j * (m_split_parameter + 1) + i); // Начало линии
            m_indices.push_back((j + 1) * (m_split_parameter + 1) + i); // Конец линии

            m_indices.push_back(j * (m_split_parameter + 1) + i + 1); // Начало линии
            m_indices.push_back((j + 1) * (m_split_parameter + 1) + i + 1); // Конец линии
        }
    }
}

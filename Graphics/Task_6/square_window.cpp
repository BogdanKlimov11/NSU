#include <QDebug> // Для вывода отладочной информации
#include <QMouseEvent> // Для обработки событий мыши
#include <cmath> // Для математических функций (sin, cos, M_PI)

#include "square_window.h" // Подключение заголовочного файла класса SquareWindow

// Конструктор класса SquareWindow
SquareWindow::SquareWindow(QWidget *parent) : QOpenGLWidget(parent) {}

// Инициализация OpenGL
void SquareWindow::initializeGL() {
    // Инициализация функций OpenGL для текущего контекста
    initializeOpenGLFunctions();

    // Создание шейдерной программы
    program_ = std::make_unique<QOpenGLShaderProgram>(this);
    // Загрузка и компиляция вершинного шейдера
    if (!program_->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vertex_shader.vsh")) {
        qDebug() << "Vertex shader error:" << program_->log();
    }
    // Загрузка и компиляция фрагментного шейдера
    if (!program_->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fragment_shader.fsh")) {
        qDebug() << "Fragment shader error:" << program_->log();
    }
    // Компоновка шейдерной программы
    if (!program_->link()) {
        qDebug() << "Shader program link error:" << program_->log();
    }

    // Получение местоположений атрибутов шейдера
    posAttr_ = program_->attributeLocation("posAttr");
    normAttr_ = program_->attributeLocation("normAttr");
    textureAttr_ = program_->attributeLocation("texAttr");
    tangentAttr_ = program_->attributeLocation("tangentAttr");
    bitangentAttr_ = program_->attributeLocation("bitangentAttr");

    // Вывод местоположений атрибутов для отладки
    qDebug() << "Attribute locations: posAttr=" << posAttr_ << ", normAttr=" << normAttr_
             << ", texAttr=" << textureAttr_ << ", tangentAttr=" << tangentAttr_
             << ", bitangentAttr=" << bitangentAttr_;

    // Создание объекта материала (текстуры Земли и карты нормалей)
    material_ = std::make_unique<MaterialEarth>();
    // Инициализация геометрии сферы с радиусом 1.5
    init_sphere(1.5f);

    // Установка цвета очистки буфера (черный, полностью непрозрачный)
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    // Включение теста глубины
    glEnable(GL_DEPTH_TEST);
    // Включение отбраковки задних граней
    glEnable(GL_CULL_FACE);

    // Установка начальной оси вращения (вдоль оси Y)
    rotationAxis = QVector3D(0.0f, 1.0f, 0.0f);

    // Запуск таймера с интервалом 30 мс для анимации
    timer.start(30, this);
}

// Отрисовка сцены
void SquareWindow::paintGL() {
    // Установка области просмотра с учетом масштаба пикселей
    glViewport(0, 0, width() * devicePixelRatio(), height() * devicePixelRatio());
    // Очистка буферов цвета и глубины
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Привязка шейдерной программы
    program_->bind();

    // Создание матриц вида и модели
    QMatrix4x4 view_matrix, model_matrix;
    view_matrix.setToIdentity();
    view_matrix.translate(0.0, 0.0, -5.0); // Перемещение камеры на 5 единиц назад
    model_matrix.setToIdentity();
    // Поворот сферы на основе текущего кадра и скорости вращения
    model_matrix.rotate(rotationSpeed * frame_ / 59.0, rotationAxis);

    // Вычисление позиции источника света (движение в плоскости XZ)
    float lightAngle = lightSpeed * frame_;
    float lightX = 5.0 * cos(lightAngle);
    float lightZ = 5.0 * sin(lightAngle);
    QVector3D lightPos = QVector3D(lightX, 0.0, lightZ);

    // Установка униформных переменных шейдера
    program_->setUniformValue("projection_matrix", projection_matrix); // Матрица проекции
    program_->setUniformValue("model", model_matrix); // Модельная матрица
    program_->setUniformValue("view_matrix", view_matrix); // Матрица вида
    program_->setUniformValue("norm_matrix", model_matrix.normalMatrix()); // Матрица нормалей
    program_->setUniformValue("viewPos", QVector3D(0.0, 0.0, 5.0)); // Позиция камеры
    program_->setUniformValue("lightPos", lightPos); // Позиция света

    // Привязка текстур материала
    material_->bind(program_.get());

    // Привязка вершинного буфера
    vertexBuffer.bind();
    float offset = 0;
    // Настройка атрибута позиции
    if (posAttr_ != -1) {
        program_->enableAttributeArray(posAttr_);
        program_->setAttributeBuffer(posAttr_, GL_FLOAT, offset, 3, sizeof(VertexData));
    }
    offset += sizeof(QVector3D);
    // Настройка атрибута нормали
    if (normAttr_ != -1) {
        program_->enableAttributeArray(normAttr_);
        program_->setAttributeBuffer(normAttr_, GL_FLOAT, offset, 3, sizeof(VertexData));
    }
    offset += sizeof(QVector3D);
    // Настройка атрибута текстурных координат
    if (textureAttr_ != -1) {
        program_->enableAttributeArray(textureAttr_);
        program_->setAttributeBuffer(textureAttr_, GL_FLOAT, offset, 2, sizeof(VertexData));
    }
    offset += sizeof(QVector2D);
    // Настройка атрибута касательного вектора
    if (tangentAttr_ != -1) {
        program_->enableAttributeArray(tangentAttr_);
        program_->setAttributeBuffer(tangentAttr_, GL_FLOAT, offset, 3, sizeof(VertexData));
    }
    offset += sizeof(QVector3D);
    // Настройка атрибута бинормального вектора
    if (bitangentAttr_ != -1) {
        program_->enableAttributeArray(bitangentAttr_);
        program_->setAttributeBuffer(bitangentAttr_, GL_FLOAT, offset, 3, sizeof(VertexData));
    }

    // Привязка индексного буфера и отрисовка
    indexBuffer.bind();
    glDrawElements(GL_TRIANGLES, indexBuffer.size(), GL_UNSIGNED_INT, nullptr);

    // Отключение атрибутов
    if (posAttr_ != -1) program_->disableAttributeArray(posAttr_);
    if (normAttr_ != -1) program_->disableAttributeArray(normAttr_);
    if (textureAttr_ != -1) program_->disableAttributeArray(textureAttr_);
    if (tangentAttr_ != -1) program_->disableAttributeArray(tangentAttr_);
    if (bitangentAttr_ != -1) program_->disableAttributeArray(bitangentAttr_);

    // Освобождение программы
    program_->release();

    // Проверка ошибок OpenGL
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        qDebug() << "OpenGL error:" << err;
    }

    // Увеличение счетчика кадров
    ++frame_;
}

// Обработка изменения размера окна
void SquareWindow::resizeGL(int w, int h) {
    // Сброс матрицы проекции
    projection_matrix.setToIdentity();
    // Установка перспективной проекции
    projection_matrix.perspective(60.0f, w / static_cast<float>(h), 0.1f, 100.0f);
}

// Обработка события нажатия мыши
void SquareWindow::mousePressEvent(QMouseEvent *e) {
    // Сохранение позиции мыши
    mousePressPosition = QVector2D(e->localPos());
    QWidget::mousePressEvent(e);
}

// Обработка события отпускания мыши
void SquareWindow::mouseReleaseEvent(QMouseEvent *e) {
    // Вычисление смещения мыши
    QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;
    // Установка оси вращения на основе смещения
    rotationAxis = QVector3D(diff.y(), diff.x(), 0.0).normalized();
    update();
}

// Обработка событий таймера
void SquareWindow::timerEvent(QTimerEvent *) {
    update(); // Запрос обновления сцены
}

// Инициализация геометрии сферы
void SquareWindow::init_sphere(float radius) {
    const int lats = 50, longs = 50; // Количество делений по широте и долготе
    const float latStep = 2 * M_PI / lats; // Шаг по широте
    const float longStep = M_PI / longs; // Шаг по долготе

    // Создание вектора вершин
    std::vector<VertexData> vertexes;
    for (unsigned i = 0; i <= longs; ++i) {
        float phi = i * longStep; // Угол по долготе
        float sinPhi = sin(phi);
        float cosPhi = cos(phi);

        for (unsigned j = 0; j <= lats; ++j) {
            float theta = j * latStep; // Угол по широте
            float sinTheta = sin(theta);
            float cosTheta = cos(theta);

            // Вычисление координат вершины
            float x = radius * sinPhi * cosTheta;
            float z = radius * sinPhi * sinTheta;
            float y = radius * cosPhi;

            // Позиция вершины
            QVector3D pos{x, y, z};
            // Нормаль (совпадает с позицией для сферы, нормализованная)
            QVector3D norm = pos.normalized();

            // Текстурные координаты (u, v)
            float u = (float)j / lats;
            float v = 1.0f - (float)i / longs;

            // Касательный вектор (производная по theta)
            QVector3D tangent = QVector3D(-sinTheta, 0.0f, cosTheta);
            // Бинормальный вектор (производная по phi, нормализованный)
            QVector3D bitangent = QVector3D(cosTheta * cosPhi, -sinPhi, sinTheta * cosPhi);
            bitangent = QVector3D::crossProduct(norm, tangent).normalized();

            // Добавление вершины
            vertexes.emplace_back(pos, norm, QVector2D(u, v), tangent, bitangent);
        }
    }

    // Создание вектора индексов
    std::vector<GLuint> indexes;
    for (unsigned i = 0; i < lats; ++i) {
        unsigned k1 = i * (longs + 1);
        unsigned k2 = k1 + longs + 1;

        for (unsigned j = 0; j < longs; ++j, ++k1, ++k2) {
            // Первый треугольник (для всех, кроме полюса)
            if (i != 0) {
                indexes.push_back(k1);
                indexes.push_back(k2);
                indexes.push_back(k1 + 1);
            }
            // Второй треугольник (для всех, кроме другого полюса)
            if (i != lats - 1) {
                indexes.push_back(k1 + 1);
                indexes.push_back(k2);
                indexes.push_back(k2 + 1);
            }
        }
    }

    // Создание и заполнение вершинного буфера
    vertexBuffer.create();
    vertexBuffer.bind();
    vertexBuffer.allocate(vertexes.data(), vertexes.size() * sizeof(VertexData));

    // Создание и заполнение индексного буфера
    indexBuffer.create();
    indexBuffer.bind();
    indexBuffer.allocate(indexes.data(), indexes.size() * sizeof(GLuint));

    // Вывод отладочной информации о количестве вершин и индексов
    qDebug() << "Sphere initialized: vertices=" << vertexes.size() << ", indices=" << indexes.size();
}

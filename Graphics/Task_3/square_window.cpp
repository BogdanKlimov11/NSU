#include <QColorDialog> // Для отображения диалогового окна выбора цвета
#include <QScreen> // Для получения информации об экране (например, для масштабирования)
#include <QtMath> // Для математических функций Qt

#include "square_window.h" // Подключение заголовочного файла класса SquareWindow

// Конструктор класса SquareWindow, наследующегося от QOpenGLWidget
SquareWindow::SquareWindow(QWidget *parent) : QOpenGLWidget(parent) {
    // Инициализация начального значения плотности сетки (n) для куба
    n = 10;
}

// Метод для инициализации OpenGL
void SquareWindow::initializeGL() {
    // Инициализация функций OpenGL для текущего контекста
    initializeOpenGLFunctions();
    // Создание уникального указателя на шейдерную программу
    program_ = std::make_unique<QOpenGLShaderProgram>(this);
    // Загрузка и добавление фрагментного шейдера из файла ресурсов
    program_->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fragment_shader.fsh");
    // Загрузка и добавление вершинного шейдера из файла ресурсов
    program_->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vertex_shader.vsh");
    // Компоновка шейдерной программы
    program_->link();

    // Получение местоположения атрибута позиции вершины в шейдере
    posAttr_ = program_->attributeLocation("posAttr");
    // Получение местоположения атрибута нормали вершины в шейдере
    normAttr_ = program_->attributeLocation("normAttr");
    // Проверка, что атрибут позиции найден
    Q_ASSERT(posAttr_ != -1);

    // Включение теста глубины для корректного отображения объектов
    glEnable(GL_DEPTH_TEST);
    // Включение отбраковки граней (рендеринг только видимых граней)
    glEnable(GL_CULL_FACE);
    // Запуск таймера с интервалом 30 мс для периодического обновления сцены
    timer.start(30, this);

    // Инициализация геометрии куба с шириной 1.5 и плотностью сетки n
    init_cube(1.5f, n);
}

// Метод для отрисовки сцены
void SquareWindow::paintGL() {
    // Получение масштаба пикселей для поддержки дисплеев с высоким разрешением (Retina)
    const auto retinaScale = devicePixelRatio();
    // Установка области просмотра OpenGL с учетом масштаба
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    // Очистка буферов цвета и глубины
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Привязка шейдерной программы для использования
    program_->bind();

    // Установка униформных переменных шейдера
    program_->setUniformValue("projection_matrix", projection_matrix); // Матрица проекции
    program_->setUniformValue("viewPos", QVector3D(0.0, 0.0, 0)); // Позиция камеры
    program_->setUniformValue("lightPos", QVector3D(light_x_param, light_y_param, light_z_param)); // Позиция источника света
    program_->setUniformValue("objectColor", cube_color); // Цвет объекта (куба)
    program_->setUniformValue("lightColor", lightColor); // Цвет источника света
    program_->setUniformValue("morph_param", morph_param); // Параметр морфинга
    program_->setUniformValue("lightType", lightType); // Тип источника света
    program_->setUniformValue("lightIntensity", lightIntensity); // Интенсивность света
    program_->setUniformValue("lightAttenuation", lightAttenuation); // Затухание света
    program_->setUniformValue("lightDirection", lightDirection); // Направление света
    program_->setUniformValue("lightCutoff", lightCutoff); // Угол обрезки света

    // Установка свойств материала
    program_->setUniformValue("materialAmbient", materialAmbient); // Фоновое освещение материала
    program_->setUniformValue("materialDiffuse", materialDiffuse); // Диффузное освещение материала
    program_->setUniformValue("materialSpecular", materialSpecular); // Зеркальное освещение материала
    program_->setUniformValue("globalAmbient", globalAmbient); // Глобальное фоновое освещение

    // Привязка вершинного буфера
    vertexBuffer.bind();
    // Привязка индексного буфера
    indexBuffer.bind();
    // Установка атрибута позиции вершины (3 компоненты типа float, начиная с начала структуры VertexData)
    program_->setAttributeBuffer(posAttr_, GL_FLOAT, 0, 3, sizeof(VertexData));
    // Включение массива атрибутов позиции
    program_->enableAttributeArray(posAttr_);
    // Установка атрибута нормали вершины (3 компоненты типа float, начиная после позиции в VertexData)
    program_->setAttributeBuffer(normAttr_, GL_FLOAT, sizeof(QVector3D), 3, sizeof(VertexData));
    // Включение массива атрибутов нормали
    program_->enableAttributeArray(normAttr_);

    // Цикл для рендеринга 9 кубов в сетке 3x3
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; ++j) {
            // Создание матрицы вида для текущего куба
            QMatrix4x4 view_matrix;
            view_matrix.setToIdentity();
            // Перемещение куба в соответствующую позицию в сетке
            view_matrix.translate(-2.5 + i * 2.5, -2.5 + j * 2.5, -8);
            // Поворот куба на основе текущего кадра анимации
            view_matrix.rotate(100.0 * frame_ / 59.0, rotationAxis);

            // Установка матриц в шейдер
            program_->setUniformValue("model", view_matrix); // Модельная матрица
            program_->setUniformValue("view_matrix", view_matrix); // Матрица вида
            program_->setUniformValue("norm_matrix", view_matrix.normalMatrix()); // Нормальная матрица

            // Отрисовка куба с использованием индексов (треугольники)
            glDrawElements(GL_TRIANGLES, indexBuffer.size(), GL_UNSIGNED_INT, nullptr);
        }
    }

    // Отключение массива атрибутов позиции
    program_->disableAttributeArray(posAttr_);
    // Отключение вершинного буфера
    vertexBuffer.release();
    // Отключение индексного буфера
    indexBuffer.release();
    // Освобождение шейдерной программы
    program_->release();
    // Увеличение счетчика кадров для анимации
    ++frame_;
}

// Метод для обработки изменения размера окна
void SquareWindow::resizeGL(const int w, const int h) {
    // Вычисление соотношения сторон окна
    const auto aspect = w / static_cast<double>(h);
    // Сброс матрицы проекции
    projection_matrix.setToIdentity();
    // Установка перспективной проекции (угол обзора 60°, соотношение сторон, ближняя и дальняя плоскости отсечения)
    projection_matrix.perspective(60.0f, aspect, 0.01f, 100.0f);
}

// Метод для изменения параметра морфинга
void SquareWindow::change_morph_param(float value) {
    // Нормализация значения (диапазон ползунка 0..1000 преобразуется в 0..1)
    morph_param = value / 1000;
    // Запрос обновления сцены
    update();
}

// Метод для изменения параметра положения света по оси X
void SquareWindow::change_light_x_param(float value) {
    if (lightType == 0) {
        // Для направленного света обновляется направление
        lightDirection.setX(value);
    } else {
        // Для точечного или прожекторного света обновляется позиция
        light_x_param = value;
    }
    // Запрос обновления сцены
    update();
}

// Метод для изменения параметра положения света по оси Y
void SquareWindow::change_light_y_param(float value) {
    if (lightType == 0) {
        // Для направленного света обновляется направление
        lightDirection.setY(value);
    } else {
        // Для точечного или прожекторного света обновляется позиция
        light_y_param = value;
    }
    // Запрос обновления сцены
    update();
}

// Метод для изменения параметра положения света по оси Z
void SquareWindow::change_light_z_param(float value) {
    if (lightType == 0) {
        // Для направленного света обновляется направление
        lightDirection.setZ(value);
    } else {
        // Для точечного или прожекторного света обновляется позиция
        light_z_param = value;
    }
    // Запрос обновления сцены
    update();
}

// Метод для обработки события нажатия мыши
void SquareWindow::mousePressEvent(QMouseEvent *e) {
    // Сохранение позиции мыши при нажатии
    mousePressPosition = QVector2D(e->position());
}

// Метод для обработки события отпускания мыши
void SquareWindow::mouseReleaseEvent(QMouseEvent *e) {
    // Вычисление смещения мыши от точки нажатия до точки отпускания
    const auto diff = QVector2D(e->position()) - mousePressPosition;
    // Установка оси вращения на основе смещения мыши (нормализованный вектор)
    rotationAxis = QVector3D(diff.y(), diff.x(), 0.0).normalized();
}

// Метод для обработки событий нажатия клавиш
void SquareWindow::keyPressEvent(QKeyEvent *e) {
    if (e->key() == Qt::Key::Key_Plus) {
        // При нажатии клавиши "+" открывается диалог выбора цвета
        const auto chosen_color = QColorDialog::getColor();
        // Установка цвета куба в нормализованном виде (0..1)
        cube_color = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    } else if (e->key() == Qt::Key::Key_Minus) {
        // При нажатии клавиши "-" открывается диалог выбора цвета
        const auto chosen_color = QColorDialog::getColor();
        // Установка цвета света в нормализованном виде (0..1)
        lightColor = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    }
}

// Метод для обработки событий таймера
void SquareWindow::timerEvent(QTimerEvent * /*event*/) {
    // Запрос обновления сцены для анимации
    update();
}

// Метод для инициализации геометрии куба
void SquareWindow::init_cube(const float width, const int N) {
    // Вычисление половины ширины куба
    auto width_div_2 = width / 2.0f;
    // Вычисление шага сетки для вершин
    auto step = width / float(N - 1);

    // Создание вектора для хранения данных вершин
    std::vector<VertexData> vertexes;
    // Резервирование памяти для вершин (6 граней, каждая с N^2 вершинами)
    vertexes.reserve(6 * pow(N, 2));
    // Создание вершин для передней и задней граней (z = ±width_div_2)
    for (auto z = -width_div_2; z <= width_div_2; z += width) {
        for (auto j = 0; j < N; j++) {
            for (auto i = 0; i < N; i++) {
                // Добавление вершины с позицией и нормалью
                vertexes.emplace_back(
                    VertexData(
                        QVector3D(-z + i * step * z / width_div_2, -width_div_2 + j * step, z), // Позиция
                        QVector3D(0.0, 0.0, z / width_div_2) // Нормаль
                        )
                    );
            }
        }
    }
    // Создание вершин для левой и правой граней (x = ±width_div_2)
    for (auto x = -width_div_2; x <= width_div_2; x += width) {
        for (auto k = 0; k < N; k++) {
            for (auto j = 0; j < N; j++) {
                // Добавление вершины с позицией и нормалью
                vertexes.emplace_back(
                    VertexData(
                        QVector3D(x, -width_div_2 + j * step, -x + x * k * step / width_div_2), // Позиция
                        QVector3D(x / width_div_2, 0.0, 0.0) // Нормаль
                        )
                    );
            }
        }
    }
    // Создание вершин для верхней и нижней граней (y = ±width_div_2)
    for (auto y = -width_div_2; y <= width_div_2; y += width) {
        for (auto i = 0; i < N; i++) {
            for (auto k = 0; k < N; k++) {
                // Добавление вершины с позицией и нормалью
                vertexes.emplace_back(
                    VertexData(
                        QVector3D(-width_div_2 + i * step, y, -y + y * k * step / width_div_2), // Позиция
                        QVector3D(0.0, y / width_div_2, 0.0) // Нормаль
                        )
                    );
            }
        }
    }

    // Создание вектора для хранения индексов треугольников
    std::vector<GLuint> indexes;
    // Вычисление общего количества индексов (36 индексов на грань, (N-1)^2 квадратов на грань)
    int vertex_count = 36 * pow(N - 1, 2);
    // Резервирование памяти для индексов
    indexes.reserve(vertex_count);
    // Создание индексов для треугольников каждой грани
    for (int i = 0; i < 6 * N * N; i += N * N) {
        for (int j = 0; j < (N - 1) * (N - 1); j += N) {
            for (int k = 0; k < (N - 1); k++) {
                // Добавление индексов для двух треугольников, формирующих квадрат
                // Первый треугольник
                indexes.emplace_back(i + j + k + N);
                indexes.emplace_back(i + j + k + 0);
                indexes.emplace_back(i + j + k + N + 1);
                // Второй треугольник
                indexes.emplace_back(i + j + k + N + 1);
                indexes.emplace_back(i + j + k + 0);
                indexes.emplace_back(i + j + k + 1);
            }
        }
    }

    // Создание и привязка вершинного буфера
    vertexBuffer.create();
    vertexBuffer.bind();
    // Выделение памяти и копирование данных вершин в буфер
    vertexBuffer.allocate(vertexes.data(), static_cast<int>(vertexes.size() * sizeof(VertexData)));

    // Создание и привязка индексного буфера
    indexBuffer.create();
    indexBuffer.bind();
    // Выделение памяти и копирование индексов в буфер
    indexBuffer.allocate(indexes.data(), static_cast<int>(indexes.size() * sizeof(GLuint)));
}

// Метод для изменения цвета куба
void SquareWindow::change_cube_color() {
    // Открытие диалога выбора цвета
    const auto chosen_color = QColorDialog::getColor();
    // Установка цвета куба в нормализованном виде (0..1)
    cube_color = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    // Запрос обновления сцены
    update();
}

// Метод для изменения цвета источника света
void SquareWindow::change_light_color() {
    // Открытие диалога выбора цвета
    const auto chosen_color = QColorDialog::getColor();
    // Установка цвета света в нормализованном виде (0..1)
    lightColor = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    // Запрос обновления сцены
    update();
}

// Метод для изменения плотности сетки куба
void SquareWindow::changeN(int new_n) {
    // Установка нового значения плотности сетки
    n = new_n;
    // Повторная инициализация геометрии куба с новым значением n
    init_cube(1.5f, n);
    // Запрос обновления сцены
    update();
}

// Метод для установки типа источника света
void SquareWindow::setLightType(int type) {
    // Установка нового типа света
    lightType = type;
    if (lightType == 0) {
        // Настройки для направленного света
        lightIntensity = 1.0f; // Интенсивность по умолчанию
        lightAttenuation = 0.0f; // Затухание отключено
        lightCutoff = 0.0f; // Угол обрезки не используется
        lightDirection = QVector3D(1.0f, 1.0f, 1.0f).normalized(); // Нормализованное направление
        light_x_param = lightDirection.x(); // Синхронизация параметров
        light_y_param = lightDirection.y();
        light_z_param = lightDirection.z();
        lightColor = QVector3D(1.0f, 1.0f, 0.8f); // Цвет направленного света
    } else if (lightType == 1) {
        // Настройки для точечного света
        lightCutoff = 0.0f; // Угол обрезки не используется
        lightDirection = QVector3D(0.0f, 0.0f, 0.0f); // Направление не используется
        // Позиция по умолчанию
        light_x_param = 1.0f;
        light_y_param = 1.0f;
        light_z_param = 1.0f;
        lightColor = QVector3D(0.6f, 0.8f, 1.0f); // Цвет точечного света
    } else if (lightType == 2) {
        // Настройки для прожекторного света
        lightDirection = QVector3D(1.0f, 1.0f, 1.0f).normalized(); // Нормализованное направление
        // Позиция по умолчанию
        light_x_param = 1.0f;
        light_y_param = 1.0f;
        light_z_param = 1.0f;
        lightColor = QVector3D(0.6f, 0.8f, 1.0f); // Цвет прожекторного света
    }
    // Запрос обновления сцены
    update();
}

// Метод для установки интенсивности света
void SquareWindow::setLightIntensity(float intensity) {
    // Установка нового значения интенсивности
    lightIntensity = intensity;
    // Запрос обновления сцены
    update();
}

// Метод для установки затухания света
void SquareWindow::setLightAttenuation(float attenuation) {
    // Нормализация значения затухания (диапазон ползунка 0..100 преобразуется в 0..1)
    lightAttenuation = attenuation / 100.0f;
    // Запрос обновления сцены
    update();
}

// Метод для установки угла обрезки света
void SquareWindow::setLightCutoff(float cutoff) {
    // Установка нового значения угла обрезки
    lightCutoff = cutoff;
    // Запрос обновления сцены
    update();
}

// Метод для установки направления света
void SquareWindow::setLightDirection(QVector3D direction) {
    // Установка нового направления света
    lightDirection = direction;
    // Запрос обновления сцены
    update();
}

// Метод для установки фонового освещения материала
void SquareWindow::setMaterialAmbient() {
    // Открытие диалога выбора цвета
    const auto chosen_color = QColorDialog::getColor();
    // Установка фонового освещения материала в нормализованном виде (0..1)
    materialAmbient = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    // Запрос обновления сцены
    update();
}

// Метод для установки диффузного освещения материала
void SquareWindow::setMaterialDiffuse() {
    // Открытие диалога выбора цвета
    const auto chosen_color = QColorDialog::getColor();
    // Установка диффузного освещения материала в нормализованном виде (0..1)
    materialDiffuse = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    // Запрос обновления сцены
    update();
}

// Метод для установки зеркального освещения материала
void SquareWindow::setMaterialSpecular() {
    // Открытие диалога выбора цвета
    const auto chosen_color = QColorDialog::getColor();
    // Установка зеркального освещения материала в нормализованном виде (0..1)
    materialSpecular = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    // Запрос обновления сцены
    update();
}

// Метод для установки глобального фонового освещения
void SquareWindow::setGlobalAmbient() {
    // Открытие диалога выбора цвета
    const auto chosen_color = QColorDialog::getColor();
    // Установка глобального фонового освещения в нормализованном виде (0..1)
    globalAmbient = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    // Запрос обновления сцены
    update();
}

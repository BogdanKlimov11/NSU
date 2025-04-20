#include <QColorDialog> // Для диалогового окна выбора цвета
#include <QScreen> // Для получения информации об экране (например, для масштабирования)
#include <QtMath> // Для математических функций Qt
#include <QDebug> // Для вывода отладочной информации

#include "square_window.h" // Подключение заголовочного файла класса SquareWindow

// Конструктор класса SquareWindow
SquareWindow::SquareWindow(QWidget *parent) : QOpenGLWidget(parent) {
    n = 10; // Инициализация начального значения плотности сетки (n)
    fpsTimer.start(); // Запуск таймера для расчета FPS
}

// Инициализация OpenGL
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

    // Получение местоположений атрибутов шейдера
    posAttr_ = program_->attributeLocation("posAttr");
    normAttr_ = program_->attributeLocation("normAttr");
    // Проверка, что атрибут позиции найден
    Q_ASSERT(posAttr_ != -1);

    // Включение теста глубины для корректного отображения объектов
    glEnable(GL_DEPTH_TEST);
    // Включение отбраковки задних граней
    glEnable(GL_CULL_FACE);
    // Запуск таймера с интервалом 30 мс для анимации
    timer.start(30, this);

    // Инициализация геометрии куба с шириной 1.5 и плотностью сетки n
    init_cube(1.5f, n);
}

// Отрисовка сцены
void SquareWindow::paintGL() {
    // Получение масштаба пикселей для поддержки дисплеев с высоким разрешением
    const auto retinaScale = devicePixelRatio();
    // Установка области просмотра OpenGL с учетом масштаба
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    // Очистка буферов цвета и глубины
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Проверка, что шейдерная программа существует и скомпонована
    if (!program_ || !program_->isLinked()) {
        qDebug() << "Shader program is not linked, skipping rendering.";
        return;
    }

    // Отладочная информация о текущем кадре и параметрах освещения
    qDebug() << "Rendering frame:" << frame_;
    qDebug() << "Light type:" << lightType << ", Light direction:" << lightDirection
             << ", Light position:" << QVector3D(light_x_param, light_y_param, light_z_param);

    // Привязка шейдерной программы
    program_->bind();

    // Создание матрицы вида (единичная матрица)
    QMatrix4x4 view_matrix;
    view_matrix.setToIdentity();

    // Установка униформных переменных шейдера
    program_->setUniformValue("projection_matrix", projection_matrix); // Матрица проекции
    program_->setUniformValue("viewPos", QVector3D(0.0f, 0.0f, 0.0f)); // Позиция камеры
    program_->setUniformValue("lightPos", QVector3D(light_x_param, light_y_param, light_z_param)); // Позиция света
    program_->setUniformValue("objectColor", cube_color); // Цвет объекта
    program_->setUniformValue("lightColor", lightColor); // Цвет света
    program_->setUniformValue("morph_param", morph_param); // Параметр морфинга
    program_->setUniformValue("lightType", lightType); // Тип света
    program_->setUniformValue("lightIntensity", lightIntensity); // Интенсивность света
    program_->setUniformValue("lightAttenuation", lightAttenuation); // Затухание света
    program_->setUniformValue("lightDirection", lightDirection); // Направление света
    program_->setUniformValue("lightCutoff", lightCutoff); // Угол обрезки света

    // Привязка вершинного и индексного буферов
    vertexBuffer.bind();
    indexBuffer.bind();
    // Установка атрибута позиции вершины
    program_->setAttributeBuffer(posAttr_, GL_FLOAT, 0, 3, sizeof(VertexData));
    program_->enableAttributeArray(posAttr_);
    // Установка атрибута нормали вершины
    program_->setAttributeBuffer(normAttr_, GL_FLOAT, sizeof(QVector3D), 3, sizeof(VertexData));
    program_->enableAttributeArray(normAttr_);

    // Отрисовка сетки кубов 3x3
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; ++j) {
            // Создание модельной матрицы для текущего куба
            QMatrix4x4 model_matrix;
            model_matrix.setToIdentity();
            // Перемещение куба в соответствующую позицию в сетке
            model_matrix.translate(-2.5 + i * 2.5, -2.5 + j * 2.5, -8.0f);
            // Поворот куба на основе текущего кадра анимации
            model_matrix.rotate(100.0 * frame_ / 59.0, rotationAxis);

            // Установка матриц в шейдер
            program_->setUniformValue("model", model_matrix);
            program_->setUniformValue("view_matrix", view_matrix);
            // Вычисление матрицы нормалей
            QMatrix4x4 modelViewMatrix = view_matrix * model_matrix;
            program_->setUniformValue("norm_matrix", modelViewMatrix.normalMatrix());

            // Отрисовка куба с использованием индексов
            glDrawElements(GL_TRIANGLES, indexBuffer.size() / sizeof(GLuint), GL_UNSIGNED_INT, nullptr);
            qDebug() << "Drawing" << (indexBuffer.size() / sizeof(GLuint)) << "indices";
        }
    }

    // Отключение атрибутов и освобождение буферов
    program_->disableAttributeArray(posAttr_);
    program_->disableAttributeArray(normAttr_);
    vertexBuffer.release();
    indexBuffer.release();
    program_->release();

    // Увеличение счетчика кадров
    ++frame_;

    // Расчет FPS
    frameCount++;
    qint64 elapsed = fpsTimer.elapsed();
    qDebug() << "Elapsed time:" << elapsed << "ms, Frame count:" << frameCount;
    if (elapsed >= 1000) {
        currentFps = frameCount * 1000 / elapsed;
        qDebug() << "FPS calculated:" << currentFps;
        emit fpsUpdated(currentFps); // Вызов сигнала для обновления FPS
        frameCount = 0;
        fpsTimer.restart();
    }
}

// Обработка изменения размера окна
void SquareWindow::resizeGL(const int w, const int h) {
    // Вычисление соотношения сторон
    const auto aspect = w / static_cast<double>(h);
    // Сброс матрицы проекции
    projection_matrix.setToIdentity();
    // Установка перспективной проекции
    projection_matrix.perspective(60.0f, aspect, 0.01f, 100.0f);
}

// Изменение параметра морфинга
void SquareWindow::change_morph_param(float value) {
    morph_param = value / 1000; // Нормализация значения (0..1000 -> 0..1)
    update(); // Запрос обновления сцены
}

// Изменение координаты X источника света
void SquareWindow::change_light_x_param(float value) {
    if (lightType == 0) {
        lightDirection.setX(value); // Для направленного света
    } else {
        light_x_param = value; // Для точечного или прожекторного света
    }
    update();
}

// Изменение координаты Y источника света
void SquareWindow::change_light_y_param(float value) {
    if (lightType == 0) {
        lightDirection.setY(value);
    } else {
        light_y_param = value;
    }
    update();
}

// Изменение координаты Z источника света
void SquareWindow::change_light_z_param(float value) {
    if (lightType == 0) {
        lightDirection.setZ(value);
    } else {
        light_z_param = value;
    }
    update();
}

// Обработка события нажатия мыши
void SquareWindow::mousePressEvent(QMouseEvent *e) {
    // Сохранение позиции мыши при нажатии
    mousePressPosition = QVector2D(e->position());
}

// Обработка события отпускания мыши
void SquareWindow::mouseReleaseEvent(QMouseEvent *e) {
    // Вычисление смещения мыши
    const auto diff = QVector2D(e->position()) - mousePressPosition;
    // Установка оси вращения на основе смещения
    rotationAxis = QVector3D(diff.y(), diff.x(), 0.0).normalized();
}

// Обработка событий нажатия клавиш
void SquareWindow::keyPressEvent(QKeyEvent *e) {
    if (e->key() == Qt::Key::Key_Plus) {
        // Открытие диалога выбора цвета для куба
        const auto chosen_color = QColorDialog::getColor();
        cube_color = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    } else if (e->key() == Qt::Key::Key_Minus) {
        // Открытие диалога выбора цвета для света
        const auto chosen_color = QColorDialog::getColor();
        lightColor = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    }
    update();
}

// Обработка событий таймера
void SquareWindow::timerEvent(QTimerEvent *e) {
    Q_UNUSED(e);
    qDebug() << "Timer event triggered, requesting update.";
    update(); // Запрос обновления сцены
}

// Инициализация геометрии куба
void SquareWindow::init_cube(const float width, const int N) {
    auto width_div_2 = width / 2.0f; // Половина ширины куба
    auto step = width / float(N - 1); // Шаг сетки

    // Создание вектора вершин
    std::vector<VertexData> vertexes;
    vertexes.reserve(6 * pow(N, 2)); // Резервирование памяти для 6 граней
    // Передняя и задняя грани (z = ±width_div_2)
    for (auto z = -width_div_2; z <= width_div_2; z += width) {
        for (auto j = 0; j < N; j++) {
            for (auto i = 0; i < N; i++) {
                vertexes.emplace_back(
                    VertexData(
                        QVector3D(-z + i * step * z / width_div_2, -width_div_2 + j * step, z),
                        QVector3D(0.0, 0.0, z / width_div_2)
                        )
                    );
            }
        }
    }
    // Левая и правая грани (x = ±width_div_2)
    for (auto x = -width_div_2; x <= width_div_2; x += width) {
        for (auto k = 0; k < N; k++) {
            for (auto j = 0; j < N; j++) {
                vertexes.emplace_back(
                    VertexData(
                        QVector3D(x, -width_div_2 + j * step, -x + x * k * step / width_div_2),
                        QVector3D(x / width_div_2, 0.0, 0.0)
                        )
                    );
            }
        }
    }
    // Верхняя и нижняя грани (y = ±width_div_2)
    for (auto y = -width_div_2; y <= width_div_2; y += width) {
        for (auto i = 0; i < N; i++) {
            for (auto k = 0; k < N; k++) {
                vertexes.emplace_back(
                    VertexData(
                        QVector3D(-width_div_2 + i * step, y, -y + y * k * step / width_div_2),
                        QVector3D(0.0, y / width_div_2, 0.0)
                        )
                    );
            }
        }
    }

    // Создание вектора индексов
    std::vector<GLuint> indexes;
    int vertex_count = 36 * pow(N - 1, 2); // Количество индексов
    indexes.reserve(vertex_count);
    for (int i = 0; i < 6 * N * N; i += N * N) {
        for (int j = 0; j < (N - 1) * (N - 1); j += N) {
            for (int k = 0; k < (N - 1); k++) {
                // Индексы для двух треугольников, формирующих квадрат
                indexes.emplace_back(i + j + k + N);
                indexes.emplace_back(i + j + k + 0);
                indexes.emplace_back(i + j + k + N + 1);
                indexes.emplace_back(i + j + k + N + 1);
                indexes.emplace_back(i + j + k + 0);
                indexes.emplace_back(i + j + k + 1);
            }
        }
    }

    // Создание и заполнение вершинного буфера
    vertexBuffer.create();
    vertexBuffer.bind();
    vertexBuffer.allocate(vertexes.data(), static_cast<int>(vertexes.size() * sizeof(VertexData)));

    // Создание и заполнение индексного буфера
    indexBuffer.create();
    indexBuffer.bind();
    indexBuffer.allocate(indexes.data(), static_cast<int>(indexes.size() * sizeof(GLuint)));
}

// Изменение цвета куба
void SquareWindow::change_cube_color() {
    const auto chosen_color = QColorDialog::getColor();
    cube_color = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    update();
}

// Изменение цвета света
void SquareWindow::change_light_color() {
    const auto chosen_color = QColorDialog::getColor();
    lightColor = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    update();
}

// Изменение плотности сетки
void SquareWindow::changeN(int new_n) {
    n = new_n;
    init_cube(1.5f, n); // Повторная инициализация куба
    update();
}

// Установка типа источника света
void SquareWindow::setLightType(int type) {
    lightType = type;

    // Настройка параметров в зависимости от типа света
    if (lightType == 0) { // Направленный свет
        lightIntensity = 1.0f;
        lightAttenuation = 0.0f;
        lightCutoff = 0.0f;
        lightDirection = QVector3D(1.0f, 1.0f, 1.0f).normalized();
        light_x_param = lightDirection.x();
        light_y_param = lightDirection.y();
        light_z_param = lightDirection.z();
    } else if (lightType == 1) { // Точечный свет
        lightCutoff = 0.0f;
        lightDirection = QVector3D(0.0f, 0.0f, 0.0f);
        light_x_param = 1.0f;
        light_y_param = 1.0f;
        light_z_param = 1.0f;
    } else if (lightType == 2) { // Прожектор
        lightDirection = QVector3D(1.0f, 1.0f, 1.0f).normalized();
        light_x_param = 1.0f;
        light_y_param = 1.0f;
        light_z_param = 1.0f;
    }
    update();
}

// Установка интенсивности света
void SquareWindow::setLightIntensity(float intensity) {
    lightIntensity = intensity;
    update();
}

// Установка затухания света
void SquareWindow::setLightAttenuation(float attenuation) {
    lightAttenuation = attenuation / 100.0f; // Нормализация значения
    update();
}

// Установка угла обрезки света
void SquareWindow::setLightCutoff(float cutoff) {
    lightCutoff = cutoff / 100.0f; // Нормализация значения
    update();
}

// Установка направления света
void SquareWindow::setLightDirection(QVector3D direction) {
    lightDirection = direction;
    update();
}

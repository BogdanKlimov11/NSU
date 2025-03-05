// Подключение необходимых заголовков Qt для работы с приложением, матрицами, шейдерами и событиями
#include <QGuiApplication>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QScreen>
#include <QtMath>
#include <QKeyEvent>

// Подключение заголовочного файла базового класса OpenGLWindow
#include "square_widget.h"

// Определение класса TriangleWindow, который наследуется от OpenGLWindow
class TriangleWindow : public OpenGLWindow {
public:
    // Использование конструктора базового класса
    using OpenGLWindow::OpenGLWindow;

    // Переопределение метода инициализации OpenGL
    void initialize() override;
    // Переопределение метода отрисовки
    void render() override;
    // Метод для установки оси вращения объекта
    void setRotationAxis(QVector3D axis);
    // Переопределение метода обработки нажатий клавиш
    void keyPressEvent(QKeyEvent *event) override;
    // Метод для настройки параметров окна
    void setupWindow();

private:
    // Переменные для хранения местоположений атрибутов и униформ в шейдерах
    GLint m_posAttr = 0; // Атрибут позиции вершин
    GLint m_colAttr = 0; // Атрибут цвета вершин
    GLint m_matrixUniform = 0; // Униформ для матрицы трансформации

    // Указатель на программу шейдеров
    QOpenGLShaderProgram *m_program = nullptr;
    // Счетчик кадров для анимации
    int m_frame = 0;
    // Ось вращения по умолчанию (диагональная)
    QVector3D m_rotationAxis = {1.0f, 1.0f, 0.0f};
    // Флаг полноэкранного режима
    bool m_fullscreen = false;
};

// Точка входа в приложение
int main(int argc, char **argv) {
    // Создание объекта приложения Qt
    QGuiApplication app(argc, argv);

    // Настройка формата поверхности с поддержкой сглаживания (16 выборок)
    QSurfaceFormat format;
    format.setSamples(16);

    // Создание объекта окна
    TriangleWindow window;
    window.setFormat(format); // Применение настроенного формата
    window.setupWindow(); // Настройка параметров окна
    window.resize(640, 480); // Установка начального размера окна
    window.show(); // Отображение окна

    // Включение анимации (постоянного обновления кадров)
    window.setAnimating(true);

    // Запуск главного цикла приложения
    return app.exec();
}

// Вершинный шейдер: преобразует координаты вершин и передает цвета
static const char *vertexShaderSource =
    "attribute highp vec4 posAttr;\n"
    "attribute lowp vec4 colAttr;\n"
    "varying lowp vec4 col;\n"
    "uniform highp mat4 matrix;\n"
    "void main() {\n"
    "   col = colAttr;\n"
    "   gl_Position = matrix * posAttr;\n"
    "}\n";

// Фрагментный шейдер: задает цвет пикселя
static const char *fragmentShaderSource =
    "varying lowp vec4 col;\n"
    "void main() {\n"
    "   gl_FragColor = col;\n"
    "}\n";

// Инициализация OpenGL: настройка шейдеров и атрибутов
void TriangleWindow::initialize() {
    m_program = new QOpenGLShaderProgram(this);
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    Q_ASSERT(m_program->link());

    // Привязка атрибутов шейдера
    m_posAttr = m_program->attributeLocation("posAttr");
    Q_ASSERT(m_posAttr != -1);
    m_colAttr = m_program->attributeLocation("colAttr");
    Q_ASSERT(m_colAttr != -1);
    m_matrixUniform = m_program->uniformLocation("matrix");
    Q_ASSERT(m_matrixUniform != -1);
}

// Метод отрисовки сцены
void TriangleWindow::render() {
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_program->bind();

    QMatrix4x4 matrix;
    matrix.perspective(60.0f, 4.0f / 3.0f, 0.1f, 100.0f);
    matrix.translate(0, 0, -2);
    matrix.rotate(100.0f * m_frame / screen()->refreshRate(), m_rotationAxis);

    m_program->setUniformValue(m_matrixUniform, matrix);

    static const GLfloat vertices[] = {
        0.5f, 0.5f, 0.5f,   0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,
        -0.5f, 0.5f, 0.5f,  0.5f, 0.5f, -0.5f,  0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, -0.5f
    };

    static const GLuint indices[] = {
        0, 3, 1, 1, 3, 2,  6, 7, 5, 4, 5, 7,
        0, 4, 7, 0, 7, 3,  1, 2, 6, 1, 6, 5,
        0, 1, 4, 1, 5, 4,  2, 3, 7, 2, 7, 6
    };

    GLuint EBO;
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    static const GLfloat colors[] = {
        1.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,  1.0f, 0.0f, 1.0f,
        1.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,  1.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,  0.0f, 0.0f, 1.0f
    };

    glVertexAttribPointer(m_posAttr, 3, GL_FLOAT, GL_FALSE, 0, vertices);
    glVertexAttribPointer(m_colAttr, 3, GL_FLOAT, GL_FALSE, 0, colors);

    glEnableVertexAttribArray(m_posAttr);
    glEnableVertexAttribArray(m_colAttr);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glDeleteBuffers(1, &EBO);

    glDisableVertexAttribArray(m_colAttr);
    glDisableVertexAttribArray(m_posAttr);

    m_program->release();

    ++m_frame;
}

// Метод для установки оси вращения
void TriangleWindow::setRotationAxis(QVector3D axis) {
    m_rotationAxis = axis.normalized();
}

// Метод настройки параметров окна
void TriangleWindow::setupWindow() {
    // Установка заголовка окна
    setTitle("Square");

    // Установка минимального размера окна
    setMinimumSize(QSize(320, 240));

    // Установка максимального размера окна (например, ограничение до 1920x1080)
    setMaximumSize(QSize(1920, 1080));
}

// Обработчик нажатия клавиш
void TriangleWindow::keyPressEvent(QKeyEvent *event) {
    switch (event->key()) {
    case Qt::Key_X: // Вращение вокруг оси X
        setRotationAxis(QVector3D(1, 0, 0));
        break;
    case Qt::Key_Y: // Вращение вокруг оси Y
        setRotationAxis(QVector3D(0, 1, 0));
        break;
    case Qt::Key_Z: // Вращение вокруг оси Z
        setRotationAxis(QVector3D(0, 0, 1));
        break;
    case Qt::Key_Space: // Вращение вокруг диагональной оси
        setRotationAxis(QVector3D(1, 1, 1));
        break;
    case Qt::Key_F: // Переключение полноэкранного режима
        m_fullscreen = !m_fullscreen;
        if (m_fullscreen) {
            showFullScreen(); // Переход в полноэкранный режим
        } else {
            showNormal(); // Возврат к обычному режиму
        }
        break;
    case Qt::Key_Escape: // Выход из полноэкранного режима или закрытие окна
        if (m_fullscreen) {
            m_fullscreen = false;
            showNormal();
        } else {
            close(); // Закрытие окна
        }
        break;
    default:
        QWindow::keyPressEvent(event); // Передача необработанных событий базовому классу
    }
}

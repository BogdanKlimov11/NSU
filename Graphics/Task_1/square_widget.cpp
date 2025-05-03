#include <QOpenGLShaderProgram>

#include "square_widget.h"

OpenGLWindow::OpenGLWindow(QWidget *parent) : QOpenGLWidget(parent) {
    setMinimumSize(320, 240);
}

OpenGLWindow::~OpenGLWindow() {
}

void OpenGLWindow::initializeGL() {
    initializeOpenGLFunctions();
    initialize();
}

void OpenGLWindow::paintGL() {
    render();
}

void OpenGLWindow::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void OpenGLWindow::initialize() {
}

void OpenGLWindow::render() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

TriangleWindow::TriangleWindow(QWidget *parent) : OpenGLWindow(parent) {
}

static const char *vertexShaderSource =
    "attribute highp vec4 posAttr;\n"
    "attribute lowp vec4 colAttr;\n"
    "varying lowp vec4 col;\n"
    "uniform highp mat4 matrix;\n"
    "void main() {\n"
    "   col = colAttr;\n"
    "   gl_Position = matrix * posAttr;\n"
    "}\n";

static const char *fragmentShaderSource =
    "varying lowp vec4 col;\n"
    "void main() {\n"
    "   gl_FragColor = col;\n"
    "}\n";

void TriangleWindow::initialize() {
    program = new QOpenGLShaderProgram(this);
    program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    Q_ASSERT(program->link());

    posAttr = program->attributeLocation("posAttr");
    Q_ASSERT(posAttr != -1);
    colAttr = program->attributeLocation("colAttr");
    Q_ASSERT(colAttr != -1);
    matrixUniform = program->uniformLocation("matrix");
    Q_ASSERT(matrixUniform != -1);
}

void TriangleWindow::render() {
    glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program->bind();

    QMatrix4x4 matrix;
    matrix.perspective(60.0f, width() / (float)height(), 0.1f, 100.0f);
    matrix.translate(0, 0, -2);
    matrix.rotate(100.0f * frame / 60.0f * rotationSpeed, rotationAxis);

    program->setUniformValue(matrixUniform, matrix);

    static const GLfloat vertices[] = {
        0.5f, 0.5f, 0.5f,
        0.5f, -0.5f, 0.5f,
        -0.5f, -0.5f, 0.5f,
        -0.5f, 0.5f, 0.5f,
        0.5f, 0.5f, -0.5f,
        0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,
        -0.5f, 0.5f, -0.5f
    };

    static const GLuint indices[] = {
        0, 3, 1, 1, 3, 2,
        6, 7, 5, 4, 5, 7,
        0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5,
        0, 1, 4, 1, 5, 4,
        2, 3, 7, 2, 7, 6
    };

    GLuint EBO;
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    GLfloat colors[8 * 3];
    if (multicolorMode) {
        static const GLfloat multicolor[] = {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            1.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 1.0f
        };
        for (int i = 0; i < 8 * 3; ++i) {
            colors[i] = multicolor[i];
        }
    }
    else {
        for (int i = 0; i < 8; ++i) {
            colors[i * 3 + 0] = cubeColor.x();
            colors[i * 3 + 1] = cubeColor.y();
            colors[i * 3 + 2] = cubeColor.z();
        }
    }

    glVertexAttribPointer(posAttr, 3, GL_FLOAT, GL_FALSE, 0, vertices);
    glVertexAttribPointer(colAttr, 3, GL_FLOAT, GL_FALSE, 0, colors);

    glEnableVertexAttribArray(posAttr);
    glEnableVertexAttribArray(colAttr);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glDeleteBuffers(1, &EBO);

    glDisableVertexAttribArray(colAttr);
    glDisableVertexAttribArray(posAttr);

    program->release();

    ++frame;
    update();
}

void TriangleWindow::setRotationAxis(QVector3D axis) {
    rotationAxis = axis.normalized();
}

void TriangleWindow::setCubeColor(const QVector3D &color) {
    cubeColor = color;
}

void TriangleWindow::setMulticolorMode(bool enabled) {
    multicolorMode = enabled;
}

void TriangleWindow::setRotationSpeed(float speed) {
    rotationSpeed = speed;
}

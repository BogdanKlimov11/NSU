#include "transform_widget.h"

SceneWidget::SceneWidget(QWidget *parent) : QOpenGLWidget(parent) {
    updateGeometry();

    QMatrix4x4 view;
    QMatrix4x4 projection;

    projection.perspective(60.0f, 1.0f, 0.1f, 100.0f);
    view.lookAt({0, 0, -2}, {0, 0, 0}, {1, 0, 0});

    viewProjection = projection * view;

    for (size_t i = 0; i < 6; ++i) {
        rotations.push_back({});
    }

    rotations[1].rotate(90, 1, 0, 0);
    rotations[2].rotate(180, 1, 0, 0);
    rotations[3].rotate(270, 1, 0, 0);
    rotations[4].rotate(90, 0, 1, 0);
    rotations[5].rotate(270, 0, 1, 0);
}

void SceneWidget::setSplitParameter(size_t parameter) {
    splitParameter = parameter;
    updateGeometry();
    update();
}

void SceneWidget::setMorphingParameter(float parameter) {
    morphingParameter = parameter;
    update();
}

void SceneWidget::setRotationX(float angle) {
    rotationX = angle;
    update();
}

void SceneWidget::setRotationY(float angle) {
    rotationY = angle;
    update();
}

void SceneWidget::setRotationZ(float angle) {
    rotationZ = angle;
    update();
}

void SceneWidget::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0, 0, 0, 1);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    shaderProgram = new QOpenGLShaderProgram(this);
    shaderProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":vertex_shader.vsh");
    shaderProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":fragment_shader.fsh");
    shaderProgram->link();
    posAttr = shaderProgram->attributeLocation("posAttr");
    colAttr = shaderProgram->attributeLocation("colAttr");
    matrixUniform = shaderProgram->uniformLocation("mvpMatrix");
    morphingUniform = shaderProgram->uniformLocation("morphingParam");
}

void SceneWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void SceneWidget::paintGL() {
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shaderProgram->bind();

    GLuint EBO;
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(posAttr, 3, GL_FLOAT, GL_FALSE, 0, vertices.data());
    glVertexAttribPointer(colAttr, 3, GL_FLOAT, GL_FALSE, 0, colors.data());

    glEnableVertexAttribArray(posAttr);
    glEnableVertexAttribArray(colAttr);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);

    QMatrix4x4 rotation;
    rotation.rotate(rotationX, 1, 0, 0);
    rotation.rotate(rotationY, 0, 1, 0);
    rotation.rotate(rotationZ, 0, 0, 1);

    for (const auto &model : rotations) {
        shaderProgram->setUniformValue(matrixUniform, viewProjection * rotation * model);
        shaderProgram->setUniformValue(morphingUniform, morphingParameter);
        glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);
    }

    glDisableVertexAttribArray(colAttr);
    glDisableVertexAttribArray(posAttr);

    glDeleteBuffers(1, &EBO);
    shaderProgram->release();
}

void SceneWidget::updateGeometry() {
    vertices.clear();
    colors.clear();
    indices.clear();

    for (size_t j = 0; j < splitParameter + 1; ++j) {
        for (size_t i = 0; i < splitParameter + 1; ++i) {
            vertices.push_back(-0.5f + GLfloat(i) / splitParameter);
            vertices.push_back(-0.5f + GLfloat(j) / splitParameter);
            vertices.push_back(-0.5f);

            colors.push_back(1.0f);
            colors.push_back(1.0f);
            colors.push_back(1.0f);
        }
    }

    for (size_t j = 0; j < splitParameter; ++j) {
        for (size_t i = 0; i < splitParameter; ++i) {
            indices.push_back(j * (splitParameter + 1) + i);
            indices.push_back(j * (splitParameter + 1) + i + 1);

            indices.push_back((j + 1) * (splitParameter + 1) + i);
            indices.push_back((j + 1) * (splitParameter + 1) + i + 1);

            indices.push_back(j * (splitParameter + 1) + i);
            indices.push_back((j + 1) * (splitParameter + 1) + i);

            indices.push_back(j * (splitParameter + 1) + i + 1);
            indices.push_back((j + 1) * (splitParameter + 1) + i + 1);
        }
    }
}

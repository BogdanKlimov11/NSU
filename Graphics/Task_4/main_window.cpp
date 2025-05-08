#include <QColorDialog>
#include <QScreen>
#include <QtMath>

#include "main_window.h"

MainWindow::MainWindow(QWidget *parent) : QOpenGLWidget(parent) {
    lightIntensity = 100.0f;
    fpsTimer.start();
}

void MainWindow::initShaders() {
    program = std::make_unique<QOpenGLShaderProgram>(this);
    program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vertex_shader.vsh");
    program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fragment_shader.fsh");
    program->link();

    posAttr = program->attributeLocation("posAttr");
    normAttr = program->attributeLocation("normAttr");
    Q_ASSERT(posAttr != -1);
}

void MainWindow::initializeGL() {
    initializeOpenGLFunctions();
    initShaders();

    glEnable(GL_DEPTH_TEST);
    timer.start(30, this);

    initCube(1.5f);
}

void MainWindow::paintGL() {
    const auto retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program->bind();

    QMatrix4x4 viewMatrix;
    viewMatrix.setToIdentity();
    viewMatrix.translate(0.0f, 0.0f, -6.0f);

    program->setUniformValue("projectionMatrix", projectionMatrix);
    program->setUniformValue("viewPos", QVector3D(0.0f, 0.0f, 0.0f));
    program->setUniformValue("lightPos", QVector3D(lightX, lightY, lightZ));
    program->setUniformValue("objectColor", cubeColor);
    program->setUniformValue("lightColor", lightColor);
    program->setUniformValue("morphing", morphing);
    program->setUniformValue("lightType", lightType);
    program->setUniformValue("lightIntensity", lightIntensity);
    program->setUniformValue("lightAttenuation", lightAttenuation);
    program->setUniformValue("lightDirection", lightDirection);
    program->setUniformValue("lightCutoff", lightCutoff);
    program->setUniformValue("outerCutoff", outerCutoff);
    program->setUniformValue("ambientStrength", ambientStrength);
    program->setUniformValue("diffuseStrength", diffuseStrength);
    program->setUniformValue("specularStrength", specularStrength);

    vertexBuffer.bind();
    indexBuffer.bind();
    program->setAttributeBuffer(posAttr, GL_FLOAT, 0, 3, sizeof(VertexData));
    program->setAttributeBuffer(normAttr, GL_FLOAT, sizeof(QVector3D), 3, sizeof(VertexData));
    program->enableAttributeArray(posAttr);
    program->enableAttributeArray(normAttr);

    float offsetX = -2.5f;
    float offsetY = -2.5f;
    float offsetZ = -2.5f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                QMatrix4x4 modelMatrix;
                modelMatrix.setToIdentity();
                modelMatrix.translate(offsetX + i * 2.5f, offsetY + j * 2.5f, offsetZ - k * 2.5f);
                modelMatrix.rotate(100.0 * frame / 59.0, rotationAxis);

                program->setUniformValue("model", modelMatrix);
                program->setUniformValue("viewMatrix", viewMatrix);
                QMatrix4x4 modelViewMatrix = viewMatrix * modelMatrix;
                program->setUniformValue("normMatrix", modelViewMatrix.normalMatrix());

                glDrawElements(GL_TRIANGLES, indexBuffer.size() / sizeof(GLuint), GL_UNSIGNED_INT, nullptr);
            }
        }
    }

    program->disableAttributeArray(posAttr);
    program->disableAttributeArray(normAttr);
    vertexBuffer.release();
    indexBuffer.release();
    program->release();
    ++frame;

    frameCount++;
    qint64 elapsed = fpsTimer.elapsed();
    if (elapsed >= 1000) {
        currentFps = frameCount * 1000 / elapsed;
        emit fpsUpdated(currentFps);
        frameCount = 0;
        fpsTimer.restart();
    }
}

void MainWindow::resizeGL(const int w, const int h) {
    const auto aspect = w / static_cast<double>(h);
    projectionMatrix.setToIdentity();
    projectionMatrix.perspective(60.0f, aspect, 0.01f, 100.0f);
}

void MainWindow::changeMorphing(float value) {
    morphing = value / 1000;
    update();
}

void MainWindow::changeLightX(float value) {
    if (lightType == 0) {
        lightDirection.setX(value / 100.0f);
    }
    else {
        lightX = value / 100.0f;
        lightDirection.setX(-lightX);
        if (lightDirection.length() < 0.001f) {
            lightDirection.setZ(-1.0f);
        }
    }
    update();
}

void MainWindow::changeLightY(float value) {
    if (lightType == 0) {
        lightDirection.setY(value / 100.0f);
    }
    else {
        lightY = value / 100.0f;
        lightDirection.setY(-lightY);
        if (lightDirection.length() < 0.001f) {
            lightDirection.setZ(-1.0f);
        }
    }
    update();
}

void MainWindow::changeLightZ(float value) {
    if (lightType == 0) {
        lightDirection.setZ(value / 100.0f);
    }
    else {
        lightZ = 2.0f + (value / 100.0f) * 3.0f;
        lightDirection.setZ(-lightZ);
        if (lightDirection.length() < 0.001f) {
            lightDirection.setZ(-1.0f);
        }
    }
    update();
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key::Key_Plus) {
        const auto chosenColor = QColorDialog::getColor();
        cubeColor =
            QVector3D(chosenColor.red() / 255.0f, chosenColor.green() / 255.0f, chosenColor.blue() / 255.0f);
    }
    else if (event->key() == Qt::Key::Key_Minus) {
        const auto chosenColor = QColorDialog::getColor();
        lightColor = QVector3D(chosenColor.red() / 255.0f, chosenColor.green() / 255.0f, chosenColor.blue() / 255.0f);
    }
}

void MainWindow::timerEvent(QTimerEvent *e) {
    Q_UNUSED(e);
    update();
}

void MainWindow::initCube(const float width) {
    auto halfWidth = width / 2.0f;
    auto step = cubeSubdivision > 1 ? width / float(cubeSubdivision - 1) : width;

    std::vector<VertexData> vertexes;
    vertexes.reserve(6 * pow(cubeSubdivision, 2));
    for (auto z = -halfWidth; z <= halfWidth; z += width) {
        for (auto j = 0; j < cubeSubdivision; j++) {
            for (auto i = 0; i < cubeSubdivision; i++) {
                vertexes.emplace_back(
                    VertexData(QVector3D(-z + i * step * z / halfWidth, -halfWidth + j * step, z), QVector3D(0.0, 0.0, z / halfWidth)));
            }
        }
    }
    for (auto x = -halfWidth; x <= halfWidth; x += width) {
        for (auto k = 0; k < cubeSubdivision; k++) {
            for (auto j = 0; j < cubeSubdivision; j++) {
                vertexes.emplace_back(
                    VertexData(QVector3D(x, -halfWidth + j * step, -x + x * k * step / halfWidth), QVector3D(x / halfWidth, 0.0, 0.0)));
            }
        }
    }
    for (auto y = -halfWidth; y <= halfWidth; y += width) {
        for (auto i = 0; i < cubeSubdivision; i++) {
            for (auto k = 0; k < cubeSubdivision; k++) {
                vertexes.emplace_back(
                    VertexData(QVector3D(-halfWidth + i * step, y, -y + y * k * step / halfWidth), QVector3D(0.0, y / halfWidth, 0.0)));
            }
        }
    }

    std::vector<GLuint> indexes;
    int vertexCount = cubeSubdivision > 1 ? 36 * pow(cubeSubdivision - 1, 2) : 36;
    indexes.reserve(vertexCount);
    if (cubeSubdivision > 1) {
        for (int i = 0; i < 6 * cubeSubdivision * cubeSubdivision; i += cubeSubdivision * cubeSubdivision) {
            for (int j = 0; j < (cubeSubdivision - 1) * (cubeSubdivision - 1); j += cubeSubdivision) {
                for (int k = 0; k < (cubeSubdivision - 1); k++) {
                    indexes.emplace_back(i + j + k + cubeSubdivision);
                    indexes.emplace_back(i + j + k + 0);
                    indexes.emplace_back(i + j + k + cubeSubdivision + 1);
                    indexes.emplace_back(i + j + k + cubeSubdivision + 1);
                    indexes.emplace_back(i + j + k + 0);
                    indexes.emplace_back(i + j + k + 1);
                }
            }
        }
    }
    else {
        for (int i = 0; i < 6; i++) {
            int base = i * cubeSubdivision * cubeSubdivision;
            indexes.emplace_back(base);
            indexes.emplace_back(base + 1);
            indexes.emplace_back(base + 2);
            indexes.emplace_back(base + 2);
            indexes.emplace_back(base + 1);
            indexes.emplace_back(base + 3);
        }
    }

    vertexBuffer.create();
    vertexBuffer.bind();
    vertexBuffer.allocate(vertexes.data(), static_cast<int>(vertexes.size() * sizeof(VertexData)));

    indexBuffer.create();
    indexBuffer.bind();
    indexBuffer.allocate(indexes.data(), static_cast<int>(indexes.size() * sizeof(GLuint)));
}

void MainWindow::changeCubeColor() {
    const auto chosenColor = QColorDialog::getColor();
    cubeColor =
        QVector3D(chosenColor.red() / 255.0f,
                  chosenColor.green() / 255.0f,
                  chosenColor.blue() / 255.0f);
    update();
}

void MainWindow::changeLightColor() {
    const auto chosenColor = QColorDialog::getColor();
    lightColor =
        QVector3D(chosenColor.red() / 255.0f,
                  chosenColor.green() / 255.0f,
                  chosenColor.blue() / 255.0f);
    update();
}

void MainWindow::changeSubdivision(int count) {
    cubeSubdivision = count;
    vertexBuffer.destroy();
    indexBuffer.destroy();
    initCube(1.5f);
    update();
}

void MainWindow::changeRotationAxisX(float value) {
    rotationAxis.setX(value / 100.0f);
    update();
}

void MainWindow::changeRotationAxisY(float value) {
    rotationAxis.setY(value / 100.0f);
    update();
}

void MainWindow::changeRotationAxisZ(float value) {
    rotationAxis.setZ(value / 100.0f);
    update();
}

void MainWindow::setLightType(int type) {
    lightType = type;
    if (lightType == 2 && lightDirection.length() < 0.001f) {
        lightDirection.setZ(-1.0f);
    }
    update();
}

void MainWindow::setLightIntensity(float intensity) {
    lightIntensity = intensity;
    update();
}

void MainWindow::setLightAttenuation(float attenuation) {
    lightAttenuation = attenuation / 1000.0f;
    update();
}

void MainWindow::setLightCutoff(float cutoff) {
    lightCutoff = cutoff / 100.0f;
    update();
}

void MainWindow::setOuterCutoff(float cutoff) {
    outerCutoff = cutoff / 100.0f;
    update();
}

void MainWindow::setLightDirection(QVector3D direction) {
    lightDirection = direction;
    update();
}

void MainWindow::setAmbientStrength(float strength) {
    ambientStrength = strength / 100.0f;
    update();
}

void MainWindow::setDiffuseStrength(float strength) {
    diffuseStrength = strength / 100.0f;
    update();
}

void MainWindow::setSpecularStrength(float strength) {
    specularStrength = strength / 100.0f;
    update();
}

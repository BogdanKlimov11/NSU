#include <cmath>

#include "earth_geometry.h"

EarthGeometry::EarthGeometry(QWidget *parent) : QOpenGLWidget(parent) {}

void EarthGeometry::initializeGL() {
    initializeOpenGLFunctions();

    program = std::make_unique<QOpenGLShaderProgram>(this);
    program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vertex_shader.vsh");
    program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fragment_shader.fsh");
    program->link();

    posAttr = program->attributeLocation("posAttr");
    normAttr = program->attributeLocation("normAttr");
    textureAttr = program->attributeLocation("texAttr");
    tangentAttr = program->attributeLocation("tangentAttr");
    bitangentAttr = program->attributeLocation("bitangentAttr");
    useTextureUniform = program->uniformLocation("useTexture");
    useNormalMapUniform = program->uniformLocation("useNormalMap");
    useNightTextureUniform = program->uniformLocation("useNightTexture");

    material = std::make_unique<EarthMaterial>();
    initSphere(1.5f);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    rotationAxis = QVector3D(0.0f, 1.0f, 0.0f);

    timer.start(30, this);
    lightTimer.start(30, this);
}

void EarthGeometry::paintGL() {
    glViewport(0, 0, width() * devicePixelRatio(), height() * devicePixelRatio());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program->bind();

    QMatrix4x4 viewMatrix, modelMatrix;
    viewMatrix.setToIdentity();
    viewMatrix.translate(0.0, 0.0, -5.0);
    modelMatrix.setToIdentity();
    modelMatrix.rotate(currentRotationAngle, rotationAxis);

    if (rotationEnabled) {
        currentRotationAngle -= 10.0f / 59.0f;
        currentRotationAngle = fmod(currentRotationAngle, 360.0f);
    }

    float lightX = 5.0f * cos(currentLightAngle);
    float lightZ = 5.0f * sin(currentLightAngle);
    QVector3D lightPos = QVector3D(lightX, 0.0f, lightZ);

    program->setUniformValue("projectionMatrix", projectionMatrix);
    program->setUniformValue("model", modelMatrix);
    program->setUniformValue("viewMatrix", viewMatrix);
    program->setUniformValue("normMatrix", modelMatrix.normalMatrix());
    program->setUniformValue("viewPos", QVector3D(0.0, 0.0, 5.0));
    program->setUniformValue("lightPos", lightPos);
    program->setUniformValue(useTextureUniform, useTexture ? 1 : 0);
    program->setUniformValue(useNormalMapUniform, useNormalMap ? 1 : 0);
    program->setUniformValue(useNightTextureUniform, useNightTexture ? 1 : 0);

    material->bind(program.get(), useTexture, useNormalMap, useNightTexture);

    vertexBuffer.bind();
    float offset = 0;
    if (posAttr != -1) {
        program->enableAttributeArray(posAttr);
        program->setAttributeBuffer(posAttr, GL_FLOAT, offset, 3, sizeof(VertexData));
    }
    offset += sizeof(QVector3D);
    if (normAttr != -1) {
        program->enableAttributeArray(normAttr);
        program->setAttributeBuffer(normAttr, GL_FLOAT, offset, 3, sizeof(VertexData));
    }
    offset += sizeof(QVector3D);
    if (textureAttr != -1) {
        program->enableAttributeArray(textureAttr);
        program->setAttributeBuffer(textureAttr, GL_FLOAT, offset, 2, sizeof(VertexData));
    }
    offset += sizeof(QVector2D);
    if (tangentAttr != -1) {
        program->enableAttributeArray(tangentAttr);
        program->setAttributeBuffer(tangentAttr, GL_FLOAT, offset, 3, sizeof(VertexData));
    }
    offset += sizeof(QVector3D);
    if (bitangentAttr != -1) {
        program->enableAttributeArray(bitangentAttr);
        program->setAttributeBuffer(bitangentAttr, GL_FLOAT, offset, 3, sizeof(VertexData));
    }

    indexBuffer.bind();
    glDrawElements(GL_TRIANGLES, indexBuffer.size(), GL_UNSIGNED_INT, nullptr);

    if (posAttr != -1) program->disableAttributeArray(posAttr);
    if (normAttr != -1) program->disableAttributeArray(normAttr);
    if (textureAttr != -1) program->disableAttributeArray(textureAttr);
    if (tangentAttr != -1) program->disableAttributeArray(tangentAttr);
    if (bitangentAttr != -1) program->disableAttributeArray(bitangentAttr);

    program->release();

    ++frame;
}

void EarthGeometry::resizeGL(int w, int h) {
    projectionMatrix.setToIdentity();
    projectionMatrix.perspective(60.0f, w / static_cast<float>(h), 0.1f, 100.0f);
}

void EarthGeometry::timerEvent(QTimerEvent *e) {
    if (e->timerId() == timer.timerId()) {
        update();
    }
    else if (e->timerId() == lightTimer.timerId()) {
        updateLightPosition();
    }
}

void EarthGeometry::updateLightPosition() {
    if (fabs(lightSpeed) > 1e-6f) {
        currentLightAngle -= lightSpeed;
        currentLightAngle = fmod(currentLightAngle, 2.0f * M_PI);
    }
}

void EarthGeometry::initSphere(float radius) {
    const int lats = 50, longs = 50;
    const float latStep = 2 * M_PI / lats;
    const float longStep = M_PI / longs;

    std::vector<VertexData> vertexes;
    for (unsigned i = 0; i <= longs; ++i) {
        float phi = i * longStep;
        float sinPhi = sin(phi);
        float cosPhi = cos(phi);

        for (unsigned j = 0; j <= lats; ++j) {
            float theta = j * latStep;
            float sinTheta = sin(theta);
            float cosTheta = cos(theta);

            float x = radius * sinPhi * cosTheta;
            float z = radius * sinPhi * sinTheta;
            float y = radius * cosPhi;

            QVector3D pos{x, y, z};
            QVector3D norm = pos.normalized();

            float u = (float)j / lats;
            float v = (float)i / longs;

            QVector3D tangent = QVector3D(-radius * sinPhi * sinTheta, 0.0f, radius * sinPhi * cosTheta).normalized();
            QVector3D bitangent = QVector3D::crossProduct(norm, tangent).normalized();

            vertexes.emplace_back(pos, norm, QVector2D(u, v), tangent, bitangent);
        }
    }

    std::vector<GLuint> indexes;
    for (unsigned i = 0; i < lats; ++i) {
        unsigned k1 = i * (longs + 1);
        unsigned k2 = k1 + longs + 1;

        for (unsigned j = 0; j < longs; ++j, ++k1, ++k2) {
            if (i != 0) {
                indexes.push_back(k1);
                indexes.push_back(k2);
                indexes.push_back(k1 + 1);
            }
            if (i != lats - 1) {
                indexes.push_back(k1 + 1);
                indexes.push_back(k2);
                indexes.push_back(k2 + 1);
            }
        }
    }

    vertexBuffer.create();
    vertexBuffer.bind();
    vertexBuffer.allocate(vertexes.data(), vertexes.size() * sizeof(VertexData));

    indexBuffer.create();
    indexBuffer.bind();
    indexBuffer.allocate(indexes.data(), indexes.size() * sizeof(GLuint));
}

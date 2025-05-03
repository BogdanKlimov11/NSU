#pragma once

#include <QBasicTimer>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QTimerEvent>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QVector3D>
#include <memory>

#include "earth_material.h"

struct VertexData {
    VertexData(QVector3D p, QVector3D n, QVector2D t, QVector3D tng, QVector3D btg)
        : position(p), normal(n), textcoord(t), tangent(tng), bitangent(btg) {}
    QVector3D position;
    QVector3D normal;
    QVector2D textcoord;
    QVector3D tangent;
    QVector3D bitangent;
};

class EarthGeometry : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit EarthGeometry(QWidget *parent = nullptr);
    ~EarthGeometry() override = default;
    void setLightSpeed(float speed) { lightSpeed = speed; update(); }

public slots:
    void setUseTexture(bool use) { useTexture = use; update(); }
    void setUseNormalMap(bool use) { useNormalMap = use; update(); }
    void setUseNightTexture(bool use) { useNightTexture = use; update(); }
    void setRotationEnabled(bool enabled) { rotationEnabled = enabled; update(); }

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void timerEvent(QTimerEvent *e) override;
    void initSphere(float radius);
    void updateLightPosition();

    GLint posAttr = -1;
    GLint normAttr = -1;
    GLint textureAttr = -1;
    GLint tangentAttr = -1;
    GLint bitangentAttr = -1;
    GLint useTextureUniform = -1;
    GLint useNormalMapUniform = -1;
    GLint useNightTextureUniform = -1;

    std::unique_ptr<QOpenGLShaderProgram> program;
    std::unique_ptr<EarthMaterial> material;

    QOpenGLBuffer vertexBuffer;
    QOpenGLBuffer indexBuffer{QOpenGLBuffer::IndexBuffer};

    QMatrix4x4 projectionMatrix;

    int frame = 0;
    float lightSpeed = 0.1f;
    float currentLightAngle = 0.0f;
    float currentRotationAngle = 0.0f;
    QVector3D rotationAxis{0., 1., 0.};
    QBasicTimer timer;
    QBasicTimer lightTimer;
    bool useTexture = true;
    bool useNormalMap = true;
    bool useNightTexture = false;
    bool rotationEnabled = true;
};

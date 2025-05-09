#pragma once

#include <QBasicTimer>
#include <QColor>
#include <QColorDialog>
#include <QKeyEvent>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QVector3D>
#include <QOpenGLFunctions>
#include <QElapsedTimer>
#include <memory>

struct VertexData {
    VertexData() {}
    VertexData(QVector3D p, QVector3D n) : position(p), normal(n) {}
    QVector3D position;
    QVector3D normal;
};

class MainWindow : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override = default;

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void wheelEvent(QWheelEvent *event) override;

    void changeMorphing(float value);
    void keyPressEvent(QKeyEvent *e) override;

    void changeLightX(float value);
    void changeLightY(float value);
    void changeLightZ(float value);

    void changeCubeColor();
    void changeLightColor();
    void changeCubeCountX(int count);
    void changeCubeCountY(int count);
    void changeCubeCountZ(int count);
    void changeRotationAxisX(float value);
    void changeRotationAxisY(float value);
    void changeRotationAxisZ(float value);

public slots:
    void setLightType(int type);
    void setLightIntensity(float intensity);
    void setLightAttenuation(float attenuation);
    void setLightCutoff(float cutoff);
    void setLightDirection(QVector3D direction);
    void setOuterCutoff(float cutoff);
    void setAmbientStrength(float strength);
    void setDiffuseStrength(float strength);
    void setSpecularStrength(float strength);

signals:
    void fpsUpdated(int fps);
    void cubeCountXChanged(int count);
    void cubeCountYChanged(int count);

private:
    void timerEvent(QTimerEvent *e) override;
    void initCube(float width);
    void initShaders();

    GLint posAttr = 0;
    GLint normAttr = 0;

    std::unique_ptr<QOpenGLShaderProgram> program = nullptr;

    QOpenGLBuffer vertexBuffer;
    QOpenGLBuffer indexBuffer{QOpenGLBuffer::IndexBuffer};
    float outerCutoff = 0.8f;
    int lightType = 0;
    float lightIntensity = 1.0f;
    float lightAttenuation = 0.1f;
    QVector3D lightDirection{0.0f, 0.0f, -1.0f};
    float lightCutoff = 0.9f;
    float lightX = 0.0f;
    float lightY = 0.0f;
    float lightZ = 0.0f;
    QMatrix4x4 projectionMatrix;
    QVector3D lightColor{1.0f, 1.0f, 1.0f};

    QVector3D cubeColor{1.0f, 0.5f, 0.0f};
    float morphing = 1.0f;
    int cubeCountX = 4;
    int cubeCountY = 4;
    int cubeCountZ = 4;
    float ambientStrength = 0.3f;
    float diffuseStrength = 0.5f;
    float specularStrength = 0.5f;
    float cameraDistance = 10.0f;

    int frame = 0;
    QVector3D rotationAxis{0., 0., 1.};
    QBasicTimer timer;

    QElapsedTimer fpsTimer;
    int frameCount = 0;
    int currentFps = 0;
};

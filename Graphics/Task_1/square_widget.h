#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QVector3D>

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

class OpenGLWindow : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit OpenGLWindow(QWidget *parent = nullptr);
    ~OpenGLWindow();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    virtual void initialize();
    virtual void render();

    QOpenGLShaderProgram *program = nullptr;
};

class TriangleWindow : public OpenGLWindow {
    Q_OBJECT
public:
    explicit TriangleWindow(QWidget *parent = nullptr);

    void initialize() override;
    void render() override;
    void setRotationAxis(QVector3D axis);
    void setCubeColor(const QVector3D &color);
    void setMulticolorMode(bool enabled);
    void setRotationSpeed(float speed);

private:
    GLint posAttr = 0;
    GLint colAttr = 0;
    GLint matrixUniform = 0;
    int frame = 0;
    QVector3D rotationAxis = {1.0f, 1.0f, 1.0f};
    QVector3D cubeColor = {1.0f, 1.0f, 1.0f};
    bool multicolorMode = false;
    float rotationSpeed = 1.0f;
};

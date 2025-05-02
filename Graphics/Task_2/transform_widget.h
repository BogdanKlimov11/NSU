#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QTimer>

class SceneWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit SceneWidget(QWidget *parent = nullptr);
    ~SceneWidget() = default;

    void setSplitParameter(size_t parameter);
    void setMorphingParameter(float parameter);
    void setRotationX(float angle);
    void setRotationY(float angle);
    void setRotationZ(float angle);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void updateGeometry();

    QMatrix4x4 viewProjection;
    std::vector<QMatrix4x4> rotations;

    std::vector<GLfloat> vertices;
    std::vector<GLfloat> colors;
    std::vector<GLuint> indices;

    float rotationX { 0 };
    float rotationY { 0 };
    float rotationZ { 0 };

    GLint posAttr { 0 };
    GLint colAttr { 0 };
    GLint matrixUniform { 0 };
    GLint morphingUniform { 0 };
    QOpenGLShaderProgram *shaderProgram { nullptr };

    float morphingParameter { 0 };
    size_t splitParameter { 10 };
};

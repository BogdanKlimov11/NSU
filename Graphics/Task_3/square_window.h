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
#include <memory>

struct VertexData {
    VertexData() {}
    VertexData(QVector3D p, QVector3D n) : position(p), normal(n) {}
    QVector3D position;
    QVector3D normal;
};

class SquareWindow : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit SquareWindow(QWidget *parent = nullptr);
    ~SquareWindow() override = default;

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void change_morph_param(float value);
    void keyPressEvent(QKeyEvent *e) override;

    void change_light_x_param(float value);
    void change_light_y_param(float value);
    void change_light_z_param(float value);

    void change_cube_color();
    void change_light_color();
    void changeN(int new_n);

public slots:
    void setLightType(int type);
    void setLightIntensity(float intensity);
    void setLightAttenuation(float attenuation);
    void setLightCutoff(float cutoff);
    void setLightDirection(QVector3D direction);

    void setMaterialAmbient();
    void setMaterialDiffuse();
    void setMaterialSpecular();
    void setGlobalAmbient();

private:
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void timerEvent(QTimerEvent *e) override;
    void init_cube(float width, int N);

    GLint posAttr_ = 0;
    GLint normAttr_ = 0;

    std::unique_ptr<QOpenGLShaderProgram> program_ = nullptr;

    QOpenGLBuffer vertexBuffer;
    QOpenGLBuffer indexBuffer{QOpenGLBuffer::IndexBuffer};

    int lightType = 0;
    float lightIntensity = 1.0f;
    float lightAttenuation = 0.1f;
    QVector3D lightDirection{0.0f, 0.0f, -1.0f};
    float lightCutoff = 0.9f;
    float light_x_param = 1.0f;
    float light_y_param = 1.0f;
    float light_z_param = 1.0f;
    QMatrix4x4 projection_matrix;
    QVector3D lightColor{1.0f, 1.0f, 1.0f};

    QVector3D cube_color{1.0f, 0.0f, 0.0f};
    float morph_param = 1.0f;
    int n = 2;

    QVector3D materialAmbient{1.0f, 0.0f, 0.0f};
    QVector3D materialDiffuse{1.0f, 0.0f, 0.0f};
    QVector3D materialSpecular{1.0f, 1.0f, 1.0f};
    QVector3D globalAmbient{0.5f, 0.3f, 0.3f};

    int frame_ = 0;
    QVector2D mousePressPosition{0., 0.};
    QVector3D rotationAxis{0., 0., 1.};
    QBasicTimer timer;
};

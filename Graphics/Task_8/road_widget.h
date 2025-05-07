#pragma once

#include <QOpenGLWidget>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QColor>
#include <QPoint>

#include "road_material.h"

class RoadWidget final : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public slots:
    void changeRoadFiltering(int filt) const;
    void changeRockFiltering(int filt) const;
    void changeCracksFiltering(int filt) const;
    void changeMixParameter(int value);
    void setAutoSpeed(float speed);
    void setCameraHeight(float height);
    void toggleAutoMode(bool enabled);

private slots:
    void updateCamera();

private:
    std::shared_ptr<RoadMaterial> road;
    QOpenGLShaderProgram* program = nullptr;
    QColor backColor = {0, 0, 0};
    QVector3D lightPos;
    QVector3D lightColor;
    QVector3D ambientColor;
    QVector3D specularColor = {1.0f, 1.0f, 1.0f};
    QVector3D viewPos = {0.0f, 0.2f, 0.0f};
    QPoint lastMousePos;
    GLint lightPosUniform = -1;
    GLint ambientIntensityUniform = -1;
    GLint diffuseIntensityUniform = -1;
    GLint ambientColorUniform = -1;
    GLint lightColorUniform = -1;
    GLint specularIntensityUniform = -1;
    GLint specularColorUniform = -1;
    GLint viewPosUniform = -1;
    GLint mixParameterUniform = -1;
    float ambientIntensity = 0.5f;
    float specularIntensity = 0.5f;
    float diffuseIntensity = 0.7f;
    float mixParameter = 0.5f;
    float autoSpeed = 0.5f;
    float startZ = 0.0f;
    float time = 0.0f;
    bool autoMode = false;

    void initUniforms();
    void setUniforms() const;
    void setLightPos(QVector3D pos);
    void setLightColor(QColor color);
    void setAmbientColor(QColor color);
    void setSpecularColor(QColor color);

public:
    explicit RoadWidget(QWidget* parent = nullptr);
    ~RoadWidget();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void wheelEvent(QWheelEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
};

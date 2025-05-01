#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QVector3D>
#include <QMatrix4x4>
#include <QPoint>
#include <vector>

#include "object_loader.h"

class Camera {
public:
    Camera() {
        position = QVector3D(0.0f, 0.0f, 7.0f);
        target = QVector3D(0.0f, 0.0f, 0.0f);
        up = QVector3D(0.0f, 1.0f, 0.0f);
    }

    void setPosZ(float z) {
        position.setZ(z);
    }

    QMatrix4x4 getViewMatrix() const {
        QMatrix4x4 view;
        view.lookAt(position, target, up);
        return view;
    }

    QVector3D getPosition() const {
        return position;
    }

private:
    QVector3D position;
    QVector3D target;
    QVector3D up;
};

class MeshWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    MeshWidget(QWidget* parent = nullptr);
    ~MeshWidget();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void initShaders();
    QOpenGLShaderProgram objectProgram;
    std::vector<Object*> objectObjects;
    Camera camera;
    float ka = 0.5f;
    float kd = 0.7f;
    float ks = 1.0f;
    QVector3D lightDir = QVector3D(-1.0f, -1.0f, -1.0f);
    bool lightEnabled = true;
    int frameCount = 0;
    int currentObjectIndex = 0;
    bool isDragging = false;
    QPoint lastMousePos;
    float rotationX = 0.0f;
    float rotationY = 0.0f;
    QVector3D objectColor = QVector3D(1.0f, 1.0f, 1.0f); // Добавлено для хранения текущего цвета

public slots:
    void moveCamera(int position);
    void setAmbientCoef(int value);
    void setDiffuseCoef(int value);
    void setSpecularCoef(int value);
    void switchDirectedLight();
    void setDirectedLightX(double x);
    void setDirectedLightY(double y);
    void setDirectedLightZ(double z);
    void setCurrentObject(int index);
    void setColorRed(int value);   // Добавлено объявление
    void setColorGreen(int value); // Добавлено объявление
    void setColorBlue(int value);  // Добавлено объявление
};

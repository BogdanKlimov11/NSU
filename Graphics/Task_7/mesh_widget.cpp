#include <QMouseEvent>

#include "mesh_widget.h"

MeshWidget::MeshWidget(QWidget* parent) : QOpenGLWidget(parent) {
}

MeshWidget::~MeshWidget() {
    makeCurrent();
    for (auto obj : objectObjects) {
        delete obj;
    }
    objectProgram.release();
    doneCurrent();
}

void MeshWidget::initializeGL() {
    initializeOpenGLFunctions();

    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    initShaders();

    objectObjects.emplace_back(new Object("MeshCube", ":/mesh_cube.obj", 1.0f));
    objectObjects.emplace_back(new Object("MeshHouse", ":/mesh_house.obj", 0.5f));
    objectObjects.emplace_back(new Object("MeshSphere", ":/mesh_sphere.obj", 0.8f));

    for (auto obj : objectObjects) {
        obj->initialize();
    }
}

void MeshWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void MeshWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    objectProgram.bind();

    QMatrix4x4 projection;
    projection.perspective(60.0f, width() / float(height()), 0.1f, 100.0f);
    QMatrix4x4 view = camera.getViewMatrix();
    QVector3D cameraPos = camera.getPosition();

    objectProgram.setUniformValue("projection", projection);
    objectProgram.setUniformValue("view", view);
    objectProgram.setUniformValue("lightDir", lightDir);
    objectProgram.setUniformValue("lightEnabled", lightEnabled);
    objectProgram.setUniformValue("ka", ka);
    objectProgram.setUniformValue("kd", kd);
    objectProgram.setUniformValue("ks", ks);
    objectProgram.setUniformValue("cameraPos", cameraPos);

    if (currentObjectIndex >= 0 && currentObjectIndex < objectObjects.size()) {
        auto obj = objectObjects[currentObjectIndex];
        if (obj->isValid()) {
            QMatrix4x4 model;
            model.translate(0.0f, 0.0f, 0.0f);
            model.rotate(rotationX, 1, 0, 0);
            model.rotate(rotationY, 0, 1, 0);
            objectProgram.setUniformValue("model", model);

            obj->render(objectProgram);
        }
    }

    objectProgram.release();

    update();
}

void MeshWidget::initShaders() {
    objectProgram.removeAllShaders();
    QOpenGLShader* vertexShader = new QOpenGLShader(QOpenGLShader::Vertex, this);
    vertexShader->compileSourceFile(":/vertex_shader.vsh");
    objectProgram.addShader(vertexShader);

    QOpenGLShader* fragmentShader = new QOpenGLShader(QOpenGLShader::Fragment, this);
    fragmentShader->compileSourceFile(":/fragment_shader.fsh");
    objectProgram.addShader(fragmentShader);

    objectProgram.bindAttributeLocation("position", 0);
    objectProgram.bindAttributeLocation("normal", 1);
    objectProgram.bindAttributeLocation("texCoord", 2);

    objectProgram.link();
    objectProgram.bind();
}

void MeshWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        isDragging = true;
        lastMousePos = event->pos();
    }
}

void MeshWidget::mouseMoveEvent(QMouseEvent* event) {
    if (isDragging) {
        QPoint delta = event->pos() - lastMousePos;
        rotationY += delta.x() * 0.5f;
        rotationX += delta.y() * 0.5f;
        lastMousePos = event->pos();
        update();
    }
}

void MeshWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        isDragging = false;
    }
}

void MeshWidget::moveCamera(int position) {
    camera.setPosZ(static_cast<float>(position));
    update();
}

void MeshWidget::setAmbientCoef(int value) {
    ka = static_cast<float>(value) / 20.0f;
    update();
}

void MeshWidget::setDiffuseCoef(int value) {
    kd = static_cast<float>(value) / 20.0f;
    update();
}

void MeshWidget::setSpecularCoef(int value) {
    ks = static_cast<float>(value) / 20.0f;
    update();
}

void MeshWidget::switchDirectedLight() {
    lightEnabled = !lightEnabled;
    update();
}

void MeshWidget::setDirectedLightX(double x) {
    lightDir.setX(static_cast<float>(x));
    update();
}

void MeshWidget::setDirectedLightY(double y) {
    lightDir.setY(static_cast<float>(y));
    update();
}

void MeshWidget::setDirectedLightZ(double z) {
    lightDir.setZ(static_cast<float>(z));
    update();
}

void MeshWidget::setCurrentObject(int index) {
    currentObjectIndex = index;
    update();
}

void MeshWidget::setColorRed(int value) {
    objectColor.setX(static_cast<float>(value) / 255.0f);
    if (currentObjectIndex >= 0 && currentObjectIndex < objectObjects.size()) {
        auto obj = objectObjects[currentObjectIndex];
        obj->setMaterialColor(objectColor);
    }
    update();
}

void MeshWidget::setColorGreen(int value) {
    objectColor.setY(static_cast<float>(value) / 255.0f);
    if (currentObjectIndex >= 0 && currentObjectIndex < objectObjects.size()) {
        auto obj = objectObjects[currentObjectIndex];
        obj->setMaterialColor(objectColor);
    }
    update();
}

void MeshWidget::setColorBlue(int value) {
    objectColor.setZ(static_cast<float>(value) / 255.0f);
    if (currentObjectIndex >= 0 && currentObjectIndex < objectObjects.size()) {
        auto obj = objectObjects[currentObjectIndex];
        obj->setMaterialColor(objectColor);
    }
    update();
}

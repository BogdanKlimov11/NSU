#include <QMouseEvent>
#include <QOpenGLFunctions>
#include <QTimer>

#include "road_widget.h"

RoadWidget::RoadWidget(QWidget* parent) : QOpenGLWidget(parent), QOpenGLFunctions() {
    setAmbientColor({255, 255, 255});
    setLightColor({255, 255, 255});
    setLightPos({2, 2, 1});
    road = std::make_shared<RoadMaterial>();
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &RoadWidget::updateCamera);
    timer->start(16);
    autoSpeed = 0.5f;
    ambientIntensity = 0.5f;
    diffuseIntensity = 0.7f;
    specularIntensity = 0.2f;
}

RoadWidget::~RoadWidget() {
    makeCurrent();
    delete program;
    doneCurrent();
}

void RoadWidget::initUniforms() {
    lightPosUniform = program->uniformLocation("lightPos");
    ambientIntensityUniform = program->uniformLocation("ambientIntensity");
    ambientColorUniform = program->uniformLocation("ambientColor");
    lightColorUniform = program->uniformLocation("lightColor");
    diffuseIntensityUniform = program->uniformLocation("diffuseIntensity");
    specularIntensityUniform = program->uniformLocation("specularIntensity");
    specularColorUniform = program->uniformLocation("specularColor");
    viewPosUniform = program->uniformLocation("viewPos");
    mixParameterUniform = program->uniformLocation("mixParameter");
}

void RoadWidget::setUniforms() const {
    program->setUniformValue(lightPosUniform, lightPos);
    program->setUniformValue(lightColorUniform, lightColor);
    program->setUniformValue(ambientIntensityUniform, ambientIntensity);
    program->setUniformValue(diffuseIntensityUniform, diffuseIntensity);
    program->setUniformValue(ambientColorUniform, ambientColor);
    program->setUniformValue(specularIntensityUniform, specularIntensity);
    program->setUniformValue(specularColorUniform, specularColor);
    program->setUniformValue(viewPosUniform, viewPos);
    program->setUniformValue(mixParameterUniform, mixParameter);
}

void RoadWidget::setLightPos(const QVector3D pos) { lightPos = pos; }
void RoadWidget::setLightColor(const QColor color) { lightColor = {color.redF(), color.greenF(), color.blueF()}; }
void RoadWidget::setAmbientColor(const QColor color) { ambientColor = {color.redF(), color.greenF(), color.blueF()}; }
void RoadWidget::setSpecularColor(const QColor color) { specularColor = {color.redF(), color.greenF(), color.blueF()}; }

void RoadWidget::changeRoadFiltering(int filt) const {
    road->changeFilterType(ROAD, static_cast<FilteringType>(filt));
}

void RoadWidget::changeRockFiltering(int filt) const {
    road->changeFilterType(ROCK, static_cast<FilteringType>(filt));
}

void RoadWidget::changeCracksFiltering(int filt) const {
    road->changeFilterType(CRACKS, static_cast<FilteringType>(filt));
}

void RoadWidget::changeMixParameter(const int value) {
    mixParameter = static_cast<float>(value) / 100.0f;
    update();
}

void RoadWidget::setAutoSpeed(float speed) {
    autoSpeed = qBound(0.1f, speed, 1.0f);
}

void RoadWidget::setCameraHeight(float height) {
    viewPos.setY(qBound(0.1f, height, 2.0f));
    update();
}

void RoadWidget::toggleAutoMode(bool enabled) {
    autoMode = enabled;
    if (enabled) {
        startZ = viewPos.z();
        time = startZ / 90.0f;
    }
}

void RoadWidget::initializeGL() {
    initializeOpenGLFunctions();
    program = new QOpenGLShaderProgram(this);
    program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vertex_shader.vsh");
    program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fragment_shader.fsh");
    program->link();
    program->bind();
    initUniforms();
    road->initialize(program);
    program->release();
    glClearColor(backColor.redF(), backColor.greenF(), backColor.blueF(), 1.0f);
    glEnable(GL_DEPTH_TEST);
}

void RoadWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    const auto retina_scale = devicePixelRatio();
    glViewport(0, 0, width() * retina_scale, height() * retina_scale);
    QMatrix4x4 matrix;
    matrix.perspective(60.0f, (float)width() / height(), 0.1f, 1000.0f);
    matrix.lookAt(viewPos, viewPos + QVector3D(0.0f, 0.0f, 1.0f), QVector3D(0.0f, 1.0f, 0.0f));
    program->bind();
    setUniforms();
    road->setTotalMatrix(matrix);
    road->render();
    program->release();
    update();
}

void RoadWidget::resizeGL(const int w, const int h) {
    glViewport(0, 0, w, h);
}

void RoadWidget::wheelEvent(QWheelEvent* event) {
    const float speed = 0.5f;
    const auto numDegrees = event->angleDelta().y();
    const auto newZ = viewPos.z() + (numDegrees > 0 ? speed : -speed);
    viewPos.setZ(qBound(0.0f, newZ, 90.0f));
    viewPos.setX(qBound(-0.5f, viewPos.x(), 0.5f));
    update();
}

void RoadWidget::mouseMoveEvent(QMouseEvent* event) {
    if (event->buttons() & Qt::LeftButton) {
        const auto delta = event->pos() - lastMousePos;
        const float newX = viewPos.x() + delta.x() * 0.01f;
        viewPos.setX(qBound(-0.5f, newX, 0.5f));
        update();
    }
    lastMousePos = event->pos();
}

void RoadWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        lastMousePos = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void RoadWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        setCursor(Qt::ArrowCursor);
    }
}

void RoadWidget::updateCamera() {
    if (autoMode) {
        time += autoSpeed * 0.0001f;
        const float t = fmod(time, 2.0f);
        float newZ;
        if (t < 1.0f) {
            newZ = t * 90.0f;
        } else {
            newZ = (2.0f - t) * 90.0f;
        }
        viewPos.setZ(qBound(0.0f, newZ, 90.0f));
        viewPos.setX(qBound(-2.5f, viewPos.x(), 2.5f));
    }
}

#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QVector3D>
#include <QPoint>
#include <vector>

class Camera {
public:
    Camera() {
        distance = 10.0f;
        yaw = 0.0f;
        pitch = 45.0f;
        target = QVector3D(0.0f, 0.0f, 0.0f);
        up = QVector3D(0.0f, 1.0f, 0.0f);
        updatePosition();
    }

    void updatePosition() {
        float radYaw = qDegreesToRadians(yaw);
        float radPitch = qDegreesToRadians(pitch);
        float x = distance * cos(radPitch) * sin(radYaw);
        float y = distance * sin(radPitch);
        float z = distance * cos(radPitch) * cos(radYaw);
        position = target + QVector3D(x, y, z);
    }

    void setDistance(float dist) {
        distance = qBound(2.0f, dist, 15.0f);
        updatePosition();
    }

    void setYaw(float y) {
        yaw = y;
        updatePosition();
    }

    void setPitch(float p) {
        pitch = qBound(15.0f, p, 80.0f);
        updatePosition();
    }

    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }
    float getDistance() const { return distance; }

    QMatrix4x4 getViewMatrix() const {
        QMatrix4x4 view;
        view.lookAt(position, target, up);
        return view;
    }

    QVector3D getPosition() const {
        return position;
    }

    void setTarget(const QVector3D& newTarget) { target = newTarget; updatePosition(); }
    QVector3D getTarget() const { return target; }

    QVector3D getViewDirection() const {
        return (target - position).normalized();
    }

private:
    QVector3D position;
    QVector3D target;
    QVector3D up;
    float distance;
    float yaw;
    float pitch;
};

class BilliardWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    BilliardWidget(QWidget* parent = nullptr);
    ~BilliardWidget();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    struct Ball {
        std::vector<QVector3D> vertices;
        std::vector<QVector3D> normals;
        std::vector<GLuint> indices;
        QOpenGLBuffer vbo;
        QOpenGLBuffer nbo;
        QOpenGLBuffer ibo;
        QVector3D color;
        QVector3D position;
        QVector3D velocity;
        bool moving;
        bool visible;

        Ball() : vbo(QOpenGLBuffer::VertexBuffer), nbo(QOpenGLBuffer::VertexBuffer), ibo(QOpenGLBuffer::IndexBuffer), moving(false), visible(true) {}
    };

    struct Shadow {
        std::vector<QVector3D> vertices;
        std::vector<QVector3D> normals;
        std::vector<GLuint> indices;
        QOpenGLBuffer vbo;
        QOpenGLBuffer nbo;
        QOpenGLBuffer ibo;
        Shadow() : vbo(QOpenGLBuffer::VertexBuffer), nbo(QOpenGLBuffer::VertexBuffer), ibo(QOpenGLBuffer::IndexBuffer) {}
    };

    struct Geometry {
        std::vector<QVector3D> vertices;
        std::vector<QVector3D> normals;
        std::vector<GLuint> indices;
        QOpenGLBuffer vbo;
        QOpenGLBuffer nbo;
        QOpenGLBuffer ibo;

        Geometry() : vbo(QOpenGLBuffer::VertexBuffer), nbo(QOpenGLBuffer::VertexBuffer), ibo(QOpenGLBuffer::IndexBuffer) {}
    };

    struct Cue {
        std::vector<QVector3D> vertices;
        std::vector<QVector3D> normals;
        std::vector<GLuint> indices;
        QOpenGLBuffer vbo;
        QOpenGLBuffer nbo;
        QOpenGLBuffer ibo;
        QVector3D position;
        QVector3D direction;

        Cue() : vbo(QOpenGLBuffer::VertexBuffer), nbo(QOpenGLBuffer::VertexBuffer), ibo(QOpenGLBuffer::IndexBuffer) {}
    };

    void generateCue(Cue& cue, float baseRadius, float topRadius, float height, int sides);
    Cue gameCue;
    float cueBaseRadius = 0.2f;
    float cueTopRadius = 0.4f;
    float cueHeight = 4.5f;

    QVector3D strikeDirectionVector;
    float strikeForceDisplay;
    bool isStrikeDirectionVisible;

    void initShaders();
    void generateSphere(Ball& ball, float radius, int slices, int stacks);
    void generateTableBorders();
    void generatePocketSquares();
    void generateTableLegs();
    void generateShadow(Shadow& shadow, float radius, int segments);
    void updatePhysics();
    void handleCollisions();
    bool checkPockets(Ball& ball);
    QVector3D screenToWorld(QPoint pos);

    QOpenGLShaderProgram objectProgram;
    std::vector<Ball> balls;
    std::vector<QVector3D> tableVertices;
    std::vector<QVector3D> tableNormals;
    std::vector<GLuint> tableIndices;
    QOpenGLBuffer tableVbo;
    QOpenGLBuffer tableNbo;
    QOpenGLBuffer tableIbo;
    Geometry tableBorders;
    Geometry tableLegs;
    Camera camera;
    Geometry pocketSquare;
    std::vector<Shadow> shadows;
    QVector3D lightDir = QVector3D(0.0f, -1.0f, 0.0f);
    QVector3D lightPos = QVector3D(0.0f, 5.0f, 0.0f);
    bool directedLightEnabled = true;
    bool isDragging = false;
    QPoint lastMousePos;
    QPoint dragStartPos;
    QPoint dragEndPos;
    float tableWidth = 8.91f;
    float tableHeight = 4.46f;
    const float ballRadius = 0.1f;
    const float damping = 0.99f;
    const float pocketRadius = 0.15f;
    float strikeForce = 0.0f;
    float cameraSpeed = 0.1f;
    const float minDistance = 2.0f;
    const float maxDistance = 15.0f;

public slots:
    void switchDirectedLight(bool enabled);
};

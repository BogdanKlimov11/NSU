#include "billiard_widget.h"

BilliardWidget::BilliardWidget(QWidget* parent) : QOpenGLWidget(parent), tableVbo(QOpenGLBuffer::VertexBuffer), tableNbo(QOpenGLBuffer::VertexBuffer), tableIbo(QOpenGLBuffer::IndexBuffer) {
    float w = 8.91f, h = 4.46f;
    tableVertices = {
        QVector3D(-w/2, 0.0f, -h/2),
        QVector3D(w/2, 0.0f, -h/2),
        QVector3D(-w/2, 0.0f, h/2),
        QVector3D(w/2, 0.0f, h/2)
    };
    tableNormals = {
        QVector3D(0.0f, 1.0f, 0.0f),
        QVector3D(0.0f, 1.0f, 0.0f),
        QVector3D(0.0f, 1.0f, 0.0f),
        QVector3D(0.0f, 1.0f, 0.0f)
    };
    tableIndices = {0, 1, 2, 3};

    setFocusPolicy(Qt::StrongFocus);
}

BilliardWidget::~BilliardWidget() {
    makeCurrent();
    for (auto& ball : balls) {
        ball.vbo.destroy();
        ball.nbo.destroy();
        ball.ibo.destroy();
    }
    for (auto& shadow : shadows) {
        shadow.vbo.destroy();
        shadow.nbo.destroy();
        shadow.ibo.destroy();
    }
    tableVbo.destroy();
    tableNbo.destroy();
    tableIbo.destroy();
    tableBorders.vbo.destroy();
    tableBorders.nbo.destroy();
    tableBorders.ibo.destroy();
    tableLegs.vbo.destroy();
    tableLegs.nbo.destroy();
    tableLegs.ibo.destroy();
    objectProgram.release();
    doneCurrent();
}

void BilliardWidget::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    initShaders();

    tableVbo.create();
    tableVbo.bind();
    tableVbo.allocate(tableVertices.data(), tableVertices.size() * sizeof(QVector3D));
    tableVbo.release();

    tableNbo.create();
    tableNbo.bind();
    tableNbo.allocate(tableNormals.data(), tableNormals.size() * sizeof(QVector3D));
    tableNbo.release();

    tableIbo.create();
    tableIbo.bind();
    tableIbo.allocate(tableIndices.data(), tableIndices.size() * sizeof(GLuint));
    tableIbo.release();

    generateTableBorders();
    tableBorders.vbo.create();
    tableBorders.vbo.bind();
    tableBorders.vbo.allocate(tableBorders.vertices.data(), tableBorders.vertices.size() * sizeof(QVector3D));
    tableBorders.vbo.release();

    tableBorders.nbo.create();
    tableBorders.nbo.bind();
    tableBorders.nbo.allocate(tableBorders.normals.data(), tableBorders.normals.size() * sizeof(QVector3D));
    tableBorders.nbo.release();

    tableBorders.ibo.create();
    tableBorders.ibo.bind();
    tableBorders.ibo.allocate(tableBorders.indices.data(), tableBorders.indices.size() * sizeof(GLuint));
    tableBorders.ibo.release();

    generatePocketSquares();
    pocketSquare.vbo.create();
    pocketSquare.vbo.bind();
    pocketSquare.vbo.allocate(pocketSquare.vertices.data(), pocketSquare.vertices.size() * sizeof(QVector3D));
    pocketSquare.vbo.release();

    pocketSquare.nbo.create();
    pocketSquare.nbo.bind();
    pocketSquare.nbo.allocate(pocketSquare.normals.data(), pocketSquare.normals.size() * sizeof(QVector3D));
    pocketSquare.nbo.release();

    pocketSquare.ibo.create();
    pocketSquare.ibo.bind();
    pocketSquare.ibo.allocate(pocketSquare.indices.data(), pocketSquare.indices.size() * sizeof(GLuint));
    pocketSquare.ibo.release();

    generateTableLegs();
    tableLegs.vbo.create();
    tableLegs.vbo.bind();
    tableLegs.vbo.allocate(tableLegs.vertices.data(), tableLegs.vertices.size() * sizeof(QVector3D));
    tableLegs.vbo.release();

    tableLegs.nbo.create();
    tableLegs.nbo.bind();
    tableLegs.nbo.allocate(tableLegs.normals.data(), tableLegs.normals.size() * sizeof(QVector3D));
    tableLegs.nbo.release();

    tableLegs.ibo.create();
    tableLegs.ibo.bind();
    tableLegs.ibo.allocate(tableLegs.indices.data(), tableLegs.indices.size() * sizeof(GLuint));
    tableLegs.ibo.release();

    generateCue(gameCue, cueBaseRadius, cueTopRadius, cueHeight, 16);

    balls.resize(16);
    balls[0].color = QVector3D(1.0f, 1.0f, 1.0f);
    balls[0].position = QVector3D(-3.0f, 0.1f, 0.0f);

    float ballRadius = 0.1f;
    float xOffset = 2.0f, zOffset = 0.0f;
    int index = 1;

    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < (5 - row); ++col) {
            float z = zOffset + (col - (5 - row - 1) / 2.0f) * 2 * ballRadius;
            float x = xOffset - row * 2 * ballRadius * 0.866;
            balls[index].position = QVector3D(x, 0.1f, z);
            switch (index % 7) {
            case 1: balls[index].color = QVector3D(1.0f, 0.0f, 0.0f); break;
            case 2: balls[index].color = QVector3D(0.0f, 0.0f, 1.0f); break;
            case 3: balls[index].color = QVector3D(0.0f, 1.0f, 0.0f); break;
            case 4: balls[index].color = QVector3D(1.0f, 1.0f, 0.0f); break;
            case 5: balls[index].color = QVector3D(1.0f, 0.0f, 1.0f); break;
            case 6: balls[index].color = QVector3D(0.0f, 1.0f, 1.0f); break;
            case 0: balls[index].color = QVector3D(0.5f, 0.0f, 0.0f); break;
            }
            ++index;
        }
    }

    for (auto& ball : balls) {
        generateSphere(ball, 0.1f, 16, 16);

        ball.vbo.create();
        ball.vbo.bind();
        ball.vbo.allocate(ball.vertices.data(), ball.vertices.size() * sizeof(QVector3D));
        ball.vbo.release();

        ball.nbo.create();
        ball.nbo.bind();
        ball.nbo.allocate(ball.normals.data(), ball.normals.size() * sizeof(QVector3D));
        ball.nbo.release();

        ball.ibo.create();
        ball.ibo.bind();
        ball.ibo.allocate(ball.indices.data(), ball.indices.size() * sizeof(GLuint));
        ball.ibo.release();
    }

    shadows.resize(16);
    for (auto& shadow : shadows) {
        generateShadow(shadow, 0.1f, 16);

        shadow.vbo.create();
        shadow.vbo.bind();
        shadow.vbo.allocate(shadow.vertices.data(), shadow.vertices.size() * sizeof(QVector3D));
        shadow.vbo.release();

        shadow.nbo.create();
        shadow.nbo.bind();
        shadow.nbo.allocate(shadow.normals.data(), shadow.normals.size() * sizeof(QVector3D));
        shadow.nbo.release();

        shadow.ibo.create();
        shadow.ibo.bind();
        shadow.ibo.allocate(shadow.indices.data(), shadow.indices.size() * sizeof(GLuint));
        shadow.ibo.release();
    }

    strikeDirectionVector = QVector3D(0.0f, 0.0f, 0.0f);
    strikeForceDisplay = 0.0f;
    isStrikeDirectionVisible = false;
}

void BilliardWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void BilliardWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    objectProgram.bind();

    QMatrix4x4 projection;
    projection.perspective(60.0f, width() / float(height()), 0.1f, 100.0f);
    QMatrix4x4 view = camera.getViewMatrix();
    QVector3D cameraPos = camera.getPosition();

    objectProgram.setUniformValue("projection", projection);
    objectProgram.setUniformValue("view", view);
    objectProgram.setUniformValue("lightDir", lightDir);
    objectProgram.setUniformValue("directedLightEnabled", directedLightEnabled);
    objectProgram.setUniformValue("cameraPos", cameraPos);

    updatePhysics();
    handleCollisions();

    QMatrix4x4 model;
    objectProgram.setUniformValue("model", model);
    objectProgram.setUniformValue("materialColor", QVector3D(0.0f, 0.4f, 0.1f));
    tableVbo.bind();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
    glEnableVertexAttribArray(0);
    tableNbo.bind();
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
    glEnableVertexAttribArray(1);
    tableIbo.bind();
    glDrawElements(GL_TRIANGLE_STRIP, tableIndices.size(), GL_UNSIGNED_INT, nullptr);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    tableVbo.release();
    tableNbo.release();
    tableIbo.release();

    objectProgram.setUniformValue("materialColor", QVector3D(0.4f, 0.2f, 0.05f));
    tableBorders.vbo.bind();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
    glEnableVertexAttribArray(0);
    tableBorders.nbo.bind();
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
    glEnableVertexAttribArray(1);
    tableBorders.ibo.bind();
    glDrawElements(GL_TRIANGLES, tableBorders.indices.size(), GL_UNSIGNED_INT, nullptr);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    tableBorders.vbo.release();
    tableBorders.nbo.release();
    tableBorders.ibo.release();

    objectProgram.setUniformValue("materialColor", QVector3D(0.2f, 0.1f, 0.05f));
    tableLegs.vbo.bind();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
    glEnableVertexAttribArray(0);
    tableLegs.nbo.bind();
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
    glEnableVertexAttribArray(1);
    tableLegs.ibo.bind();
    glDrawElements(GL_TRIANGLES, tableLegs.indices.size(), GL_UNSIGNED_INT, nullptr);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    tableLegs.vbo.release();
    tableLegs.nbo.release();
    tableLegs.ibo.release();

    objectProgram.setUniformValue("lightPos", lightPos);

    objectProgram.setUniformValue("materialColor", QVector3D(0.0f, 0.0f, 0.0f));
    pocketSquare.vbo.bind();
    pocketSquare.vbo.bind();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
    glEnableVertexAttribArray(0);
    pocketSquare.nbo.bind();
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
    glEnableVertexAttribArray(1);
    pocketSquare.ibo.bind();
    glDrawElements(GL_TRIANGLES, pocketSquare.indices.size(), GL_UNSIGNED_INT, nullptr);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    pocketSquare.vbo.release();
    pocketSquare.nbo.release();
    pocketSquare.ibo.release();

    for (size_t i = 0; i < balls.size(); ++i) {
        if (!balls[i].visible) continue;

        QVector3D ballPos = balls[i].position;
        float t = (0.001f - lightPos.y()) / (ballPos.y() - lightPos.y());
        float scaleX = (lightPos.y() - 0.001f) / (lightPos.y() - ballPos.y());
        float scaleZ = scaleX;

        float shadowX = lightPos.x() + (ballPos.x() - lightPos.x()) * t;
        float shadowZ = lightPos.z() + (ballPos.z() - lightPos.z()) * t;

        QMatrix4x4 shadowModel;
        shadowModel.translate(shadowX, 0.001f, shadowZ);
        shadowModel.scale(scaleX, 1.0f, scaleZ);

        objectProgram.setUniformValue("model", shadowModel);
        objectProgram.setUniformValue("materialColor", QVector3D(0.0f, 0.0f, 0.0f));

        shadows[i].vbo.bind();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        glEnableVertexAttribArray(0);

        shadows[i].nbo.bind();
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        glEnableVertexAttribArray(1);

        shadows[i].ibo.bind();
        glDrawElements(GL_TRIANGLES, shadows[i].indices.size(), GL_UNSIGNED_INT, nullptr);

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        shadows[i].vbo.release();
        shadows[i].nbo.release();
        shadows[i].ibo.release();
    }

    for (auto& ball : balls) {
        if (!ball.visible) continue;
        QMatrix4x4 ballModel;
        ballModel.translate(ball.position);
        objectProgram.setUniformValue("model", ballModel);
        objectProgram.setUniformValue("materialColor", ball.color);

        ball.vbo.bind();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        glEnableVertexAttribArray(0);

        ball.nbo.bind();
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        glEnableVertexAttribArray(1);

        ball.ibo.bind();
        glDrawElements(GL_TRIANGLES, ball.indices.size(), GL_UNSIGNED_INT, nullptr);

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        ball.vbo.release();
        ball.nbo.release();
        ball.ibo.release();
    }

    if (isStrikeDirectionVisible && !balls[0].moving) {
        QMatrix4x4 cueModel;
        float minDistance = 0.2f;
        float maxDistance = 0.7f;
        float cueDistance = minDistance + (maxDistance - minDistance) * strikeForceDisplay;
        QVector3D cueOffset = -strikeDirectionVector.normalized() * cueDistance;
        cueModel.translate(balls[0].position + cueOffset);

        QVector3D targetDirection = -strikeDirectionVector.normalized();
        QVector3D initialDirection(1.0f, 0.0f, 0.0f);
        QVector3D axis = QVector3D::crossProduct(initialDirection, targetDirection).normalized();
        float angle = acos(QVector3D::dotProduct(initialDirection, targetDirection));
        if (axis.lengthSquared() > 0.0f && !qIsNaN(angle)) {
            cueModel.rotate(qRadiansToDegrees(angle), axis);
        }

        cueModel.rotate(10.0f, QVector3D(0.0f, 0.0f, 1.0f));

        const float fixedLength = 0.5f;
        cueModel.scale(fixedLength, 0.1f, 0.1f);

        objectProgram.setUniformValue("model", cueModel);
        objectProgram.setUniformValue("materialColor", QVector3D(0.8f, 0.4f, 0.2f));

        gameCue.vbo.bind();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        glEnableVertexAttribArray(0);
        gameCue.nbo.bind();
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        glEnableVertexAttribArray(1);
        gameCue.ibo.bind();
        glDrawElements(GL_TRIANGLES, gameCue.indices.size(), GL_UNSIGNED_INT, nullptr);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        gameCue.vbo.release();
        gameCue.nbo.release();
        gameCue.ibo.release();
    }

    objectProgram.release();
    update();
}

void BilliardWidget::initShaders() {
    objectProgram.removeAllShaders();
    QOpenGLShader* vertexShader = new QOpenGLShader(QOpenGLShader::Vertex, this);
    vertexShader->compileSourceFile(":/vertex_shader.vsh");
    objectProgram.addShader(vertexShader);

    QOpenGLShader* fragmentShader = new QOpenGLShader(QOpenGLShader::Fragment, this);
    fragmentShader->compileSourceFile(":/fragment_shader.fsh");
    objectProgram.addShader(fragmentShader);

    objectProgram.bindAttributeLocation("position", 0);
    objectProgram.bindAttributeLocation("normal", 1);

    objectProgram.link();
    objectProgram.bind();

    objectProgram.setUniformValue("lightPos", lightPos);
}

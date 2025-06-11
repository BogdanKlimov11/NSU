#include <QMouseEvent>
#include <random>

#include "billiard_widget.h"

void BilliardWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton && !balls[0].moving) {
        isDragging = true;
        dragEndPos = event->pos();
        strikeForce = 0.0f;
        setFocus();
    }
    else {
        if (event->button() == Qt::LeftButton) {
            isDragging = true;
            lastMousePos = event->pos();
            setFocus();
        }
    }
}

void BilliardWidget::mouseMoveEvent(QMouseEvent* event) {
    if (isDragging) {
        if (event->buttons() & Qt::LeftButton) {
            QPoint delta = event->pos() - lastMousePos;
            float deltaYaw = delta.x() * 0.5f;
            float deltaPitch = delta.y() * 0.5f;
            camera.setYaw(camera.getYaw() - deltaYaw);
            camera.setPitch(camera.getPitch() + deltaPitch);
            lastMousePos = event->pos();
        }
        else {
            if (event->buttons() & Qt::RightButton && !balls[0].moving) {
                dragStartPos = event->pos();
                strikeForce = qMin((float)(dragEndPos - dragStartPos).manhattanLength() / 100.0f, 1.0f);
                QVector3D worldStart = screenToWorld(dragEndPos);
                QVector3D worldEnd = screenToWorld(event->pos());
                QVector3D rawDirection = (worldEnd - worldStart);

                QMatrix4x4 viewMatrix = camera.getViewMatrix();
                strikeDirectionVector = viewMatrix.inverted().mapVector(rawDirection);
                strikeDirectionVector.setY(0.0f);
                if (strikeDirectionVector.lengthSquared() > 0.0f) {
                    strikeDirectionVector.normalize();
                    isStrikeDirectionVisible = true;
                }
                strikeForceDisplay = strikeForce;
            }
        }
        update();
    }
}

void BilliardWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton && !balls[0].moving) {
        dragStartPos = event->pos();
        QVector3D worldStart = screenToWorld(dragEndPos);
        QVector3D worldEnd = screenToWorld(dragStartPos);
        QVector3D rawDirection = (worldEnd - worldStart);

        QMatrix4x4 viewMatrix = camera.getViewMatrix();
        QVector3D cameraDirection = viewMatrix.inverted().mapVector(rawDirection);
        cameraDirection.setY(0.0f);
        if (cameraDirection.lengthSquared() > 0.0f) {
            cameraDirection.normalize();
            balls[0].velocity = cameraDirection * strikeForce * 0.5f;
            balls[0].moving = true;
        }
        strikeForce = 0.0f;
        isDragging = false;
        isStrikeDirectionVisible = false;
        update();
    }
    else {
        if (event->button() == Qt::LeftButton) {
            isDragging = false;
        }
    }
}

void BilliardWidget::wheelEvent(QWheelEvent* event) {
    float delta = event->angleDelta().y() > 0 ? -0.5f : 0.5f;
    camera.setDistance(camera.getDistance() + delta);
    update();
}

void BilliardWidget::keyPressEvent(QKeyEvent *event) {
    if (!isDragging) {
        QVector3D viewDir = camera.getViewDirection();
        QVector3D rightDir = QVector3D::crossProduct(viewDir, QVector3D(0.0f, 1.0f, 0.0f)).normalized();
        QVector3D forwardDir = QVector3D(viewDir.x(), 0.0f, viewDir.z()).normalized();

        switch (event->key()) {
        case Qt::Key_Up:
            camera.setTarget(camera.getTarget() + forwardDir * cameraSpeed);
            break;
        case Qt::Key_Down:
            camera.setTarget(camera.getTarget() - forwardDir * cameraSpeed);
            break;
        case Qt::Key_Left:
            camera.setTarget(camera.getTarget() - rightDir * cameraSpeed);
            break;
        case Qt::Key_Right:
            camera.setTarget(camera.getTarget() + rightDir * cameraSpeed);
            break;
        case Qt::Key_Space:
            camera.setTarget(QVector3D(0.0f, 0.0f, 0.0f));
            camera.setDistance(10.0f);
            camera.setYaw(0.0f);
            camera.setPitch(45.0f);
            break;
        }
        float tableMinX = -tableWidth / 2.0f + 1.0f;
        float tableMaxX = tableWidth / 2.0f - 1.0f;
        float tableMinZ = -tableHeight / 2.0f + 1.0f;
        float tableMaxZ = tableHeight / 2.0f - 1.0f;
        QVector3D currentTarget = camera.getTarget();
        currentTarget.setX(qBound(tableMinX, currentTarget.x(), tableMaxX));
        currentTarget.setZ(qBound(tableMinZ, currentTarget.z(), tableMaxZ));
        camera.setTarget(currentTarget);
        update();
    }
}

void BilliardWidget::updatePhysics() {
    for (auto& ball : balls) {
        if (!ball.visible || !ball.moving) continue;
        ball.position += ball.velocity;
        ball.velocity *= damping;
        if (ball.velocity.lengthSquared() < 0.0001f) {
            ball.velocity = QVector3D(0.0f, 0.0f, 0.0f);
            ball.moving = false;
        }
    }
}

bool BilliardWidget::checkPockets(Ball& ball) {
    if (!ball.visible) return true;

    float ballRadius = 0.1f;
    QVector2D ballPos(ball.position.x(), ball.position.z());
    float pocketRadius = 0.15f;

    QVector2D pockets[] = {
        QVector2D(-tableWidth / 2.0f, -tableHeight / 2.0f),
        QVector2D(tableWidth / 2.0f, -tableHeight / 2.0f),
        QVector2D(-tableWidth / 2.0f, tableHeight / 2.0f),
        QVector2D(tableWidth / 2.0f, tableHeight / 2.0f),
        QVector2D(0.0f, -tableHeight / 2.0f),
        QVector2D(0.0f, tableHeight / 2.0f)
    };

    for (const auto& pocket : pockets) {
        float dist = (ballPos - pocket).length();
        if (dist < pocketRadius + ballRadius) {
            if (&ball == &balls[0]) {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> disX(-tableWidth / 2 + ballRadius, tableWidth / 2 - ballRadius);
                std::uniform_real_distribution<> disZ(-tableHeight / 2 + ballRadius, tableHeight / 2 - ballRadius);
                bool positionValid = false;
                int attempts = 0;
                const int maxAttempts = 100;
                while (!positionValid && attempts < maxAttempts) {
                    float newX = disX(gen);
                    float newZ = disZ(gen);
                    QVector3D newPos(newX, 0.1f, newZ);
                    bool collision = false;
                    for (const auto& otherBall : balls) {
                        if (&otherBall != &ball && otherBall.visible) {
                            QVector3D distVec = newPos - otherBall.position;
                            if (distVec.length() < 2.0f * ballRadius) {
                                collision = true;
                                break;
                            }
                        }
                    }
                    if (!collision) {
                        ball.position = newPos;
                        ball.velocity = QVector3D(0.0f, 0.0f, 0.0f);
                        ball.moving = false;
                        positionValid = true;
                    }
                    attempts++;
                }
            }
            else {
                ball.visible = false;
                ball.moving = false;
                ball.velocity = QVector3D(0.0f, 0.0f, 0.0f);
            }
            return true;
        }
    }
    return false;
}

QVector3D BilliardWidget::screenToWorld(QPoint pos) {
    float x = -(2.0f * pos.x() / width() - 1.0f) * (tableWidth / 2.0f);
    float z = -(2.0f * pos.y() / height() - 1.0f) * (tableHeight / 2.0f);
    return QVector3D(x, 0.1f, z);
}

void BilliardWidget::handleCollisions() {
    for (size_t i = 0; i < balls.size(); ++i) {
        if (!balls[i].visible) continue;

        checkPockets(balls[i]);

        for (size_t j = i + 1; j < balls.size(); ++j) {
            if (!balls[j].visible) continue;
            QVector3D dist = balls[j].position - balls[i].position;
            float distSqr = dist.lengthSquared();
            float minDist = 2.0f * ballRadius;
            if (distSqr < minDist * minDist && distSqr > 0.0f) {
                float distLen = sqrt(distSqr);
                QVector3D normal = dist / distLen;
                float overlap = minDist - distLen;

                balls[i].position -= normal * (overlap * 0.5f);
                balls[j].position += normal * (overlap * 0.5f);

                QVector3D v1 = balls[i].velocity;
                QVector3D v2 = balls[j].velocity;

                float v1n = QVector3D::dotProduct(v1, normal);
                float v2n = QVector3D::dotProduct(v2, normal);

                QVector3D v1t = v1 - normal * v1n;
                QVector3D v2t = v2 - normal * v2n;

                QVector3D v1n_new = normal * v2n;
                QVector3D v2n_new = normal * v1n;

                balls[i].velocity = v1t + v1n_new;
                balls[j].velocity = v2t + v2n_new;

                balls[i].moving = balls[i].velocity.lengthSquared() > 0.0001f;
                balls[j].moving = balls[j].velocity.lengthSquared() > 0.0001f;
            }
        }

        float borderThickness = 0.2f;
        QVector2D ballPos2D(balls[i].position.x(), balls[i].position.z());

        QVector2D pockets2D[] = {
            QVector2D(-tableWidth / 2.2f, -tableHeight / 2.2f),
            QVector2D(tableWidth / 2.2f, -tableHeight / 2.2f),
            QVector2D(-tableWidth / 2.2f, tableHeight / 2.2f),
            QVector2D(tableWidth / 2.2f, tableHeight / 2.2f),
            QVector2D(0.0f, -tableHeight / 2.2f),
            QVector2D(0.0f, tableHeight / 2.2f)
        };

        bool inPocket = false;
        for (const auto& pocket : pockets2D) {
            float distToPocket = (ballPos2D - pocket).length();
            if (distToPocket < pocketRadius + ballRadius) {
                inPocket = true;
                break;
            }
        }

        if (!inPocket) {
            if (balls[i].position.x() - ballRadius < -tableWidth / 2 + borderThickness) {
                balls[i].position.setX(-tableWidth / 2 + borderThickness + ballRadius);
                balls[i].velocity.setX(-balls[i].velocity.x() * 0.9f);
                balls[i].moving = true;
            }
            else {
                if (balls[i].position.x() + ballRadius > tableWidth / 2 - borderThickness) {
                    balls[i].position.setX(tableWidth / 2 - borderThickness - ballRadius);
                    balls[i].velocity.setX(-balls[i].velocity.x() * 0.9f);
                    balls[i].moving = true;
                }
            }
            if (balls[i].position.z() - ballRadius < -tableHeight / 2 + borderThickness) {
                balls[i].position.setZ(-tableHeight / 2 + borderThickness + ballRadius);
                balls[i].velocity.setZ(-balls[i].velocity.z() * 0.9f);
                balls[i].moving = true;
            }
            else {
                if (balls[i].position.z() + ballRadius > tableHeight / 2 - borderThickness) {
                    balls[i].position.setZ(tableHeight / 2 - borderThickness - ballRadius);
                    balls[i].velocity.setZ(-balls[i].velocity.z() * 0.9f);
                    balls[i].moving = true;
                }
            }
        }
    }
}

void BilliardWidget::switchDirectedLight(bool enabled) {
    directedLightEnabled = enabled;
    update();
}

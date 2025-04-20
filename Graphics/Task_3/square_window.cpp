#include <QColorDialog>
#include <QScreen>
#include <QtMath>

#include "square_window.h"

SquareWindow::SquareWindow(QWidget *parent) : QOpenGLWidget(parent) {
    n = 10;
}

void SquareWindow::initializeGL() {
    initializeOpenGLFunctions();
    program_ = std::make_unique<QOpenGLShaderProgram>(this);
    program_->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fragment_shader.fsh");
    program_->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vertex_shader.vsh");
    program_->link();

    posAttr_ = program_->attributeLocation("posAttr");
    normAttr_ = program_->attributeLocation("normAttr");
    Q_ASSERT(posAttr_ != -1);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    timer.start(30, this);

    init_cube(1.5f, n);
}

void SquareWindow::paintGL() {
    const auto retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program_->bind();

    program_->setUniformValue("projection_matrix", projection_matrix);
    program_->setUniformValue("viewPos", QVector3D(0.0, 0.0, 0));
    program_->setUniformValue("lightPos", QVector3D(light_x_param, light_y_param, light_z_param));
    program_->setUniformValue("objectColor", cube_color);
    program_->setUniformValue("lightColor", lightColor);
    program_->setUniformValue("morph_param", morph_param);
    program_->setUniformValue("lightType", lightType);
    program_->setUniformValue("lightIntensity", lightIntensity);
    program_->setUniformValue("lightAttenuation", lightAttenuation);
    program_->setUniformValue("lightDirection", lightDirection);
    program_->setUniformValue("lightCutoff", lightCutoff);

    program_->setUniformValue("materialAmbient", materialAmbient);
    program_->setUniformValue("materialDiffuse", materialDiffuse);
    program_->setUniformValue("materialSpecular", materialSpecular);
    program_->setUniformValue("globalAmbient", globalAmbient);

    vertexBuffer.bind();
    indexBuffer.bind();
    program_->setAttributeBuffer(posAttr_, GL_FLOAT, 0, 3, sizeof(VertexData));
    program_->enableAttributeArray(posAttr_);
    program_->setAttributeBuffer(normAttr_, GL_FLOAT, sizeof(QVector3D), 3, sizeof(VertexData));
    program_->enableAttributeArray(normAttr_);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; ++j) {
            QMatrix4x4 view_matrix;
            view_matrix.setToIdentity();
            view_matrix.translate(-2.5 + i * 2.5, -2.5 + j * 2.5, -8);
            view_matrix.rotate(100.0 * frame_ / 59.0, rotationAxis);

            program_->setUniformValue("model", view_matrix);
            program_->setUniformValue("view_matrix", view_matrix);
            program_->setUniformValue("norm_matrix", view_matrix.normalMatrix());

            glDrawElements(GL_TRIANGLES, indexBuffer.size(), GL_UNSIGNED_INT, nullptr);
        }
    }

    program_->disableAttributeArray(posAttr_);
    vertexBuffer.release();
    indexBuffer.release();
    program_->release();
    ++frame_;
}

void SquareWindow::resizeGL(const int w, const int h) {
    const auto aspect = w / static_cast<double>(h);
    projection_matrix.setToIdentity();
    projection_matrix.perspective(60.0f, aspect, 0.01f, 100.0f);
}

void SquareWindow::change_morph_param(float value) {
    morph_param = value / 1000;
    update();
}

void SquareWindow::change_light_x_param(float value) {
    if (lightType == 0) {
        lightDirection.setX(value);
    } else {
        light_x_param = value;
    }
    update();
}

void SquareWindow::change_light_y_param(float value) {
    if (lightType == 0) {
        lightDirection.setY(value);
    } else {
        light_y_param = value;
    }
    update();
}

void SquareWindow::change_light_z_param(float value) {
    if (lightType == 0) {
        lightDirection.setZ(value);
    } else {
        light_z_param = value;
    }
    update();
}

void SquareWindow::mousePressEvent(QMouseEvent *e) {
    mousePressPosition = QVector2D(e->position());
}

void SquareWindow::mouseReleaseEvent(QMouseEvent *e) {
    const auto diff = QVector2D(e->position()) - mousePressPosition;
    rotationAxis = QVector3D(diff.y(), diff.x(), 0.0).normalized();
}

void SquareWindow::keyPressEvent(QKeyEvent *e) {
    if (e->key() == Qt::Key::Key_Plus) {
        const auto chosen_color = QColorDialog::getColor();
        cube_color = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    } else if (e->key() == Qt::Key::Key_Minus) {
        const auto chosen_color = QColorDialog::getColor();
        lightColor = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    }
}

void SquareWindow::timerEvent(QTimerEvent * /*event*/) { update(); }

void SquareWindow::init_cube(const float width, const int N) {
    auto width_div_2 = width / 2.0f;
    auto step = width / float(N - 1);

    std::vector<VertexData> vertexes;
    vertexes.reserve(6 * pow(N, 2));
    for (auto z = -width_div_2; z <= width_div_2; z += width) {
        for (auto j = 0; j < N; j++) {
            for (auto i = 0; i < N; i++) {
                vertexes.emplace_back(
                    VertexData(QVector3D(-z + i * step * z / width_div_2, -width_div_2 + j * step, z), QVector3D(0.0, 0.0, z / width_div_2)));
            }
        }
    }
    for (auto x = -width_div_2; x <= width_div_2; x += width) {
        for (auto k = 0; k < N; k++) {
            for (auto j = 0; j < N; j++) {
                vertexes.emplace_back(
                    VertexData(QVector3D(x, -width_div_2 + j * step, -x + x * k * step / width_div_2), QVector3D(x / width_div_2, 0.0, 0.0)));
            }
        }
    }
    for (auto y = -width_div_2; y <= width_div_2; y += width) {
        for (auto i = 0; i < N; i++) {
            for (auto k = 0; k < N; k++) {
                vertexes.emplace_back(
                    VertexData(QVector3D(-width_div_2 + i * step, y, -y + y * k * step / width_div_2), QVector3D(0.0, y / width_div_2, 0.0)));
            }
        }
    }

    std::vector<GLuint> indexes;
    int vertex_count = 36 * pow(N - 1, 2);
    indexes.reserve(vertex_count);
    for (int i = 0; i < 6 * N * N; i += N * N) {
        for (int j = 0; j < (N - 1) * (N - 1); j += N) {
            for (int k = 0; k < (N - 1); k++) {
                indexes.emplace_back(i + j + k + N);
                indexes.emplace_back(i + j + k + 0);
                indexes.emplace_back(i + j + k + N + 1);
                indexes.emplace_back(i + j + k + N + 1);
                indexes.emplace_back(i + j + k + 0);
                indexes.emplace_back(i + j + k + 1);
            }
        }
    }

    vertexBuffer.create();
    vertexBuffer.bind();
    vertexBuffer.allocate(vertexes.data(), static_cast<int>(vertexes.size() * sizeof(VertexData)));

    indexBuffer.create();
    indexBuffer.bind();
    indexBuffer.allocate(indexes.data(), static_cast<int>(indexes.size() * sizeof(GLuint)));
}

void SquareWindow::change_cube_color() {
    const auto chosen_color = QColorDialog::getColor();
    cube_color = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    update();
}

void SquareWindow::change_light_color() {
    const auto chosen_color = QColorDialog::getColor();
    lightColor = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    update();
}

void SquareWindow::changeN(int new_n) {
    n = new_n;
    init_cube(1.5f, n);
    update();
}

void SquareWindow::setLightType(int type) {
    lightType = type;
    if (lightType == 0) {
        lightIntensity = 1.0f;
        lightAttenuation = 0.0f;
        lightCutoff = 0.0f;
        lightDirection = QVector3D(1.0f, 1.0f, 1.0f).normalized();
        light_x_param = lightDirection.x();
        light_y_param = lightDirection.y();
        light_z_param = lightDirection.z();
        lightColor = QVector3D(1.0f, 1.0f, 0.8f);
    } else if (lightType == 1) {
        lightCutoff = 0.0f;
        lightDirection = QVector3D(0.0f, 0.0f, 0.0f);
        light_x_param = 1.0f;
        light_y_param = 1.0f;
        light_z_param = 1.0f;
        lightColor = QVector3D(0.6f, 0.8f, 1.0f);
    } else if (lightType == 2) {
        lightDirection = QVector3D(1.0f, 1.0f, 1.0f).normalized();
        light_x_param = 1.0f;
        light_y_param = 1.0f;
        light_z_param = 1.0f;
        lightColor = QVector3D(0.6f, 0.8f, 1.0f);
    }
    update();
}

void SquareWindow::setLightIntensity(float intensity) {
    lightIntensity = intensity;
    update();
}

void SquareWindow::setLightAttenuation(float attenuation) {
    lightAttenuation = attenuation / 100.0f;
    update();
}

void SquareWindow::setLightCutoff(float cutoff) {
    lightCutoff = cutoff;
    update();
}

void SquareWindow::setLightDirection(QVector3D direction) {
    lightDirection = direction;
    update();
}

void SquareWindow::setMaterialAmbient() {
    const auto chosen_color = QColorDialog::getColor();
    materialAmbient = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    update();
}

void SquareWindow::setMaterialDiffuse() {
    const auto chosen_color = QColorDialog::getColor();
    materialDiffuse = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    update();
}

void SquareWindow::setMaterialSpecular() {
    const auto chosen_color = QColorDialog::getColor();
    materialSpecular = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    update();
}

void SquareWindow::setGlobalAmbient() {
    const auto chosen_color = QColorDialog::getColor();
    globalAmbient = QVector3D(chosen_color.red() / 255.0f, chosen_color.green() / 255.0f, chosen_color.blue() / 255.0f);
    update();
}

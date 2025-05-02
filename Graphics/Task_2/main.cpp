#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QApplication>
#include <QSlider>
#include <QLineEdit>
#include <QIntValidator>
#include <QDoubleValidator>
#include <QLabel>

#include "transform_widget.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);

    auto sceneWindow = new QWidget;
    auto sceneWidget = new SceneWidget(sceneWindow);
    auto sceneLayout = new QVBoxLayout(sceneWindow);
    sceneLayout->addWidget(sceneWidget);
    sceneWindow->setWindowTitle("Display");
    sceneWidget->setMinimumSize(640, 640);
    sceneWindow->resize(640, 640);

    auto controlWindow = new QWidget;

    auto morphingSlider = new QSlider(Qt::Horizontal, controlWindow);
    auto morphingEdit = new QLineEdit(controlWindow);
    auto splitParameterSlider = new QSlider(Qt::Horizontal, controlWindow);
    auto splitParameterEdit = new QLineEdit(controlWindow);
    auto rotationSliderX = new QSlider(Qt::Horizontal, controlWindow);
    auto rotationSliderY = new QSlider(Qt::Horizontal, controlWindow);
    auto rotationSliderZ = new QSlider(Qt::Horizontal, controlWindow);
    auto rotationEditX = new QLineEdit(controlWindow);
    auto rotationEditY = new QLineEdit(controlWindow);
    auto rotationEditZ = new QLineEdit(controlWindow);

    rotationSliderX->setRange(0, 360);
    rotationSliderY->setRange(0, 360);
    rotationSliderZ->setRange(0, 360);
    morphingSlider->setRange(0, 100);
    splitParameterSlider->setRange(1, 99);

    rotationEditX->setAlignment(Qt::AlignHCenter);
    rotationEditY->setAlignment(Qt::AlignHCenter);
    rotationEditZ->setAlignment(Qt::AlignHCenter);
    rotationEditX->setValidator(new QIntValidator(0, 360, rotationEditX));
    rotationEditY->setValidator(new QIntValidator(0, 360, rotationEditY));
    rotationEditZ->setValidator(new QIntValidator(0, 360, rotationEditZ));
    rotationEditX->setText("0");
    rotationEditY->setText("0");
    rotationEditZ->setText("0");

    morphingEdit->setAlignment(Qt::AlignHCenter);
    morphingEdit->setValidator(new QDoubleValidator(0.0, 1.0, 2, morphingEdit));
    morphingEdit->setText("0.00");
    morphingSlider->setValue(0);

    splitParameterEdit->setAlignment(Qt::AlignHCenter);
    splitParameterEdit->setValidator(new QIntValidator(1, 99, splitParameterEdit));
    splitParameterEdit->setText("10");
    splitParameterSlider->setValue(10);

    QObject::connect(rotationSliderX, &QAbstractSlider::valueChanged, [=](int value) {
        sceneWidget->setRotationX(float(value));
        rotationEditX->setText(QString::number(value));
        sceneWidget->update();
    });
    QObject::connect(rotationSliderY, &QAbstractSlider::valueChanged, [=](int value) {
        sceneWidget->setRotationY(float(value));
        rotationEditY->setText(QString::number(value));
        sceneWidget->update();
    });
    QObject::connect(rotationSliderZ, &QAbstractSlider::valueChanged, [=](int value) {
        sceneWidget->setRotationZ(float(value));
        rotationEditZ->setText(QString::number(value));
        sceneWidget->update();
    });
    QObject::connect(morphingSlider, &QAbstractSlider::valueChanged, [=](int value) {
        float morphValue = float(value) / 100.0f;
        sceneWidget->setMorphingParameter(morphValue);
        morphingEdit->setText(QString::number(morphValue, 'f', 2));
        sceneWidget->update();
    });
    QObject::connect(splitParameterSlider, &QAbstractSlider::valueChanged, [=](int value) {
        sceneWidget->setSplitParameter(size_t(value));
        splitParameterEdit->setText(QString::number(value));
        sceneWidget->update();
    });

    QObject::connect(rotationEditX, &QLineEdit::textChanged, [=](const QString &text) {
        int value = text.toInt();
        sceneWidget->setRotationX(float(value));
        rotationSliderX->setValue(value);
        sceneWidget->update();
    });
    QObject::connect(rotationEditY, &QLineEdit::textChanged, [=](const QString &text) {
        int value = text.toInt();
        sceneWidget->setRotationY(float(value));
        rotationSliderY->setValue(value);
        sceneWidget->update();
    });
    QObject::connect(rotationEditZ, &QLineEdit::textChanged, [=](const QString &text) {
        int value = text.toInt();
        sceneWidget->setRotationZ(float(value));
        rotationSliderZ->setValue(value);
        sceneWidget->update();
    });
    QObject::connect(morphingEdit, &QLineEdit::textChanged, [=](const QString &text) {
        float value = text.toFloat();
        if (value >= 0.0f && value <= 1.0f) {
            sceneWidget->setMorphingParameter(value);
            morphingSlider->setValue(int(value * 100));
            sceneWidget->update();
        }
    });
    QObject::connect(splitParameterEdit, &QLineEdit::textChanged, [=](const QString &text) {
        int value = text.toInt();
        if (value >= 1 && value <= 99) {
            sceneWidget->setSplitParameter(size_t(value));
            splitParameterSlider->setValue(value);
            sceneWidget->update();
        }
    });

    auto rotationLabelX = new QLabel("X axis:", controlWindow);
    auto rotationLabelY = new QLabel("Y axis:", controlWindow);
    auto rotationLabelZ = new QLabel("Z axis:", controlWindow);
    auto morphingLabel = new QLabel("Morphing:", controlWindow);
    auto splitLabel = new QLabel("Split option:", controlWindow);

    auto control_layout = new QVBoxLayout(controlWindow);
    control_layout->addWidget(rotationLabelX);
    control_layout->addWidget(rotationSliderX);
    control_layout->addWidget(rotationEditX);
    control_layout->addWidget(rotationLabelY);
    control_layout->addWidget(rotationSliderY);
    control_layout->addWidget(rotationEditY);
    control_layout->addWidget(rotationLabelZ);
    control_layout->addWidget(rotationSliderZ);
    control_layout->addWidget(rotationEditZ);
    control_layout->addWidget(morphingLabel);
    control_layout->addWidget(morphingSlider);
    control_layout->addWidget(morphingEdit);
    control_layout->addWidget(splitLabel);
    control_layout->addWidget(splitParameterSlider);
    control_layout->addWidget(splitParameterEdit);
    control_layout->addStretch();

    controlWindow->setWindowTitle("Control panel");
    controlWindow->resize(300, 420);

    sceneWindow->show();
    controlWindow->show();

    return a.exec();
}

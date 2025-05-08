#include <QGroupBox>
#include <QHBoxLayout>
#include <QGridLayout>

#include "light_widget.h"

template <typename T, typename = std::enable_if_t<std::is_pointer<T>::value>>
using main = T;

auto slider(int minValue, int maxValue, int tickValue) {
    auto *const slider = main<QSlider*>(new QSlider(Qt::Orientation::Horizontal));
    slider->setRange(minValue, maxValue);
    slider->setSingleStep(tickValue);
    slider->setTickPosition(QSlider::TicksRight);
    slider->setMinimumWidth(150);
    slider->setValue(0);
    return slider;
}

LightWidget::LightWidget(MainWindow *mainWindow, QWidget *parent) : QWidget(parent) {
    auto *const mainLayout = main<QGridLayout *>(new QGridLayout);
    mainLayout->setAlignment(Qt::AlignCenter);

    auto *leftColumn = new QVBoxLayout;
    leftColumn->setAlignment(Qt::AlignTop);

    auto *fpsGroup = new QGroupBox("Frames per second");
    auto *fpsLayout = new QHBoxLayout;
    fpsLabel = new QLabel("FPS: 0");
    fpsLayout->addWidget(fpsLabel);
    fpsLayout->setAlignment(Qt::AlignCenter);
    fpsGroup->setLayout(fpsLayout);
    leftColumn->addWidget(fpsGroup);

    auto *lightPosGroup = new QGroupBox("Light position");
    auto *lightPosLayout = new QGridLayout;
    sunSliderX = slider(-100, 100, 10);
    sunSliderY = slider(-100, 100, 10);
    sunSliderZ = slider(0, 100, 10);
    sunSliderX->setValue(0);
    sunSliderY->setValue(0);
    sunSliderZ->setValue(0);
    lightXLabel = new QLabel("X axis:");
    lightYLabel = new QLabel("Y axis:");
    lightZLabel = new QLabel("Z axis:");
    lightXValueLabel = new QLabel("0");
    lightYValueLabel = new QLabel("0");
    lightZValueLabel = new QLabel("2.0");
    lightXValueLabel->setFixedWidth(40);
    lightYValueLabel->setFixedWidth(40);
    lightZValueLabel->setFixedWidth(40);
    lightPosLayout->addWidget(lightXLabel, 0, 0);
    lightPosLayout->addWidget(sunSliderX, 0, 1);
    lightPosLayout->addWidget(lightXValueLabel, 0, 2);
    lightPosLayout->addWidget(lightYLabel, 1, 0);
    lightPosLayout->addWidget(sunSliderY, 1, 1);
    lightPosLayout->addWidget(lightYValueLabel, 1, 2);
    lightPosLayout->addWidget(lightZLabel, 2, 0);
    lightPosLayout->addWidget(sunSliderZ, 2, 1);
    lightPosLayout->addWidget(lightZValueLabel, 2, 2);
    lightPosGroup->setLayout(lightPosLayout);
    leftColumn->addWidget(lightPosGroup);

    auto *colorGroup = new QGroupBox("Colors");
    auto *colorLayout = new QHBoxLayout;
    QPushButton *changeColorCubeButton = new QPushButton(tr("Cube color"));
    changeColorCubeButton->setFocusPolicy(Qt::NoFocus);
    QPushButton *changeColorLightButton = new QPushButton(tr("Light color"));
    changeColorLightButton->setFocusPolicy(Qt::NoFocus);
    colorLayout->addWidget(changeColorCubeButton);
    colorLayout->addWidget(changeColorLightButton);
    colorLayout->setAlignment(Qt::AlignCenter);
    colorGroup->setLayout(colorLayout);
    leftColumn->addWidget(colorGroup);

    auto *lightTypeGroup = new QGroupBox("Light type");
    auto *lightTypeLayout = new QHBoxLayout;
    directionalLightRadio = new QRadioButton("Directional light");
    pointLightRadio = new QRadioButton("Point light");
    spotlightRadio = new QRadioButton("Spotlight");
    directionalLightRadio->setChecked(true);
    lightTypeLayout->addWidget(directionalLightRadio);
    lightTypeLayout->addWidget(pointLightRadio);
    lightTypeLayout->addWidget(spotlightRadio);
    lightTypeLayout->setAlignment(Qt::AlignCenter);
    lightTypeGroup->setLayout(lightTypeLayout);
    leftColumn->addWidget(lightTypeGroup);

    auto *subdivisionGroup = new QGroupBox("Cube splitting");
    auto *subdivisionLayout = new QHBoxLayout;
    subdivisionSpinBox = new QSpinBox;
    subdivisionSpinBox->setRange(1, 99);
    subdivisionSpinBox->setValue(10);
    subdivisionSpinBox->setSingleStep(1);
    subdivisionLayout->addWidget(new QLabel("Splitting parameter (from 1 to 99):"));
    subdivisionLayout->addWidget(subdivisionSpinBox);
    subdivisionGroup->setLayout(subdivisionLayout);
    leftColumn->addWidget(subdivisionGroup);

    auto *rightColumn = new QVBoxLayout;
    rightColumn->setAlignment(Qt::AlignTop);

    auto *rotationAxisGroup = new QGroupBox("Rotation axis and morphing");
    auto *rotationAxisLayout = new QGridLayout;
    rotationAxisXSlider = slider(-100, 100, 10);
    rotationAxisYSlider = slider(-100, 100, 10);
    rotationAxisZSlider = slider(-100, 100, 10);
    morphSlider = slider(0, 1000, 100);
    morphSlider->setValue(1000);
    rotationAxisZSlider->setValue(100);
    rotationAxisXLabel = new QLabel("X axis:");
    rotationAxisYLabel = new QLabel("Y axis:");
    rotationAxisZLabel = new QLabel("Z axis:");
    morphLabel = new QLabel("Morphing:");
    rotationAxisXValueLabel = new QLabel("0");
    rotationAxisYValueLabel = new QLabel("0");
    rotationAxisZValueLabel = new QLabel("1");
    morphValueLabel = new QLabel("1.0");
    rotationAxisXValueLabel->setFixedWidth(40);
    rotationAxisYValueLabel->setFixedWidth(40);
    rotationAxisZValueLabel->setFixedWidth(40);
    rotationAxisLayout->addWidget(rotationAxisXLabel, 0, 0);
    rotationAxisLayout->addWidget(rotationAxisXSlider, 0, 1);
    rotationAxisLayout->addWidget(rotationAxisXValueLabel, 0, 2);
    rotationAxisLayout->addWidget(rotationAxisYLabel, 1, 0);
    rotationAxisLayout->addWidget(rotationAxisYSlider, 1, 1);
    rotationAxisLayout->addWidget(rotationAxisYValueLabel, 1, 2);
    rotationAxisLayout->addWidget(rotationAxisZLabel, 2, 0);
    rotationAxisLayout->addWidget(rotationAxisZSlider, 2, 1);
    rotationAxisLayout->addWidget(rotationAxisZValueLabel, 2, 2);
    rotationAxisLayout->addWidget(morphLabel, 3, 0);
    rotationAxisLayout->addWidget(morphSlider, 3, 1);
    rotationAxisLayout->addWidget(morphValueLabel, 3, 2);
    rotationAxisGroup->setLayout(rotationAxisLayout);
    rightColumn->addWidget(rotationAxisGroup);

    auto *lightingCompGroup = new QGroupBox("Lighting components");
    auto *lightingCompLayout = new QGridLayout;
    ambientSlider = slider(0, 100, 1);
    diffuseSlider = slider(0, 100, 1);
    specularSlider = slider(0, 100, 1);
    ambientSlider->setValue(30);
    diffuseSlider->setValue(50);
    specularSlider->setValue(50);
    ambientLabel = new QLabel("Ambient:");
    diffuseLabel = new QLabel("Diffuse:");
    specularLabel = new QLabel("Specular:");
    ambientValueLabel = new QLabel("0.3");
    diffuseValueLabel = new QLabel("0.5");
    specularValueLabel = new QLabel("0.5");
    ambientValueLabel->setFixedWidth(40);
    diffuseValueLabel->setFixedWidth(40);
    specularValueLabel->setFixedWidth(40);
    lightingCompLayout->addWidget(ambientLabel, 0, 0);
    lightingCompLayout->addWidget(ambientSlider, 0, 1);
    lightingCompLayout->addWidget(ambientValueLabel, 0, 2);
    lightingCompLayout->addWidget(diffuseLabel, 1, 0);
    lightingCompLayout->addWidget(diffuseSlider, 1, 1);
    lightingCompLayout->addWidget(diffuseValueLabel, 1, 2);
    lightingCompLayout->addWidget(specularLabel, 2, 0);
    lightingCompLayout->addWidget(specularSlider, 2, 1);
    lightingCompLayout->addWidget(specularValueLabel, 2, 2);
    lightingCompGroup->setLayout(lightingCompLayout);
    rightColumn->addWidget(lightingCompGroup);

    auto *lightParamGroup = new QGroupBox("Light parameters");
    auto *lightParamLayout = new QGridLayout;
    intensitySlider = slider(1, 100, 1);
    attenuationSlider = slider(1, 100, 1);
    innerCutoffSlider = slider(50, 100, 1);
    outerCutoffSlider = slider(50, 100, 1);
    intensitySlider->setValue(100);
    attenuationSlider->setValue(1);
    innerCutoffSlider->setValue(80);
    outerCutoffSlider->setValue(60);
    intensityLabel = new QLabel("Intensity:");
    attenuationLabel = new QLabel("Attenuation:");
    innerCutoffLabel = new QLabel("Inner cutoff:");
    outerCutoffLabel = new QLabel("Outer cutoff:");
    intensityValueLabel = new QLabel("100");
    attenuationValueLabel = new QLabel("1");
    innerCutoffValueLabel = new QLabel("0.8");
    outerCutoffValueLabel = new QLabel("0.6");
    intensityValueLabel->setFixedWidth(40);
    attenuationValueLabel->setFixedWidth(40);
    innerCutoffValueLabel->setFixedWidth(40);
    outerCutoffValueLabel->setFixedWidth(40);
    lightParamLayout->addWidget(intensityLabel, 0, 0);
    lightParamLayout->addWidget(intensitySlider, 0, 1);
    lightParamLayout->addWidget(intensityValueLabel, 0, 2);
    lightParamLayout->addWidget(attenuationLabel, 1, 0);
    lightParamLayout->addWidget(attenuationSlider, 1, 1);
    lightParamLayout->addWidget(attenuationValueLabel, 1, 2);
    lightParamLayout->addWidget(innerCutoffLabel, 2, 0);
    lightParamLayout->addWidget(innerCutoffSlider, 2, 1);
    lightParamLayout->addWidget(innerCutoffValueLabel, 2, 2);
    lightParamLayout->addWidget(outerCutoffLabel, 3, 0);
    lightParamLayout->addWidget(outerCutoffSlider, 3, 1);
    lightParamLayout->addWidget(outerCutoffValueLabel, 3, 2);
    lightParamGroup->setLayout(lightParamLayout);
    rightColumn->addWidget(lightParamGroup);

    mainLayout->addLayout(leftColumn, 0, 0);
    mainLayout->addLayout(rightColumn, 0, 1);
    mainLayout->setColumnStretch(0, 1);
    mainLayout->setColumnStretch(1, 1);

    setLayout(mainLayout);

    connect(subdivisionSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), mainWindow, &MainWindow::changeSubdivision);
    connect(sunSliderX, &QSlider::valueChanged, mainWindow, &MainWindow::changeLightX);
    connect(sunSliderY, &QSlider::valueChanged, mainWindow, &MainWindow::changeLightY);
    connect(sunSliderZ, &QSlider::valueChanged, mainWindow, &MainWindow::changeLightZ);
    connect(rotationAxisXSlider, &QSlider::valueChanged, mainWindow, &MainWindow::changeRotationAxisX);
    connect(rotationAxisYSlider, &QSlider::valueChanged, mainWindow, &MainWindow::changeRotationAxisY);
    connect(rotationAxisZSlider, &QSlider::valueChanged, mainWindow, &MainWindow::changeRotationAxisZ);
    connect(morphSlider, &QSlider::valueChanged, mainWindow, &MainWindow::changeMorphing);
    connect(ambientSlider, &QSlider::valueChanged, mainWindow, &MainWindow::setAmbientStrength);
    connect(diffuseSlider, &QSlider::valueChanged, mainWindow, &MainWindow::setDiffuseStrength);
    connect(specularSlider, &QSlider::valueChanged, mainWindow, &MainWindow::setSpecularStrength);
    connect(changeColorCubeButton, &QPushButton::clicked, mainWindow, &MainWindow::changeCubeColor);
    connect(changeColorLightButton, &QPushButton::clicked, mainWindow, &MainWindow::changeLightColor);
    connect(directionalLightRadio, &QRadioButton::clicked, [mainWindow]() {
        mainWindow->setLightType(0);
    });
    connect(pointLightRadio, &QRadioButton::clicked, [mainWindow]() {
        mainWindow->setLightType(1);
    });
    connect(spotlightRadio, &QRadioButton::clicked, [mainWindow]() {
        mainWindow->setLightType(2);
    });
    connect(intensitySlider, &QSlider::valueChanged, mainWindow, &MainWindow::setLightIntensity);
    connect(attenuationSlider, &QSlider::valueChanged, mainWindow, &MainWindow::setLightAttenuation);
    connect(innerCutoffSlider, &QSlider::valueChanged, mainWindow, &MainWindow::setLightCutoff);
    connect(outerCutoffSlider, &QSlider::valueChanged, mainWindow, &MainWindow::setOuterCutoff);
    connect(mainWindow, &MainWindow::fpsUpdated, this, [this](int fps) {
        fpsLabel->setText(QString("FPS: %1").arg(fps));
    });

    connect(sunSliderX, &QSlider::valueChanged, this, [this](int value) {
        lightXValueLabel->setText(QString::number(value / 100.0f));
    });
    connect(sunSliderY, &QSlider::valueChanged, this, [this](int value) {
        lightYValueLabel->setText(QString::number(value / 100.0f));
    });
    connect(sunSliderZ, &QSlider::valueChanged, this, [this](int value) {
        float z = 2.0f + (value / 100.0f) * 3.0f;
        lightZValueLabel->setText(QString::number(z, 'f', 1));
    });
    connect(rotationAxisXSlider, &QSlider::valueChanged, this, [this](int value) {
        rotationAxisXValueLabel->setText(QString::number(value / 100.0f));
    });
    connect(rotationAxisYSlider, &QSlider::valueChanged, this, [this](int value) {
        rotationAxisYValueLabel->setText(QString::number(value / 100.0f));
    });
    connect(rotationAxisZSlider, &QSlider::valueChanged, this, [this](int value) {
        rotationAxisZValueLabel->setText(QString::number(value / 100.0f));
    });
    connect(morphSlider, &QSlider::valueChanged, this, [this](int value) {
        morphValueLabel->setText(QString::number(value / 1000.0f));
    });
    connect(ambientSlider, &QSlider::valueChanged, this, [this](int value) {
        ambientValueLabel->setText(QString::number(value / 100.0f));
    });
    connect(diffuseSlider, &QSlider::valueChanged, this, [this](int value) {
        diffuseValueLabel->setText(QString::number(value / 100.0f));
    });
    connect(specularSlider, &QSlider::valueChanged, this, [this](int value) {
        specularValueLabel->setText(QString::number(value / 100.0f));
    });
    connect(intensitySlider, &QSlider::valueChanged, this, [this](int value) {
        intensityValueLabel->setText(QString::number(value));
    });
    connect(attenuationSlider, &QSlider::valueChanged, this, [this](int value) {
        attenuationValueLabel->setText(QString::number(value));
    });
    connect(innerCutoffSlider, &QSlider::valueChanged, this, [this, mainWindow](int value) {
        innerCutoffSlider->blockSignals(true);
        outerCutoffSlider->blockSignals(true);

        innerCutoffValueLabel->setText(QString::number(value / 100.0f));
        if (value <= outerCutoffSlider->value()) {
            outerCutoffSlider->setValue(value - 1);
            outerCutoffValueLabel->setText(QString::number((value - 1) / 100.0f));
        }

        mainWindow->setLightCutoff(value);
        mainWindow->setOuterCutoff(outerCutoffSlider->value());

        innerCutoffSlider->blockSignals(false);
        outerCutoffSlider->blockSignals(false);
    });
    connect(outerCutoffSlider, &QSlider::valueChanged, this, [this, mainWindow](int value) {
        innerCutoffSlider->blockSignals(true);
        outerCutoffSlider->blockSignals(true);

        outerCutoffValueLabel->setText(QString::number(value / 100.0f));
        if (value >= innerCutoffSlider->value()) {
            innerCutoffSlider->setValue(value + 1);
            innerCutoffValueLabel->setText(QString::number((value + 1) / 100.0f));
        }

        mainWindow->setLightCutoff(innerCutoffSlider->value());
        mainWindow->setOuterCutoff(value);

        innerCutoffSlider->blockSignals(false);
        outerCutoffSlider->blockSignals(false);
    });
}

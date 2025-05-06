#include "main_window.h"

MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
    squareWindow = new EarthGeometry(this);

    lightSpeedSlider = new QSlider(Qt::Vertical, this);
    lightSpeedSlider->setRange(0, 200);
    lightSpeedSlider->setValue(100);
    lightSpeedSlider->setFixedHeight(450);
    lightSpeedSlider->setTickPosition(QSlider::TicksRight);
    lightSpeedSlider->setTickInterval(20);

    textureCheckBox = new QCheckBox("Texture", this);
    textureCheckBox->setChecked(true);
    normalMapCheckBox = new QCheckBox("Normal map", this);
    normalMapCheckBox->setChecked(true);
    nightTextureCheckBox = new QCheckBox("Night texture", this);
    nightTextureCheckBox->setChecked(false);
    rotateCheckBox = new QCheckBox("Rotate planet", this);
    rotateCheckBox->setChecked(true);

    QVBoxLayout *sliderLayout = new QVBoxLayout();
    sliderLayout->addWidget(new QLabel("Light:", this));
    sliderLayout->addWidget(lightSpeedSlider);
    sliderLayout->addSpacing(10);
    sliderLayout->addWidget(textureCheckBox);
    sliderLayout->addWidget(normalMapCheckBox);
    sliderLayout->addWidget(nightTextureCheckBox);
    sliderLayout->addWidget(rotateCheckBox);
    sliderLayout->setAlignment(Qt::AlignTop);

    QWidget *sliderPanel = new QWidget(this);
    sliderLayout->addStretch();
    sliderPanel->setLayout(sliderLayout);
    sliderPanel->setMinimumWidth(150);

    QHBoxLayout *mainLayout = new QHBoxLayout(this);
    mainLayout->addWidget(squareWindow, 1);
    mainLayout->addWidget(sliderPanel, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    setLayout(mainLayout);

    connect(lightSpeedSlider, &QSlider::valueChanged, this, [=](int value) {
        float speed = value * 0.01f;
        squareWindow->setLightSpeed(speed / 10.0f);
    });

    connect(textureCheckBox, &QCheckBox::toggled, squareWindow, &EarthGeometry::setUseTexture);
    connect(normalMapCheckBox, &QCheckBox::toggled, squareWindow, &EarthGeometry::setUseNormalMap);
    connect(nightTextureCheckBox, &QCheckBox::toggled, squareWindow, &EarthGeometry::setUseNightTexture);
    connect(rotateCheckBox, &QCheckBox::toggled, squareWindow, &EarthGeometry::setRotationEnabled);

    squareWindow->setLightSpeed(0.1f);

    setWindowTitle("Display");
    resize(800, 600);
}

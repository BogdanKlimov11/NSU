#include <QApplication>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QCheckBox>

#include "road_widget.h"

int main(int argc, char* argv[]) {
    QApplication a(argc, argv);
    QWidget* mainWindow = new QWidget;
    mainWindow->setWindowTitle("Display");
    RoadWidget* roadWidget = new RoadWidget(mainWindow);
    roadWidget->setMinimumSize(200, 200);
    QWidget* settingsWindow = new QWidget;
    settingsWindow->setWindowTitle("Texture settings");
    QGridLayout* layout = new QGridLayout;
    layout->addWidget(new QLabel("Rock filter:"), 0, 0);
    layout->addWidget(new QLabel("Road filter:"), 0, 1);
    layout->addWidget(new QLabel("Cracks filter:"), 0, 2);
    layout->addWidget(new QLabel("Mix parameter:"), 2, 0);
    layout->addWidget(new QLabel("Camera height:"), 3, 0);
    layout->addWidget(new QLabel("Auto speed:"), 4, 0);
    layout->addWidget(new QLabel("Auto mode:"), 5, 0);
    QDoubleSpinBox* rockFilterSelector = new QDoubleSpinBox;
    QDoubleSpinBox* roadFilterSelector = new QDoubleSpinBox;
    QDoubleSpinBox* crackFilterSelector = new QDoubleSpinBox;
    rockFilterSelector->setRange(0, 2);
    roadFilterSelector->setRange(0, 2);
    crackFilterSelector->setRange(0, 2);
    rockFilterSelector->setDecimals(0);
    roadFilterSelector->setDecimals(0);
    crackFilterSelector->setDecimals(0);
    rockFilterSelector->setValue(2);
    roadFilterSelector->setValue(2);
    crackFilterSelector->setValue(2);
    QSlider* mixSlider = new QSlider(Qt::Horizontal);
    mixSlider->setTickInterval(1);
    mixSlider->setRange(0, 100);
    mixSlider->setValue(50);
    QSlider* autoSpeedSlider = new QSlider(Qt::Horizontal);
    autoSpeedSlider->setTickInterval(1);
    autoSpeedSlider->setRange(10, 100);
    autoSpeedSlider->setValue(50);
    QSlider* cameraHeightSlider = new QSlider(Qt::Horizontal);
    cameraHeightSlider->setTickInterval(1);
    cameraHeightSlider->setRange(10, 200);
    cameraHeightSlider->setValue(20);
    QCheckBox* autoModeCheckBox = new QCheckBox;
    autoModeCheckBox->setChecked(false);
    layout->addWidget(rockFilterSelector, 1, 0);
    layout->addWidget(roadFilterSelector, 1, 1);
    layout->addWidget(crackFilterSelector, 1, 2);
    layout->addWidget(mixSlider, 2, 1, 1, 2);
    layout->addWidget(cameraHeightSlider, 3, 1, 1, 2);
    layout->addWidget(autoSpeedSlider, 4, 1, 1, 2);
    layout->addWidget(autoModeCheckBox, 5, 1, 1, 2);
    QObject::connect(mixSlider, &QSlider::valueChanged, roadWidget, &RoadWidget::changeMixParameter);
    QObject::connect(rockFilterSelector, &QDoubleSpinBox::valueChanged, roadWidget, &RoadWidget::changeRockFiltering);
    QObject::connect(roadFilterSelector, &QDoubleSpinBox::valueChanged, roadWidget, &RoadWidget::changeRoadFiltering);
    QObject::connect(crackFilterSelector, &QDoubleSpinBox::valueChanged, roadWidget, &RoadWidget::changeCracksFiltering);
    QObject::connect(autoSpeedSlider, &QSlider::valueChanged, roadWidget, [roadWidget](int value) {
        roadWidget->setAutoSpeed(value / 100.0f);
    });
    QObject::connect(cameraHeightSlider, &QSlider::valueChanged, roadWidget, [roadWidget](int value) {
        roadWidget->setCameraHeight(value / 100.0f);
    });
    QObject::connect(autoModeCheckBox, &QCheckBox::toggled, roadWidget, &RoadWidget::toggleAutoMode);
    settingsWindow->setLayout(layout);
    settingsWindow->resize(300, 170);
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(roadWidget);
    mainWindow->setLayout(mainLayout);
    mainWindow->resize(700, 700);
    mainWindow->show();
    settingsWindow->show();
    return a.exec();
}

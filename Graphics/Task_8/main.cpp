#include <QApplication>
#include <QSlider>
#include <QComboBox>
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

    QComboBox* rockFilterCombo = new QComboBox();
    QComboBox* roadFilterCombo = new QComboBox();
    QComboBox* cracksFilterCombo = new QComboBox();

    const QStringList filterOptions = {
        "NEAREST",
        "LINEAR",
        "NEAREST-NEAREST",
        "NEAREST-LINEAR",
        "LINEAR-NEAREST",
        "LINEAR-LINEAR"
    };

    for (const auto& option : filterOptions) {
        rockFilterCombo->addItem(option);
        roadFilterCombo->addItem(option);
        cracksFilterCombo->addItem(option);
    }

    rockFilterCombo->setCurrentIndex(5);
    roadFilterCombo->setCurrentIndex(5);
    cracksFilterCombo->setCurrentIndex(5);

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

    layout->addWidget(rockFilterCombo, 1, 0);
    layout->addWidget(roadFilterCombo, 1, 1);
    layout->addWidget(cracksFilterCombo, 1, 2);
    layout->addWidget(mixSlider, 2, 1, 1, 2);
    layout->addWidget(cameraHeightSlider, 3, 1, 1, 2);
    layout->addWidget(autoSpeedSlider, 4, 1, 1, 2);
    layout->addWidget(autoModeCheckBox, 5, 1, 1, 2);

    QObject::connect(mixSlider, &QSlider::valueChanged, roadWidget, &RoadWidget::changeMixParameter);
    QObject::connect(rockFilterCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     roadWidget, &RoadWidget::changeRockFiltering);
    QObject::connect(roadFilterCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     roadWidget, &RoadWidget::changeRoadFiltering);
    QObject::connect(cracksFilterCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     roadWidget, &RoadWidget::changeCracksFiltering);
    QObject::connect(autoSpeedSlider, &QSlider::valueChanged, roadWidget, [roadWidget](int value) {
        roadWidget->setAutoSpeed(value / 100.0f);
    });
    QObject::connect(cameraHeightSlider, &QSlider::valueChanged, roadWidget, [roadWidget](int value) {
        roadWidget->setCameraHeight(value / 100.0f);
    });
    QObject::connect(autoModeCheckBox, &QCheckBox::toggled, roadWidget, &RoadWidget::toggleAutoMode);

    settingsWindow->setLayout(layout);
    settingsWindow->resize(350, 170);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(roadWidget);
    mainWindow->setLayout(mainLayout);
    mainWindow->resize(700, 700);


    mainWindow->show();
    settingsWindow->show();

    return a.exec();
}

#include <QApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QSurfaceFormat>

#include "mesh_widget.h"
#include "light_window.h"

int main(int argc, char* argv[]) {
    QSurfaceFormat format;
    format.setVersion(2, 1);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);

    QWidget* mainWindow = new QWidget;
    mainWindow->setFixedSize(800, 600);
    mainWindow->setWindowTitle("Display");
    mainWindow->setWindowIcon(QIcon(":/cube.ico"));

    QPalette darkTheme;
    darkTheme.setColor(QPalette::Window, QColor(53, 53, 53));
    darkTheme.setColor(QPalette::WindowText, QColor(203, 203, 203));
    mainWindow->setPalette(darkTheme);

    QVBoxLayout* mainLayout = new QVBoxLayout;

    QHBoxLayout* topLayout = new QHBoxLayout;

    MeshWidget* meshWidget = new MeshWidget;
    meshWidget->setMinimumSize(400, 400);
    meshWidget->setFixedWidth(675);
    topLayout->addWidget(meshWidget, 1);

    QVBoxLayout* cameraLayout = new QVBoxLayout;
    QLabel* cameraLabel = new QLabel("Camera");
    QSlider* cameraSlider = new QSlider(Qt::Vertical);
    cameraSlider->setRange(1, 100);
    cameraSlider->setSliderPosition(10);
    cameraSlider->setFixedHeight(450);
    cameraLayout->addStretch();
    cameraLayout->addWidget(cameraLabel, 0, Qt::AlignHCenter);
    cameraLayout->addWidget(cameraSlider, 0, Qt::AlignHCenter);
    cameraLayout->addStretch();

    QHBoxLayout* cameraHorizontalLayout = new QHBoxLayout;
    cameraHorizontalLayout->addStretch();
    cameraHorizontalLayout->addLayout(cameraLayout);
    cameraHorizontalLayout->addStretch();

    topLayout->addLayout(cameraHorizontalLayout);

    mainLayout->addLayout(topLayout);

    QHBoxLayout* firstRow = new QHBoxLayout;
    QLabel* kaLabel = new QLabel("Ambient");
    QSlider* kaSlider = new QSlider(Qt::Horizontal);
    kaSlider->setRange(1, 20);
    kaSlider->setSliderPosition(10);
    QComboBox* objectSelector = new QComboBox;
    objectSelector->addItem("Cube");
    objectSelector->addItem("House");
    objectSelector->addItem("Sphere");
    objectSelector->setFixedWidth(100);
    firstRow->addWidget(kaLabel);
    firstRow->addWidget(kaSlider, 1);
    firstRow->addWidget(objectSelector);
    mainLayout->addLayout(firstRow);

    QHBoxLayout* secondRow = new QHBoxLayout;
    QLabel* kdLabel = new QLabel("Diffusion");
    QSlider* kdSlider = new QSlider(Qt::Horizontal);
    kdSlider->setRange(1, 20);
    kdSlider->setSliderPosition(10);
    QPushButton* settingsButton = new QPushButton("Settings");
    settingsButton->setFixedWidth(100);
    secondRow->addWidget(kdLabel);
    secondRow->addWidget(kdSlider, 1);
    secondRow->addWidget(settingsButton);
    mainLayout->addLayout(secondRow);

    QHBoxLayout* thirdRow = new QHBoxLayout;
    QLabel* ksLabel = new QLabel("Specular");
    QSlider* ksSlider = new QSlider(Qt::Horizontal);
    ksSlider->setRange(1, 40);
    ksSlider->setSliderPosition(0);
    QPushButton* closeButton = new QPushButton;
    closeButton->setIcon(QIcon(":/close.ico"));
    closeButton->setFixedWidth(100);
    thirdRow->addWidget(ksLabel);
    thirdRow->addWidget(ksSlider, 1);
    thirdRow->addWidget(closeButton);
    mainLayout->addLayout(thirdRow);

    LightsController* lightsController = new LightsController(meshWidget);
    lightsController->setPalette(darkTheme);

    QObject::connect(objectSelector, &QComboBox::currentIndexChanged, meshWidget, &MeshWidget::setCurrentObject);
    QObject::connect(kaSlider, &QSlider::valueChanged, meshWidget, &MeshWidget::setAmbientCoef);
    QObject::connect(kdSlider, &QSlider::valueChanged, meshWidget, &MeshWidget::setDiffuseCoef);
    QObject::connect(ksSlider, &QSlider::valueChanged, meshWidget, &MeshWidget::setSpecularCoef);
    QObject::connect(cameraSlider, &QSlider::valueChanged, meshWidget, &MeshWidget::moveCamera);
    QObject::connect(settingsButton, &QPushButton::clicked, lightsController, &QWidget::show);
    QObject::connect(closeButton, &QPushButton::clicked, mainWindow, &QWidget::close);
    QObject::connect(closeButton, &QPushButton::clicked, lightsController, &QWidget::close);

    mainWindow->setLayout(mainLayout);
    mainWindow->show();

    return app.exec();
}

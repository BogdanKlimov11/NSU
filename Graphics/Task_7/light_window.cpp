#include <QVBoxLayout>
#include <QLabel>

#include "light_window.h"

LightsController::LightsController(MeshWidget* meshWidget, QWidget* parent)
    : QDialog(parent), meshWidget(meshWidget) {
    setFixedSize(200, 250);
    setWindowTitle("Settings");
    setWindowIcon(QIcon(":/settings.ico"));
    QVBoxLayout* layout = new QVBoxLayout(this);

    directedLightCheck = new QCheckBox("Directional light", this);
    directedLightCheck->setChecked(true);
    layout->addWidget(directedLightCheck);

    lightX = new QDoubleSpinBox(this);
    lightX->setRange(-10.0, 10.0);
    lightX->setValue(-10.0);
    layout->addWidget(new QLabel("X (from -10 to 10):"));
    layout->addWidget(lightX);

    lightY = new QDoubleSpinBox(this);
    lightY->setRange(-10.0, 10.0);
    lightY->setValue(-10.0);
    layout->addWidget(new QLabel("Y (from -10 to 10):"));
    layout->addWidget(lightY);

    lightZ = new QDoubleSpinBox(this);
    lightZ->setRange(-10.0, 10.0);
    lightZ->setValue(-10.0);
    layout->addWidget(new QLabel("Z (from -10 to 10):"));
    layout->addWidget(lightZ);

    connect(directedLightCheck, &QCheckBox::toggled, meshWidget, &MeshWidget::switchDirectedLight);
    connect(lightX, &QDoubleSpinBox::valueChanged, meshWidget, &MeshWidget::setDirectedLightX);
    connect(lightY, &QDoubleSpinBox::valueChanged, meshWidget, &MeshWidget::setDirectedLightY);
    connect(lightZ, &QDoubleSpinBox::valueChanged, meshWidget, &MeshWidget::setDirectedLightZ);
}

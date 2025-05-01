#pragma once

#include <QDialog>
#include <QCheckBox>
#include <QDoubleSpinBox>

#include "mesh_widget.h"

class LightsController : public QDialog {
    Q_OBJECT
public:
    LightsController(MeshWidget* meshWidget, QWidget* parent = nullptr);

private:
    MeshWidget* meshWidget;
    QCheckBox* directedLightCheck;
    QDoubleSpinBox* lightX;
    QDoubleSpinBox* lightY;
    QDoubleSpinBox* lightZ;
};

#pragma once

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCheckBox>

#include "earth_geometry.h"

class MainWindow : public QWidget {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private:
    EarthGeometry *squareWindow;
    QSlider *lightSpeedSlider;
    QCheckBox *textureCheckBox;
    QCheckBox *normalMapCheckBox;
    QCheckBox *nightTextureCheckBox;
    QCheckBox *rotateCheckBox;
};

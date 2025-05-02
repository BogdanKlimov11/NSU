#pragma once

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCheckBox>

#include "square_window.h"

class MainWindow : public QWidget {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private:
    SquareWindow *squareWindow;
    QSlider *lightSpeedSlider;
    QCheckBox *textureCheckBox;
    QCheckBox *normalMapCheckBox;
    QCheckBox *nightTextureCheckBox;
    QCheckBox *rotateCheckBox;
};

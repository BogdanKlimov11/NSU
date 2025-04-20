#pragma once

#include <QColorDialog>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QWidget>
#include <type_traits>
#include <QPushButton>

#include "square_window.h"

class LightWidget : public QWidget {
    Q_OBJECT

public:
    explicit LightWidget(SquareWindow *renderWindow, QWidget *parent = nullptr);

private:
    SquareWindow *renderWindow_;

    QSlider *morphSlider;
    QSpinBox *tSpinBox;
    QSlider *sun_Slider_x;
    QSlider *sun_Slider_y;
    QSlider *sun_Slider_z;
    QLabel *nLabel;
    QLabel *lightXLabel;
    QLabel *lightYLabel;
    QLabel *lightZLabel;
    QLabel *morphLabel;

    QRadioButton *directionalLightRadio;
    QRadioButton *pointLightRadio;
    QRadioButton *spotlightRadio;
    QSlider *intensitySlider;
    QSlider *attenuationSlider;
    QSlider *cutoffSlider;

    QPushButton *materialAmbientButton;
    QPushButton *materialDiffuseButton;
    QPushButton *materialSpecularButton;
    QPushButton *globalAmbientButton;
};

#pragma once

#include <QColorDialog>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QPushButton>

#include "main_window.h"

class LightWidget : public QWidget {
    Q_OBJECT

public:
    LightWidget(MainWindow *mainWindow, QWidget *parent = nullptr);

private:
    QSlider *morphSlider;
    QSpinBox *subdivisionSpinBox;
    QSlider *sunSliderX;
    QSlider *sunSliderY;
    QSlider *sunSliderZ;
    QSlider *rotationAxisXSlider;
    QSlider *rotationAxisYSlider;
    QSlider *rotationAxisZSlider;
    QSlider *ambientSlider;
    QSlider *diffuseSlider;
    QSlider *specularSlider;
    QLabel *subdivisionLabel;
    QLabel *lightXLabel;
    QLabel *lightYLabel;
    QLabel *lightZLabel;
    QLabel *rotationAxisXLabel;
    QLabel *rotationAxisYLabel;
    QLabel *rotationAxisZLabel;
    QLabel *morphLabel;
    QLabel *ambientLabel;
    QLabel *diffuseLabel;
    QLabel *specularLabel;
    QLabel *lightXValueLabel;
    QLabel *lightYValueLabel;
    QLabel *lightZValueLabel;
    QLabel *rotationAxisXValueLabel;
    QLabel *rotationAxisYValueLabel;
    QLabel *rotationAxisZValueLabel;
    QLabel *morphValueLabel;
    QLabel *ambientValueLabel;
    QLabel *diffuseValueLabel;
    QLabel *specularValueLabel;
    QRadioButton *directionalLightRadio;
    QRadioButton *pointLightRadio;
    QRadioButton *spotlightRadio;
    QSlider *intensitySlider;
    QSlider *attenuationSlider;
    QSlider *innerCutoffSlider;
    QSlider *outerCutoffSlider;
    QLabel *intensityLabel;
    QLabel *attenuationLabel;
    QLabel *innerCutoffLabel;
    QLabel *outerCutoffLabel;
    QLabel *intensityValueLabel;
    QLabel *attenuationValueLabel;
    QLabel *innerCutoffValueLabel;
    QLabel *outerCutoffValueLabel;
};

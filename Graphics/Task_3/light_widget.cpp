#include "light_widget.h"

template <typename T, typename = std::enable_if_t<std::is_pointer<T>::value>>
using main = T;

auto slider(int min_value, int max_value, int tick_value) {
    auto *const slider = main<QSlider*>(new QSlider(Qt::Orientation::Horizontal));
    slider->setRange(min_value, max_value);
    slider->setSingleStep(tick_value);
    slider->setTickPosition(QSlider::TicksRight);
    slider->setValue(max_value);
    return slider;
}

LightWidget::LightWidget(SquareWindow *renderWindow, QWidget *parent) : QWidget(parent), renderWindow_(renderWindow) {
    auto *const container = main<QGridLayout *>(new QGridLayout);
    auto *const mainLayout = main<QVBoxLayout *>(new QVBoxLayout);
    auto *const w = main<QWidget *>(new QWidget);

    tSpinBox = new QSpinBox;
    tSpinBox->setRange(2, 100);
    tSpinBox->setValue(10);
    tSpinBox->setSingleStep(1);

    sun_Slider_x = slider(-100, 100, 5);
    sun_Slider_y = slider(-100, 100, 5);
    sun_Slider_z = slider(-100, 100, 5);
    morphSlider = slider(0, 1000, 10);

    nLabel = new QLabel("Плотность сетки (n):");
    lightXLabel = new QLabel("Свет X:");
    lightYLabel = new QLabel("Свет Y:");
    lightZLabel = new QLabel("Свет Z:");
    morphLabel = new QLabel("Морфинг:");
    directionalLightRadio = new QRadioButton("Направленный свет");
    pointLightRadio = new QRadioButton("Точечный свет");
    spotlightRadio = new QRadioButton("Прожектор");
    directionalLightRadio->setChecked(true);
    intensitySlider = slider(1, 100, 1);
    attenuationSlider = slider(1, 100, 1);
    cutoffSlider = slider(70, 100, 1);

    QPushButton *change_color_cube_button = new QPushButton(tr("&Цвет куба"));
    change_color_cube_button->setFocusPolicy(Qt::NoFocus);

    QPushButton *change_color_light_button = new QPushButton(tr("&Цвет света"));
    change_color_light_button->setFocusPolicy(Qt::NoFocus);

    materialAmbientButton = new QPushButton(tr("&Ambient материал"));
    materialAmbientButton->setFocusPolicy(Qt::NoFocus);
    materialDiffuseButton = new QPushButton(tr("&Diffuse материал"));
    materialDiffuseButton->setFocusPolicy(Qt::NoFocus);
    materialSpecularButton = new QPushButton(tr("&Specular материал"));
    materialSpecularButton->setFocusPolicy(Qt::NoFocus);
    globalAmbientButton = new QPushButton(tr("&Фоновое освещение"));
    globalAmbientButton->setFocusPolicy(Qt::NoFocus);

    int row = 0;
    container->addWidget(nLabel, row, 0);
    container->addWidget(tSpinBox, row++, 1);
    container->addWidget(lightXLabel, row, 0);
    container->addWidget(sun_Slider_x, row++, 1);
    container->addWidget(lightYLabel, row, 0);
    container->addWidget(sun_Slider_y, row++, 1);
    container->addWidget(lightZLabel, row, 0);
    container->addWidget(sun_Slider_z, row++, 1);
    container->addWidget(morphLabel, row, 0);
    container->addWidget(morphSlider, row++, 1);
    container->addWidget(change_color_cube_button, row, 0);
    container->addWidget(change_color_light_button, row++, 1);
    container->addWidget(new QLabel("Тип света:"), row, 0);
    container->addWidget(directionalLightRadio, row++, 1);
    container->addWidget(pointLightRadio, row, 0);
    container->addWidget(spotlightRadio, row++, 1);
    container->addWidget(new QLabel("Интенсивность:"), row, 0);
    container->addWidget(intensitySlider, row++, 1);
    container->addWidget(new QLabel("Затухание:"), row, 0);
    container->addWidget(attenuationSlider, row++, 1);
    container->addWidget(new QLabel("Угол обреза:"), row, 0);
    container->addWidget(cutoffSlider, row++, 1);
    container->addWidget(materialAmbientButton, row, 0);
    container->addWidget(materialDiffuseButton, row++, 1);
    container->addWidget(materialSpecularButton, row, 0);
    container->addWidget(globalAmbientButton, row++, 1);

    w->setLayout(container);
    mainLayout->addWidget(w);
    setLayout(mainLayout);

    connect(tSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), renderWindow_, &SquareWindow::changeN);
    connect(sun_Slider_x, &QSlider::valueChanged, renderWindow_, &SquareWindow::change_light_x_param);
    connect(sun_Slider_y, &QSlider::valueChanged, renderWindow_, &SquareWindow::change_light_y_param);
    connect(sun_Slider_z, &QSlider::valueChanged, renderWindow_, &SquareWindow::change_light_z_param);
    connect(morphSlider, &QSlider::valueChanged, renderWindow_, &SquareWindow::change_morph_param);
    connect(change_color_cube_button, &QPushButton::clicked, renderWindow_, &SquareWindow::change_cube_color);
    connect(change_color_light_button, &QPushButton::clicked, renderWindow_, &SquareWindow::change_light_color);

    connect(directionalLightRadio, &QRadioButton::clicked, [this]() {
        renderWindow_->setLightType(0);
    });

    connect(pointLightRadio, &QRadioButton::clicked, [this]() {
        renderWindow_->setLightType(1);
    });

    connect(spotlightRadio, &QRadioButton::clicked, [this]() {
        renderWindow_->setLightType(2);
    });

    connect(intensitySlider, &QSlider::valueChanged, renderWindow_, &SquareWindow::setLightIntensity);
    connect(attenuationSlider, &QSlider::valueChanged, renderWindow_, &SquareWindow::setLightAttenuation);
    connect(cutoffSlider, &QSlider::valueChanged, renderWindow_, &SquareWindow::setLightCutoff);

    connect(materialAmbientButton, &QPushButton::clicked, renderWindow_, &SquareWindow::setMaterialAmbient);
    connect(materialDiffuseButton, &QPushButton::clicked, renderWindow_, &SquareWindow::setMaterialDiffuse);
    connect(materialSpecularButton, &QPushButton::clicked, renderWindow_, &SquareWindow::setMaterialSpecular);
    connect(globalAmbientButton, &QPushButton::clicked, renderWindow_, &SquareWindow::setGlobalAmbient);
}

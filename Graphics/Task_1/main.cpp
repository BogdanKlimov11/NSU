#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QSurfaceFormat>
#include <QDoubleValidator>
#include <QResizeEvent>

#include "square_widget.h"

class MainWindow : public QMainWindow {
public:
    MainWindow() {
        QSurfaceFormat format;
        format.setSamples(16);

        TriangleWindow *glWidget = new TriangleWindow;
        glWidget->setFormat(format);
        glWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        QWidget *controlPanel = new QWidget;
        controlPanel->setMinimumHeight(150);
        QVBoxLayout *controlLayout = new QVBoxLayout(controlPanel);
        controlLayout->setSpacing(10);
        controlLayout->setContentsMargins(10, 10, 10, 10);

        QHBoxLayout *axisLabelLayout = new QHBoxLayout;
        QLabel *axisLabel = new QLabel("Rotation axis <x> <y> <z> (coordinates take values ​​-1, 0, 1):");
        axisLabel->setFixedWidth(400);
        axisLabelLayout->addWidget(axisLabel);
        axisLabelLayout->addStretch();
        controlLayout->addLayout(axisLabelLayout);

        QHBoxLayout *axisInputLayout = new QHBoxLayout;
        QLineEdit *xAxisInput = new QLineEdit("1");
        QLineEdit *yAxisInput = new QLineEdit("1");
        QLineEdit *zAxisInput = new QLineEdit("1");
        xAxisInput->setValidator(new QDoubleValidator(-1, 1, 6, this));
        xAxisInput->setContentsMargins(0, 0, 65, 0);
        yAxisInput->setValidator(new QDoubleValidator(-1, 1, 6, this));
        yAxisInput->setContentsMargins(0, 0, 65, 0);
        zAxisInput->setValidator(new QDoubleValidator(-1, 1, 6, this));
        zAxisInput->setContentsMargins(0, 0, 65, 0);
        xAxisInput->setFixedWidth(100);
        yAxisInput->setFixedWidth(100);
        zAxisInput->setFixedWidth(100);
        QLabel *xLabel = new QLabel("X axis:");
        QLabel *yLabel = new QLabel("Y axis:");
        QLabel *zLabel = new QLabel("Z axis:");
        xLabel->setFixedWidth(35);
        yLabel->setFixedWidth(35);
        zLabel->setFixedWidth(35);
        axisInputLayout->addWidget(xLabel);
        axisInputLayout->addWidget(xAxisInput);
        axisInputLayout->addWidget(yLabel);
        axisInputLayout->addWidget(yAxisInput);
        axisInputLayout->addWidget(zLabel);
        axisInputLayout->addWidget(zAxisInput);
        axisInputLayout->addStretch();
        controlLayout->addLayout(axisInputLayout);

        QHBoxLayout *colorLayout = new QHBoxLayout;
        QLabel *colorLabel = new QLabel("Cube color:");
        colorLabel->setFixedWidth(140);
        QComboBox *colorCombo = new QComboBox;
        colorCombo->addItem("White", QVariant(QVector3D(1, 1, 1)));
        colorCombo->addItem("Black", QVariant(QVector3D(0, 0, 0)));
        colorCombo->addItem("Red", QVariant(QVector3D(1, 0, 0)));
        colorCombo->addItem("Blue", QVariant(QVector3D(0, 0, 1)));
        colorCombo->addItem("Green", QVariant(QVector3D(0, 1, 0)));
        colorCombo->addItem("Yellow", QVariant(QVector3D(1, 1, 0)));
        colorCombo->addItem("Pink", QVariant(QVector3D(1, 0, 1)));
        colorCombo->addItem("Cyan", QVariant(QVector3D(0, 1, 1)));
        colorCombo->addItem("Multicolor", QVariant(QVector3D(-1, -1, -1)));
        colorCombo->setCurrentText("White");
        colorCombo->setFixedWidth(240);
        colorLayout->addWidget(colorLabel);
        colorLayout->addWidget(colorCombo);
        colorLayout->addStretch();
        controlLayout->addLayout(colorLayout);

        QHBoxLayout *speedLayout = new QHBoxLayout;
        QLabel *speedLabel = new QLabel("Rotation speed:");
        speedLabel->setFixedWidth(140);
        QSlider *speedSlider = new QSlider(Qt::Horizontal);
        speedSlider->setRange(0, 100);
        speedSlider->setValue(100);
        speedSlider->setFixedWidth(200);
        QLabel *speedValueLabel = new QLabel("100%");
        speedValueLabel->setFixedWidth(50);
        speedLayout->addWidget(speedLabel);
        speedLayout->addWidget(speedSlider);
        speedLayout->addWidget(speedValueLabel);
        speedLayout->addStretch();
        controlLayout->addLayout(speedLayout);

        controlLayout->addStretch();

        QWidget *centralWidget = new QWidget;
        QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
        mainLayout->addWidget(glWidget, 3);
        mainLayout->addWidget(controlPanel, 1);
        setCentralWidget(centralWidget);

        auto updateCustomAxis = [=]() {
            bool okX, okY, okZ;
            float x = xAxisInput->text().toFloat(&okX);
            float y = yAxisInput->text().toFloat(&okY);
            float z = zAxisInput->text().toFloat(&okZ);
            if (okX && okY && okZ) {
                glWidget->setRotationAxis(QVector3D(x, y, z));
            }
        };
        connect(xAxisInput, &QLineEdit::editingFinished, this, updateCustomAxis);
        connect(yAxisInput, &QLineEdit::editingFinished, this, updateCustomAxis);
        connect(zAxisInput, &QLineEdit::editingFinished, this, updateCustomAxis);

        connect(colorCombo, &QComboBox::activated, this, [=](int index) {
            QVector3D color = colorCombo->itemData(index).value<QVector3D>();
            if (color == QVector3D(-1, -1, -1)) {
                glWidget->setMulticolorMode(true);
            }
            else {
                glWidget->setMulticolorMode(false);
                glWidget->setCubeColor(color);
            }
        });

        connect(speedSlider, &QSlider::valueChanged, this, [=](int value) {
            speedValueLabel->setText(QString("%1%").arg(value));
            glWidget->setRotationSpeed(value / 100.0f);
        });

        setWindowTitle("Display");
        resize(640, 480);
    }

protected:
    void resizeEvent(QResizeEvent *event) override {
        QMainWindow::resizeEvent(event);
        const float aspectRatio = 4.0f / 3.0f;
        int newWidth = event->size().width();
        int newHeight = static_cast<int>(newWidth / aspectRatio);
        if (newHeight != event->size().height()) {
            resize(newWidth, newHeight);
        }
    }
};

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}

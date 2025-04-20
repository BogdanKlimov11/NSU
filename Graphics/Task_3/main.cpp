#include <QApplication>
#include <QSurfaceFormat>

#include "light_widget.h"
#include "square_window.h"

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(16);
    format.setVersion(2, 1);
    QSurfaceFormat::setDefaultFormat(format);

    SquareWindow *renderWindow = new SquareWindow();
    renderWindow->setWindowTitle("Дисплей");
    renderWindow->resize(800, 600);
    renderWindow->setWindowIcon(QIcon(":/display.ico"));
    renderWindow->show();

    LightWidget *controlWindow = new LightWidget(renderWindow);
    controlWindow->setWindowTitle("Панель управления");
    controlWindow->resize(400, 600);
    controlWindow->setWindowIcon(QIcon(":/control.ico"));
    controlWindow->show();

    renderWindow->move(100, 100);
    controlWindow->move(900, 100);

    return app.exec();
}

#include <QApplication>
#include <QSurfaceFormat>

#include "light_widget.h"
#include "main_window.h"

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(16);
    format.setVersion(2, 1);

    MainWindow *sceneWindow = new MainWindow();
    sceneWindow->setWindowTitle("Display");
    sceneWindow->setMaximumSize(900, 700);
    sceneWindow->resize(900, 700);
    sceneWindow->setWindowIcon(QIcon(":/display.ico"));
    sceneWindow->show();

    LightWidget *settingsWindow = new LightWidget(sceneWindow);
    settingsWindow->setWindowTitle("Settings");
    settingsWindow->setMinimumSize(600, 450);
    settingsWindow->resize(600, 450);
    settingsWindow->setWindowIcon(QIcon(":/control.ico"));
    settingsWindow->show();

    return app.exec();
}

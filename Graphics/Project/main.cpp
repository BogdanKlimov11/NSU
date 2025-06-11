#include <QApplication>

#include "billiard_widget.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    BilliardWidget widget;
    widget.resize(800, 600);
    widget.setWindowIcon(QIcon(":/billiard.png"));
    widget.setWindowTitle("Billiard");
    widget.show();
    return app.exec();
}

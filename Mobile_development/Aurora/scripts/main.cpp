#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "GameLogic.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    // Регистрация логики игры для использования в QML
    qmlRegisterType<GameLogic>("CheckersGame", 1, 0, "GameLogic");

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/CheckersGame.qml"));
    engine.load(url);

    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}

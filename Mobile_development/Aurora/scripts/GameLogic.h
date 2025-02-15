#ifndef GAMELOGIC_H
#define GAMELOGIC_H

#include <QObject>

class GameLogic : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QStringList board READ board NOTIFY boardChanged)
    Q_PROPERTY(int currentPlayer READ currentPlayer NOTIFY currentPlayerChanged)

public:
    explicit GameLogic(QObject *parent = nullptr);

    // Метод для получения состояния доски
    QStringList board() const;

    // Метод для получения текущего игрока
    int currentPlayer() const;

    // Метод для совершения хода
    Q_INVOKABLE void makeMove(int fromRow, int fromCol, int toRow, int toCol);

signals:
    void boardChanged();
    void currentPlayerChanged();

private:
    QStringList m_board;
    int m_currentPlayer;  // 1 для игрока 1, -1 для игрока 2

    // Метод для инициализации доски
    void initializeBoard();
};

#endif // GAMELOGIC_H

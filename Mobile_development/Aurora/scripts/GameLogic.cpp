#include "GameLogic.h"

GameLogic::GameLogic(QObject *parent) : QObject(parent), m_currentPlayer(1)
{
    initializeBoard();
}

QStringList GameLogic::board() const
{
    return m_board;
}

int GameLogic::currentPlayer() const
{
    return m_currentPlayer;
}

void GameLogic::initializeBoard()
{
    m_board.clear();
    for (int i = 0; i < 8; ++i) {
        QString row = "";
        for (int j = 0; j < 8; ++j) {
            if ((i + j) % 2 == 0) {
                row.append("0");  // Пустое поле
            } else if (i < 3) {
                row.append("1");  // Шашка игрока 1
            } else if (i > 4) {
                row.append("-1"); // Шашка игрока 2
            } else {
                row.append("0");
            }
        }
        m_board.append(row);
    }
    emit boardChanged();
}

void GameLogic::makeMove(int fromRow, int fromCol, int toRow, int toCol)
{
    // Реализуйте логику для проверки допустимости хода, захвата шашки и обновления доски
    // Для простоты пока сделаем переход без проверки правил
    if (m_board[fromRow][fromCol] != "0") {
        m_board[toRow][toCol] = m_board[fromRow][fromCol];
        m_board[fromRow][fromCol] = "0";
        m_currentPlayer = -m_currentPlayer; // Переключаем игрока
        emit boardChanged();
        emit currentPlayerChanged();
    }
}

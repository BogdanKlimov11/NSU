import QtQuick 2.15
import QtQuick.Controls 2.15
import CheckersGame 1.0

ApplicationWindow {
    visible: true
    width: 600
    height: 600
    title: "Checkers Game"

    GameLogic {
        id: game
    }

    GridView {
        width: parent.width
        height: parent.height
        model: 8
        delegate: Rectangle {
            width: parent.width / 8
            height: parent.height / 8
            color: (index + Math.floor(game.board[index].toString() / 8)) % 2 == 0 ? "white" : "black"
            border.color: "gray"

            Text {
                anchors.centerIn: parent
                text: game.board[index]
                color: "black"
            }
        }
    }
}

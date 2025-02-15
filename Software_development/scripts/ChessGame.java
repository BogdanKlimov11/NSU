package game;

import board.Board;
import pieces.Piece;

public class ChessGame {
    private Board board;
    private boolean whiteTurn;  // Определяет, чья очередь

    public ChessGame() {
        board = new Board();
        whiteTurn = true;
    }

    public boolean makeMove(int startX, int startY, int endX, int endY) {
        Piece piece = board.getPiece(startX, startY);
        if (piece != null && piece.getColor().equals(whiteTurn ? "white" : "black")) {
            boolean isValid = board.movePiece(startX, startY, endX, endY);
            if (isValid) {
                whiteTurn = !whiteTurn;  // Переключаем очередь
            }
            return isValid;
        }
        return false;
    }

    public boolean isWhiteTurn() {
        return whiteTurn;
    }
}

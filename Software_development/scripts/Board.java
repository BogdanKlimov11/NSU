package board;

import pieces.*;

public class Board {
    private Piece[][] board;  // Игровая доска 8x8

    public Board() {
        this.board = new Piece[8][8];
        initializeBoard();  // Инициализация доски
    }

    private void initializeBoard() {
        // Размещение фигур на доске
        board[0][0] = new Rook("white");
        board[0][1] = new Knight("white");
        board[0][2] = new Bishop("white");
        board[0][3] = new Queen("white");
        board[0][4] = new King("white");
        board[0][5] = new Bishop("white");
        board[0][6] = new Knight("white");
        board[0][7] = new Rook("white");

        for (int i = 0; i < 8; i++) {
            board[1][i] = new Pawn("white");  // Белые пешки
        }

        // Черные фигуры
        board[7][0] = new Rook("black");
        board[7][1] = new Knight("black");
        board[7][2] = new Bishop("black");
        board[7][3] = new Queen("black");
        board[7][4] = new King("black");
        board[7][5] = new Bishop("black");
        board[7][6] = new Knight("black");
        board[7][7] = new Rook("black");

        for (int i = 0; i < 8; i++) {
            board[6][i] = new Pawn("black");  // Черные пешки
        }
    }

    public Piece getPiece(int x, int y) {
        return board[x][y];
    }

    public boolean movePiece(int startX, int startY, int endX, int endY) {
        Piece piece = getPiece(startX, startY);
        if (piece != null && piece.isValidMove(startX, startY, endX, endY)) {
            board[endX][endY] = piece;
            board[startX][startY] = null;
            return true;
        }
        return false;
    }
}

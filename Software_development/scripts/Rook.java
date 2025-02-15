package pieces;

public class Rook extends Piece {

    public Rook(String color) {
        super(color, "Rook");
    }

    @Override
    public boolean isValidMove(int startX, int startY, int endX, int endY) {
        // Ладья может двигаться только по вертикали или горизонтали
        return startX == endX || startY == endY;
    }
}

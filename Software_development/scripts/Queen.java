package pieces;

public class Queen extends Piece {

    public Queen(String color) {
        super(color, "Queen");
    }

    @Override
    public boolean isValidMove(int startX, int startY, int endX, int endY) {
        // Ферзь может ходить по вертикали, горизонтали или диагонали
        return startX == endX || startY == endY || Math.abs(startX - endX) == Math.abs(startY - endY);
    }
}

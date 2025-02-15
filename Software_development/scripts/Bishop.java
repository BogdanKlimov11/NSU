package pieces;

public class Bishop extends Piece {

    public Bishop(String color) {
        super(color, "Bishop");
    }

    @Override
    public boolean isValidMove(int startX, int startY, int endX, int endY) {
        // Слон может двигаться только по диагонали
        return Math.abs(startX - endX) == Math.abs(startY - endY);
    }
}

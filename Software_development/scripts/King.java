package pieces;

public class King extends Piece {

    public King(String color) {
        super(color, "King");
    }

    @Override
    public boolean isValidMove(int startX, int startY, int endX, int endY) {
        // Король ходит на одну клетку в любом направлении
        return Math.abs(startX - endX) <= 1 && Math.abs(startY - endY) <= 1;
    }
}

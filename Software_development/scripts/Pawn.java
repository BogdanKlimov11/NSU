package pieces;

public class Pawn extends Piece {

    public Pawn(String color) {
        super(color, "Pawn");
    }

    @Override
    public boolean isValidMove(int startX, int startY, int endX, int endY) {
        if (this.color.equals("white")) {
            return (startX == endX && startY + 1 == endY);  // Белая пешка
        } else {
            return (startX == endX && startY - 1 == endY);  // Черная пешка
        }
    }
}

package pieces;

public class Knight extends Piece {

    public Knight(String color) {
        super(color, "Knight");
    }

    @Override
    public boolean isValidMove(int startX, int startY, int endX, int endY) {
        // Конь ходит буквой "Г"
        return (Math.abs(startX - endX) == 2 && Math.abs(startY - endY) == 1) || 
               (Math.abs(startX - endX) == 1 && Math.abs(startY - endY) == 2);
    }
}

package pieces;

public abstract class Piece {
    protected String color;  // Цвет фигуры (черная или белая)
    protected String type;   // Тип фигуры (король, ферзь и т.д.)

    public Piece(String color, String type) {
        this.color = color;
        this.type = type;
    }

    public String getColor() {
        return color;
    }

    public String getType() {
        return type;
    }

    public abstract boolean isValidMove(int startX, int startY, int endX, int endY);
}

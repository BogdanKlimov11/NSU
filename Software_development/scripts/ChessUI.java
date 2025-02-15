package ui;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.layout.GridPane;
import javafx.stage.Stage;
import game.ChessGame;
import javafx.scene.control.Button;

public class ChessUI extends Application {

    private ChessGame game;

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) {
        game = new ChessGame();

        GridPane grid = new GridPane();
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                Button button = new Button();
                button.setMinSize(60, 60);
                button.setStyle("-fx-background-color: white; -fx-border-color: black;");
                button.setOnAction(event -> handleCellClick(i, j));
                grid.add(button, i, j);
            }
        }

        Scene scene = new Scene(grid, 480, 480);
        primaryStage.setTitle("Chess Game");
        primaryStage.setScene(scene);
        primaryStage.show();
    }

    private void handleCellClick(int x, int y) {
        // Логика обработки кликов по клеткам
        System.out.println("Cell clicked: " + x + ", " + y);
    }
}

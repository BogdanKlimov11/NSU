import Foundation
import UIKit

class GameModel {
    var snake: [(x: Int, y: Int)] = [(x: 10, y: 10)] // координаты сегментов змейки
    var food: (x: Int, y: Int) = (x: 5, y: 5) // координаты еды
    var direction: Direction = .right // начальное направление движения
    var gameOver: Bool = false // флаг окончания игры
    
    enum Direction {
        case up, down, left, right
    }
    
    func moveSnake() {
        guard !gameOver else { return }
        
        // Передвигаем голову змейки в соответствии с направлением
        var head = snake.first!
        switch direction {
        case .up:
            head.y -= 1
        case .down:
            head.y += 1
        case .left:
            head.x -= 1
        case .right:
            head.x += 1
        }
        
        // Добавляем новый сегмент головы и убираем хвост
        snake.insert(head, at: 0)
        
        // Проверка на столкновение с едой
        if head.x == food.x && head.y == food.y {
            // Генерация новой еды
            generateFood()
        } else {
            snake.removeLast()
        }
        
        // Проверка на столкновение с собой
        if snake.dropFirst().contains(where: { $0 == head }) {
            gameOver = true
        }
        
        // Проверка на столкновение с границей
        if head.x < 0 || head.x >= 20 || head.y < 0 || head.y >= 20 {
            gameOver = true
        }
    }
    
    func generateFood() {
        // Генерация случайных координат для еды
        food = (x: Int.random(in: 0..<20), y: Int.random(in: 0..<20))
    }
    
    func resetGame() {
        snake = [(x: 10, y: 10)]
        food = (x: 5, y: 5)
        direction = .right
        gameOver = false
    }
}

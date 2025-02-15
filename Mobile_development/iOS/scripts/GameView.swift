import UIKit

class GameView: UIView {
    var gameModel: GameModel!
    
    override func draw(_ rect: CGRect) {
        guard let context = UIGraphicsGetCurrentContext() else { return }
        
        // Очистка экрана
        context.clear(rect)
        
        // Отображение змейки
        for segment in gameModel.snake {
            context.setFillColor(UIColor.green.cgColor)
            context.fill(CGRect(x: segment.x * 20, y: segment.y * 20, width: 20, height: 20))
        }
        
        // Отображение еды
        context.setFillColor(UIColor.red.cgColor)
        context.fill(CGRect(x: gameModel.food.x * 20, y: gameModel.food.y * 20, width: 20, height: 20))
        
        // Если игра закончена, показываем сообщение
        if gameModel.gameOver {
            let gameOverText = "Game Over"
            let attributes: [NSAttributedString.Key: Any] = [
                .font: UIFont.boldSystemFont(ofSize: 40),
                .foregroundColor: UIColor.black
            ]
            let textSize = gameOverText.size(withAttributes: attributes)
            let point = CGPoint(x: (rect.width - textSize.width) / 2, y: rect.height / 2)
            gameOverText.draw(at: point, withAttributes: attributes)
        }
    }
}

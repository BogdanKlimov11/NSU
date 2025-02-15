import UIKit

class ViewController: UIViewController {
    var gameModel: GameModel!
    var gameView: GameView!
    var timer: Timer?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        gameModel = GameModel()
        gameView = GameView(frame: view.bounds)
        gameView.gameModel = gameModel
        view.addSubview(gameView)
        
        // Запуск таймера для обновления игры каждую секунду
        timer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(updateGame), userInfo: nil, repeats: true)
        
        // Обработка свайпов для управления направлением змейки
        let swipeUp = UISwipeGestureRecognizer(target: self, action: #selector(swipeDetected(_:)))
        swipeUp.direction = .up
        view.addGestureRecognizer(swipeUp)
        
        let swipeDown = UISwipeGestureRecognizer(target: self, action: #selector(swipeDetected(_:)))
        swipeDown.direction = .down
        view.addGestureRecognizer(swipeDown)
        
        let swipeLeft = UISwipeGestureRecognizer(target: self, action: #selector(swipeDetected(_:)))
        swipeLeft.direction = .left
        view.addGestureRecognizer(swipeLeft)
        
        let swipeRight = UISwipeGestureRecognizer(target: self, action: #selector(swipeDetected(_:)))
        swipeRight.direction = .right
        view.addGestureRecognizer(swipeRight)
    }
    
    @objc func updateGame() {
        gameModel.moveSnake()
        gameView.setNeedsDisplay()
    }
    
    @objc func swipeDetected(_ gesture: UISwipeGestureRecognizer) {
        switch gesture.direction {
        case .up:
            if gameModel.direction != .down { gameModel.direction = .up }
        case .down:
            if gameModel.direction != .up { gameModel.direction = .down }
        case .left:
            if gameModel.direction != .right { gameModel.direction = .left }
        case .right:
            if gameModel.direction != .left { gameModel.direction = .right }
        default:
            break
        }
    }
}

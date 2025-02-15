import Foundation

class ScoreManager {
    private var score: Int = 0
    
    func increaseScore() {
        score += 1
    }
    
    func resetScore() {
        score = 0
    }
    
    func getScore() -> Int {
        return score
    }
}

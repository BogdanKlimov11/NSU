package com.example.tictactoe

import android.os.Bundle
import android.widget.Button
import android.widget.GridLayout
import androidx.appcompat.app.AppCompatActivity

class GameActivity : AppCompatActivity() {
    private lateinit var buttons: Array<Array<Button>>
    private var currentPlayer = 'X'
    private val board = Array(3) { CharArray(3) { ' ' } }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_game)

        val gridLayout: GridLayout = findViewById(R.id.game_grid)
        buttons = Array(3) { row ->
            Array(3) { col ->
                val button = gridLayout.getChildAt(row * 3 + col) as Button
                button.setOnClickListener { onCellClicked(row, col) }
                button
            }
        }

        findViewById<Button>(R.id.btn_restart).setOnClickListener { resetGame() }
    }

    private fun onCellClicked(row: Int, col: Int) {
        if (board[row][col] == ' ') {
            board[row][col] = currentPlayer
            buttons[row][col].text = currentPlayer.toString()
            if (checkWin()) {
                disableBoard()
            } else {
                currentPlayer = if (currentPlayer == 'X') 'O' else 'X'
            }
        }
    }

    private fun checkWin(): Boolean {
        for (i in 0..2) {
            if (board[i][0] == currentPlayer && board[i][1] == currentPlayer && board[i][2] == currentPlayer) return true
            if (board[0][i] == currentPlayer && board[1][i] == currentPlayer && board[2][i] == currentPlayer) return true
        }
        if (board[0][0] == currentPlayer && board[1][1] == currentPlayer && board[2][2] == currentPlayer) return true
        if (board[0][2] == currentPlayer && board[1][1] == currentPlayer && board[2][0] == currentPlayer) return true
        return false
    }

    private fun disableBoard() {
        for (row in buttons) {
            for (button in row) {
                button.isEnabled = false
            }
        }
    }

    private fun resetGame() {
        for (i in 0..2) {
            for (j in 0..2) {
                board[i][j] = ' '
                buttons[i][j].text = ""
                buttons[i][j].isEnabled = true
            }
        }
        currentPlayer = 'X'
    }
}

package com.tictactoe

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel

class TicTacToeViewModel : ViewModel() {

    private val _gameState = MutableLiveData<GameState>()
    val gameState: LiveData<GameState> get() = _gameState

    init {
        _gameState.value = GameState()
    }

    fun playMove(position: Int) {
        val currentState = _gameState.value ?: return
        if (currentState.isGameOver || currentState.board[position] != null) return
        val newBoard = currentState.board.toMutableList()
        newBoard[position] = currentState.currentPlayer
        val newState = currentState.copy(board = newBoard, currentPlayer = currentState.currentPlayer.opposite())
        _gameState.value = newState
    }
}

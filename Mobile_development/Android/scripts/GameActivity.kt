package com.tictactoe

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.ViewModelProvider
import com.tictactoe.databinding.ActivityGameBinding

class GameActivity : AppCompatActivity() {

    private lateinit var binding: ActivityGameBinding
    private lateinit var viewModel: TicTacToeViewModel

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityGameBinding.inflate(layoutInflater)
        setContentView(binding.root)

        viewModel = ViewModelProvider(this).get(TicTacToeViewModel::class.java)

        binding.gridView.setOnClickListener { position ->
            viewModel.playMove(position)
        }

        viewModel.gameState.observe(this, { gameState ->
            binding.gridView.updateGrid(gameState)
            if (gameState.isGameOver) {
                binding.gameResult.text = "Game Over! Winner: ${gameState.winner}"
            }
        })
    }
}

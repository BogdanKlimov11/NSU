package com.example.tictactoe

import android.content.Intent
import android.os.Bundle
import android.widget.Button
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val playButton: Button = findViewById(R.id.btn_play)
        val exitButton: Button = findViewById(R.id.btn_exit)

        playButton.setOnClickListener {
            val intent = Intent(this, GameActivity::class.java)
            startActivity(intent)
        }

        exitButton.setOnClickListener {
            finish()
        }
    }
}

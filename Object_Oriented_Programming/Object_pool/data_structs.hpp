#pragma once

struct Point final {
    int x, y;

    int X() const { return x; }
    int Y() const { return y; }
};

class Board final {
private:
    int cols, rows;
    int* board;

public:
    Board() : cols(0), rows(0), board(nullptr) {}
    explicit Board(int cols_, int rows_) 
        : cols(cols_), rows(rows_), board(new int[cols_ * rows_]()) {}

    ~Board() { delete[] board; }

    int Cols() const { return cols; }
    int Rows() const { return rows; }
};

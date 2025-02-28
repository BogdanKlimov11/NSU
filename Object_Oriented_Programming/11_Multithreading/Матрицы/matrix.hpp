#pragma once

#include <cassert>
#include <iostream>

class Matrix final {
    size_t col = 0;
    size_t row = 0;
    double* matrix = nullptr;

    Matrix& change_lines(size_t line1, size_t line2, int sign = 1) {
        assert(sign == 1 || sign == -1);
        if (line1 >= rows() || line2 >= rows()) {
            throw std::out_of_range("out of range");
        }
        for (size_t k = 0; k < cols(); ++k) {
            double temp = coeffRef(line1, k) * sign;
            coeffRef(line1, k) = coeffRef(line2, k);
            coeffRef(line2, k) = temp;
        }
        return *this;
    }

    Matrix& clean() {
        row = 0;
        col = 0;
        delete[] matrix;
        matrix = nullptr;
        return *this;
    }

public:
    Matrix() = default;
    Matrix(size_t cols);
    Matrix(size_t rows, size_t cols);
    ~Matrix();

    Matrix(const Matrix& mat);

    Matrix operator*(const Matrix& mat) const;
    Matrix operator-(const Matrix& mat) const;
    Matrix operator+(const Matrix& mat) const;

    Matrix operator*(double value) const;
    Matrix operator/(double value) const;

    Matrix& operator=(const Matrix& mat);
    Matrix& operator*=(const Matrix& mat);
    Matrix& operator+=(const Matrix& mat);
    Matrix& operator-=(const Matrix& mat);

    Matrix& operator*=(double value);
    Matrix& operator/=(double value);

    bool isValid() const;

    void resize(size_t rows, size_t cols);

    const double& coeffRef(size_t rowIdx, size_t colIdx) const;
    double& coeffRef(size_t rowIdx, size_t colIdx);

    const double* data() const;
    double* data();

    size_t rows() const;
    size_t cols() const;

    Matrix& setIdentity();
    Matrix& setZero();
    Matrix& setConstants(double value);

    Matrix& setIdentity(size_t rows, size_t cols);
    Matrix& setZero(size_t rows, size_t cols);
    Matrix& setConstants(size_t rows, size_t cols, double value);

    Matrix diag() const;
    Matrix transpose() const;
    Matrix inverse() const;
    double det() const;

    static Matrix identity(size_t rows, size_t cols);
    static Matrix zeros(size_t rows, size_t cols);
    static Matrix constants(size_t rows, size_t cols, double value);

    friend Matrix operator*(double value, const Matrix& mat);
};

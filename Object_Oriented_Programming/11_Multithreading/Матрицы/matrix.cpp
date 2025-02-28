#include <cassert>
#include <iostream>
#include <cmath>
#include <cstring>

#include "matrix.hpp"

Matrix::Matrix(size_t cols) {
    resize(1, cols);
}

Matrix::Matrix(size_t rows, size_t cols) {
    row = rows;
    col = cols;
    matrix = new double[rows * cols];
}

Matrix::Matrix(const Matrix& mat) {
    row = mat.rows();
    col = mat.cols();
    matrix = new double[row * col];
    std::memcpy(matrix, mat.data(), sizeof(double) * row * col);
}

Matrix::~Matrix() {
    delete[] matrix;
}

Matrix& Matrix::operator=(const Matrix& mat) {
    resize(mat.rows(), mat.cols());
    if (isValid()) {
        std::memcpy(matrix, mat.data(), sizeof(double) * rows() * cols());
    }
    return *this;
}

Matrix& Matrix::operator*=(const Matrix& mat) {
    if (cols() != mat.rows() || rows() != mat.cols()) {
        return clean();
    }
    
    Matrix finalMatrix = Matrix::zeros(rows(), mat.cols());
    for (int i = 0; i < finalMatrix.rows(); i++) {
        for (int j = 0; j < finalMatrix.cols(); j++) {
            for (int k = 0; k < cols(); k++) {
                double temp = matrix[i * cols() + k] * mat.matrix[k * mat.cols() + j];
                finalMatrix.matrix[i * finalMatrix.cols() + j] += temp;
            }
        }
    }
    
    *this = finalMatrix;
    return *this;
}

Matrix& Matrix::operator+=(const Matrix& mat) {
    if (rows() != mat.rows() || cols() != mat.cols()) {
        return clean();
    }

    for (int i = 0; i < cols() * rows(); i++) {
        matrix[i] += mat.matrix[i];
    }

    return *this;
}

Matrix& Matrix::operator-=(const Matrix& mat) {
    if (rows() != mat.rows() || cols() != mat.cols()) {
        return clean();
    }

    for (int i = 0; i < cols() * rows(); i++) {
        matrix[i] -= mat.matrix[i];
    }

    return *this;
}

Matrix& Matrix::operator*=(double value) {
    for (int i = 0; i < cols() * rows(); i++) {
        matrix[i] *= value;
    }
    return *this;
}

Matrix& Matrix::operator/=(double value) {
    if (value == 0) {
        throw std::overflow_error("Division by zero");
    }

    for (int i = 0; i < cols() * rows(); i++) {
        matrix[i] /= value;
    }
    return *this;
}

Matrix Matrix::operator*(const Matrix& mat) const {
    if (cols() != mat.rows()) {
        return Matrix();
    }

    Matrix matrix(*this);
    matrix *= mat;
    return matrix;
}

Matrix Matrix::operator+(const Matrix& mat) const {
    if (rows() != mat.rows() || cols() != mat.cols()) {
        return Matrix();
    }

    Matrix matrix(*this);
    matrix += mat;
    return matrix;
}

Matrix Matrix::operator-(const Matrix& mat) const {
    if (rows() != mat.rows() || cols() != mat.cols()) {
        return Matrix();
    }

    Matrix matrix(*this);
    matrix -= mat;
    return matrix;
}

Matrix Matrix::operator*(double value) const {
    Matrix matrix(*this);
    matrix *= value;
    return matrix;
}

Matrix Matrix::operator/(double value) const {
    if (value == 0) {
        throw std::overflow_error("Division by zero");
    }

    Matrix matrix(*this);
    matrix /= value;
    return matrix;
}

bool Matrix::isValid() const {
    return row != 0 && col != 0 && matrix != NULL;
}

void Matrix::resize(size_t rows, size_t cols) {
    if (rows == 0 || cols == 0) {
        row = col = 0;
    }

    if (rows != this->rows() || cols != this->cols()) {
        row = 0;
        col = 0;
        delete[] matrix;
        matrix = NULL;
    } else {
        return;
    }

    row = rows;
    col = cols;

    if (this->rows() != 0 && this->cols() != 0) {
        matrix = new double[this->rows() * this->cols()];
    }
}

const double& Matrix::coeffRef(size_t rowIdx, size_t colIdx) const {
    if (rowIdx >= this->row || colIdx >= this->col) {
        throw std::out_of_range("Out of range");
    }

    const double& ref = matrix[rowIdx * cols() + colIdx];
    return ref;
}

double& Matrix::coeffRef(size_t rowIdx, size_t colIdx) {
    if (rowIdx >= this->row || colIdx >= this->col) {
        throw std::out_of_range("Out of range");
    }

    double& ref = matrix[rowIdx * cols() + colIdx];
    return ref;
}

const double* Matrix::data() const {
    return matrix;
}

double* Matrix::data() {
    return matrix;
}

size_t Matrix::rows() const {
    return row;
}

size_t Matrix::cols() const {
    return col;
}

Matrix& Matrix::setIdentity() {
    for (size_t i = 0; i < rows(); i++) {
        for (size_t j = 0; j < cols(); j++) {
            coeffRef(i, j) = i == j ? 1.0 : 0.0;
        }
    }
    return *this;
}

Matrix& Matrix::setZero() {
    return setConstants(0);
}

Matrix& Matrix::setConstants(double value) {
    for (size_t i = 0; i < rows() * cols(); i++) {
        matrix[i] = value;
    }
    return *this;
}

Matrix& Matrix::setIdentity(size_t rows, size_t cols) {
    this->resize(rows, cols);
    this->setIdentity();
    return *(this);
}

Matrix& Matrix::setZero(size_t rows, size_t cols) {
    this->resize(rows, cols);
    this->setZero();
    return *(this);
}

Matrix& Matrix::setConstants(size_t rows, size_t cols, double value) {
    this->resize(rows, cols);
    this->setConstants(value);
    return *(this);
}

Matrix Matrix::transpose() const {
    if (cols() != rows() || !isValid()) {
        return Matrix();
    }

    Matrix _matrix(cols(), rows());
    for (size_t i = 0; i < row; i++) {
        for (size_t j = 0; j < col; j++) {
            _matrix.coeffRef(j, i) = coeffRef(i, j);
        }
    }

    return _matrix;
}

Matrix Matrix::diag() const {
    if (cols() != rows() || !isValid()) {
        return Matrix();
    }

    Matrix _temp(*this);

    for (size_t j = 0; j < _temp.rows(); j++) {
        if (_temp.coeffRef(j, j) == 0) {
            size_t i = j + 1;
            for (; i < rows() && coeffRef(i, j) == 0; ++i);
            if (i == rows()) {
                return Matrix();
            }
            _temp.change_lines(i, j, -1);
        }

        for (size_t k = j + 1; k < _temp.rows(); k++) {
            double temp = -_temp.coeffRef(k, j) / _temp.coeffRef(j, j);
            for (size_t l = 0; l < _temp.cols(); l++) {
                _temp.coeffRef(k, l) += _temp.coeffRef(j, l) * temp;
            }
        }
    }

    for (long int j = _temp.rows() - 1; j >= 0; j--) {
        if (_temp.coeffRef(j, j) == 0) {
            long int i = j - 1;
            for (; i < rows() && coeffRef(i, j) == 0; i--);
            _temp.change_lines(i, j, -1);
        }

        for (long int k = j - 1; k >= 0; k--) {
            double temp = -_temp.coeffRef(k, j) / _temp.coeffRef(j, j);
            for (long int l = 0; l < _temp.cols(); l++) {
                _temp.coeffRef(k, l) += _temp.coeffRef(j, l) * temp;
            }
        }
    }

    return _temp;
}

Matrix Matrix::invert() const {
    return *this;
}

Matrix Matrix::zeros(size_t rows, size_t cols) {
    Matrix matrix;
    matrix.resize(rows, cols);
    matrix.setZero();
    return matrix;
}

void Matrix::change_lines(size_t first, size_t second, int sign) {
    for (size_t i = 0; i < cols(); i++) {
        double temp = matrix[first * cols() + i];
        matrix[first * cols() + i] = sign * matrix[second * cols() + i];
        matrix[second * cols() + i] = temp;
    }
}

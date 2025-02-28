#include <gtest/gtest.h>

#include <iostream>
#include <cstdlib>

#include "matrix.hpp"

#define err (1e-10)

double abs(double x) {
    if (x >= 0)
        return x;
    return -x;
}

bool near(double d1, double d2) {
    return abs(d1 - d2) <= err;
}

bool isDiagonal(const Matrix& m) {
    if (!m.isValid())
        return false;
    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            if ((i != j && !near(m.coeffRef(i, j), 0)) || (i == j && near(m.coeffRef(i, j), 0)))
                return false;
        }
    }
    return true;
}

void fillRand(Matrix& matrix) {
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            matrix.coeffRef(i, j) = rand();
        }
    }
}

Matrix diagonal(size_t dim, double diag = 1.0, double val = 0.0) {
    Matrix matrix{dim, dim};
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            matrix.coeffRef(i, j) = i == j ? diag : val;
        }
    }
    return matrix;
}

std::ostream& operator<<(std::ostream& out, const Matrix& m) {
    out << std::endl << "Matrix { rows: " << m.rows() << ", cols: " << m.cols() << " }" << std::endl;
    out << std::setprecision(5);
    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            out << m.coeffRef(i, j);
            if (j != m.cols() - 1)
                out << ", ";
        }
        out << std::endl;
    }
    return out;
}

inline bool operator==(const Matrix& matrix1, const Matrix& matrix2) {
    if (matrix1.rows() != matrix2.rows() || matrix1.cols() != matrix2.cols())
        return false;

    for (int i = 0; i < matrix1.rows(); ++i) {
        for (int j = 0; j < matrix1.cols(); ++j) {
            if (matrix1.coeffRef(i, j) - matrix2.coeffRef(i, j) > 1e-10)
                return false;
        }
    }
    return true;
}

TEST(Operators, MultNonSquare) {
    {
        const auto matrix = Matrix::identity(3, 5);
        const auto matrix1 = Matrix::identity(5, 3);
        auto result = matrix * matrix1;
        ASSERT_TRUE(result.isValid());
        ASSERT_EQ(result.rows(), matrix.rows());
        ASSERT_EQ(result.cols(), matrix1.cols());
        ASSERT_EQ(result.rows(), result.cols());

        auto data = result.data();
        for (size_t i = 0; i < result.rows(); ++i) {
            for (size_t j = 0; j < result.cols(); ++j) {
                EXPECT_EQ(data[i * result.cols() + j], (i == j ? 1.0 : 0.0)) << "error with i=" << i << " j=" << j << " must be=" << (i == j ? 1.0 : 0.0);
            }
        }
    }
    {
        auto matrix = Matrix::constants(5, 3, 1.0);
        auto matrix1 = Matrix::constants(3, 5, 1.0);
        auto result = matrix * matrix1;
        ASSERT_TRUE(result.isValid());
        ASSERT_EQ(result.rows(), matrix.rows());
        ASSERT_EQ(result.cols(), matrix1.cols());
        ASSERT_EQ(result.rows(), result.cols());

        auto data = result.data();
        for (size_t i = 0; i < matrix.rows(); ++i) {
            for (size_t j = 0; j < matrix.cols(); ++j) {
                EXPECT_NEAR(data[i * matrix.cols() + j], 3.0, 1e-10) << "error with i=" << i << " j=" << j << " must be=" << 5.0;
            }
        }
    }
}

TEST(Algebra, Determinant) {
    auto dims = {1u, 2u, 3u, 4u, 5u, 6u, 11u, 20u, 31u, 100u};
    for (auto dim : dims) {
        auto m = diagonal(dim, 0, 1);
        double result = ((double)(dim & 1) != 0 ? 1 : -1) * ((double)dim - 1);
        EXPECT_NEAR(m.det(), result, 1.0e-10);
    }
    {
        Matrix m(3, 3);
        double m_data[]{
            -1, 7, 4,
            3, 5, -10,
            2, -3, 9};
        std::copy(std::begin(m_data), std::end(m_data), m.data());
        EXPECT_DOUBLE_EQ(m.det(), -420.0);
    }
    {
        Matrix m(4, 4);
        double m_data[]{
            6, -3, 33, 18,
            -8, 9, -2, -101,
            -21, -1, 11, -8,
            0, 0, -16, 1};
        std::copy(std::begin(m_data), std::end(m_data), m.data());
        EXPECT_DOUBLE_EQ(m.det(), -51915.0);
    }
}

TEST(Algebra, Transpose) {
    auto dims = {1u, 2u, 3u, 4u, 5u};
    for (auto dim : dims) {
        Matrix m{dim, dim};
        fillRand(m);
        auto tm = m.transpose();
        for (int i = 0; i < m.rows(); ++i) {
            for (int j = 0; j < m.cols(); ++j) {
                EXPECT_NEAR(m.coeffRef(i, j), tm.coeffRef(j, i), 1e-10);
            }
        }
    }
}

TEST(Algebra, Inverse) {
    {
        Matrix i_expect(3, 3);
        double i_data[]{
            -0.5, 0.5, 0.5,
            0.5, -0.5, 0.5,
            0.5, 0.5, -0.5};
        std::copy(std::begin(i_data), std::end(i_data), i_expect.data());
        auto m = diagonal(3, 0, 1);
        auto im = m.inverse();
        EXPECT_EQ(im, i_expect) << "error when finding the inverse matrix for: " << m;
    }

    auto dims = {2u, 3u, 4u, 5u, 6u};
    for (auto dim : dims) {
        auto m = diagonal(dim, 0, 1);
        auto im = m.inverse();
        EXPECT_EQ(m * im, Matrix::identity(dim, dim)) << "error when finding the inverse matrix for: " << m << " result: " << im;
    }
}

TEST(Algebra, Diag) {
    {
        Matrix m = Matrix::identity(3, 3);
        auto d = m.diag();
        EXPECT_EQ(d, m) << "error when finding the diagonal for: " << m;
    }
    {
        Matrix m = diagonal(3, 3.0);
        auto d = m.diag();
        EXPECT_EQ(d, m) << "error when finding the diagonal for: " << m;
    }
    {
        Matrix m(3, 3);
        double m_data[]{
            0, 1, -2,
            0, 1, 0,
            1, -1, 3};
        std::copy(std::begin(m_data), std::end(m_data), m.data());
        auto diag = m.diag();
        Matrix d_expect(3, 3);
        double d_data[]{
            1, 0, 0,
            0, 1, 0,
            0, 0, 2};
        std::copy(std::begin(d_data), std::end(d_data), d_expect.data());
        EXPECT_EQ(diag, d_expect) << "error when finding the diagonal for: " << m;
    }
    {
        Matrix m(3, 3);
        double m_data[]{
            2, -2, 3,
            1, 1, 1,
            1, 3, -1};
        std::copy(std::begin(m_data), std::end(m_data), m.data());
        auto diag = m.diag();
        Matrix d_expect(3, 3);
        double d_data[]{
            2, 0, 0,
            0, 2, 0,
            0, 0, -3 / 2};
        std::copy(std::begin(d_data), std::end(d_data), d_expect.data());
        EXPECT_EQ(diag, d_expect) << "error when finding the diagonal for: " << m;
    }
    {
        Matrix m(3, 3);
        double m_data[]{
            0, -2, 3,
            0, 1, 1,
            0, 3, -1};
        std::copy(std::begin(m_data), std::end(m_data), m.data());
        auto diag = m.diag();
        EXPECT_FALSE(diag.isValid());
    }
}

bool compareMatrix(const Matrix& mat1, const Matrix& mat2) {
    if (mat1.rows() != mat2.rows() || mat1.cols() != mat2.cols())
        return false;

    for (size_t i = 0; i < mat1.rows(); i++) {
        for (size_t j = 0; j < mat1.cols(); j++) {
            if ((mat1.coeffRef(i, j) - mat2.coeffRef(i, j)) > 10e-5)
                return false;
        }
    }
    return true;
}

TEST(Constructors, Default) {
    const Matrix default_matrix{};
    ASSERT_FALSE(default_matrix.isValid());
}

TEST(Constructors, Vector) {
    const Matrix vector(20);
    ASSERT_TRUE(vector.isValid());
}

TEST(Constructors, VectorInvalid) {
    const Matrix vector(0);
    ASSERT_FALSE(vector.isValid());
}

TEST(Constructors, Full) {
    const Matrix matrix(3, 3);
    ASSERT_TRUE(matrix.isValid());
}

TEST(Constructors, FullInvalid) {
    const Matrix matrix1(0, 3);
    ASSERT_FALSE(matrix1.isValid());
    const Matrix matrix2(3, 0);
    ASSERT_FALSE(matrix2.isValid());
    const Matrix matrix3(0, 0);
    ASSERT_FALSE(matrix3.isValid());
}

TEST(Constructors, Copy) {
    const Matrix matrix1(3, 3);
    const Matrix matrix2(matrix1);
    ASSERT_TRUE(matrix2.isValid());
}

TEST(Constructors, CopyInvalid) {
    const Matrix matrix1(0, 3);
    const Matrix matrix2(matrix1);
    ASSERT_FALSE(matrix2.isValid());
}

TEST(Methods, Dimentions) {
    const Matrix matrix(3, 4);
    EXPECT_EQ(matrix.cols(), 4);
    EXPECT_EQ(matrix.rows(), 3);
}

TEST(Methods, Resize) {
    Matrix matrix(3, 4);
    matrix.resize(5, 5);
    EXPECT_EQ(matrix.cols(), 5);
    EXPECT_EQ(matrix.rows(), 5);

    matrix.resize(0, 0);
    ASSERT_FALSE(matrix.isValid());

    Matrix matrix1(5, 1);
    for (size_t i = 0; i < matrix1.rows(); i++) {
        for (size_t j = 0; j < matrix1.cols(); j++) {
            ASSERT_NO_THROW(matrix1.coeffRef(i, j) = i);
        }
    }
}

TEST(Operations, Plus) {
    const Matrix matrix1 = Matrix::constants(3, 3, 5);
    const Matrix matrix2 = Matrix::constants(3, 3, -4);
    const Matrix matrix3 = Matrix::constants(3, 3, 1);
    ASSERT_TRUE(compareMatrix(matrix1 + matrix2, matrix3));

    Matrix matrix4(2, 2);
    Matrix matrix5 = matrix1 + matrix4;
    ASSERT_FALSE(matrix5.isValid());
}

TEST(Operations, PlusAssign) {
    Matrix matrix1 = Matrix::constants(2, 2, 4);
    const Matrix matrix2 = Matrix::constants(2, 2, 6);
    const Matrix matrix3 = Matrix::constants(2, 2, 10);
    ASSERT_TRUE(compareMatrix(matrix1 += matrix2, matrix3));

    Matrix matrix4(3, 3);
    matrix1 += matrix4;
    ASSERT_FALSE(matrix1.isValid());
}

TEST(Operations, Minus) {
    const Matrix matrix1 = Matrix::constants(3, 3, 5);
    const Matrix matrix2 = Matrix::constants(3, 3, -4);
    const Matrix matrix3 = Matrix::constants(3, 3, 9);
    ASSERT_TRUE(compareMatrix(matrix1 - matrix2, matrix3));

    Matrix matrix4(2, 2);
    Matrix matrix5 = matrix1 - matrix4;
    ASSERT_FALSE(matrix5.isValid());
}

TEST(Operations, MinusAssign) {
    Matrix matrix1 = Matrix::constants(2, 2, 4);
    const Matrix matrix2 = Matrix::constants(2, 2, 6);
    const Matrix matrix3 = Matrix::constants(2, 2, -2);
    ASSERT_TRUE(compareMatrix(matrix1 -= matrix2, matrix3));

    Matrix matrix4(3, 3);
    matrix1 -= matrix4;
    ASSERT_FALSE(matrix1.isValid());
}

TEST(Operations, Multiplication) {
    const Matrix matrix1 = Matrix::constants(2, 3, 2);
    const Matrix matrix2 = Matrix::constants(3, 2, 2);
    const Matrix matrix3 = Matrix::constants(2, 2, 12);
    ASSERT_TRUE(compareMatrix(matrix1 * matrix2, matrix3));

    Matrix matrix4(5, 6);
    Matrix matrix5 = matrix1 * matrix4;
    ASSERT_FALSE(matrix5.isValid());
}

TEST(Operations, MultiplicationAssign) {
    Matrix matrix1 = Matrix::constants(2, 3, 2);
    const Matrix matrix2 = Matrix::constants(3, 2, 2);
    const Matrix matrix3 = Matrix::constants(2, 2, 12);
    matrix1 *= matrix2;
    ASSERT_TRUE(compareMatrix(matrix1, matrix3));

    Matrix matrix4(3, 3);
    matrix1 *= matrix4;
    ASSERT_FALSE(matrix1.isValid());
}

TEST(Operations, MultiplicationByValue) {
    Matrix matrix1 = Matrix::constants(2, 3, 2);
    matrix1 = matrix1 * 3;
    Matrix matrix2 = Matrix::constants(2, 3, 6);
    ASSERT_TRUE(compareMatrix(matrix1, matrix2));
}

TEST(Operations, DivisionByValue) {
    Matrix matrix1 = Matrix::constants(2, 3, 2);
    matrix1 = matrix1 / 2;
    Matrix matrix2 = Matrix::constants(2, 3, 1);
    ASSERT_TRUE(compareMatrix(matrix1, matrix2));
    EXPECT_THROW(matrix1 / 0, std::overflow_error);
}

TEST(Methods, CoefficientReference) {
    Matrix matrix1 = Matrix::constants(2, 3, 2);
    const double a = matrix1.coeffRef(1, 1);
    EXPECT_DOUBLE_EQ(a, 2);
    EXPECT_THROW(matrix1.coeffRef(7, 7), std::out_of_range);
}

TEST(Algebra, Det) {
    const Matrix matrix = Matrix::identity(1, 1);
    EXPECT_DOUBLE_EQ(matrix.det(), 1.0);

    const Matrix zero_matrix = Matrix::zeros(1, 1);
    EXPECT_DOUBLE_EQ(zero_matrix.det(), 0);

    const Matrix wrong_matrix = Matrix::identity(3, 4);
}

TEST(Algebra, inverse) {
    Matrix matrix = Matrix::constants(3, 3, 5);
    matrix = matrix.inverse();
    ASSERT_FALSE(matrix.isValid());

    Matrix ident_matrix = Matrix::identity(3, 3);
    ASSERT_TRUE(compareMatrix(ident_matrix.inverse(), ident_matrix));

    Matrix const_matrix = Matrix::constants(3, 3, 1);
    const_matrix += Matrix::identity(3, 3);
    Matrix inv_const_matrix = const_matrix.inverse();
    ASSERT_TRUE(compareMatrix(const_matrix * inv_const_matrix, ident_matrix));

    Matrix d_matrix = Matrix::constants(3, 3, 1);
    Matrix d_ident_matrix = Matrix::identity(3, 3);
    d_matrix -= d_ident_matrix;
    Matrix inv_d_matrix = d_matrix.inverse();
    ASSERT_TRUE(compareMatrix(d_matrix * inv_d_matrix, d_ident_matrix));
}

TEST(Algebra, diag_and_transpose) {
    Matrix const_matrix = Matrix::constants(3, 3, 1);
    const_matrix += Matrix::identity(3, 3);
    ASSERT_TRUE(const_matrix.diag().isValid());

    Matrix check_matrix = Matrix::zeros(3, 3);
    check_matrix.coeffRef(0, 0) = 2;
    check_matrix.coeffRef(1, 1) = 1.5;
    check_matrix.coeffRef(2, 2) = 1.33333;

    Matrix diag_matrix = const_matrix.diag();
    ASSERT_TRUE(compareMatrix(diag_matrix, check_matrix));

    {
        Matrix m(3, 3);
        double m_data[]{
            0, 1, -2,
            0, 1, 0,
            1, -1, 3};
        std::copy(std::begin(m_data), std::end(m_data), m.data());
        auto diag = m.diag();
        EXPECT_TRUE(diag.isValid());
        EXPECT_TRUE(isDiagonal(diag));
        EXPECT_NEAR(m.det(), diag.det(), err);
    }
    {
        Matrix m(3, 3);
        double m_data[]{
            2, -2, 3,
            1, 1, 1,
            1, 3, -1};
        std::copy(std::begin(m_data), std::end(m_data), m.data());
        auto diag = m.diag();
        EXPECT_TRUE(diag.isValid());
        EXPECT_TRUE(isDiagonal(diag));
        EXPECT_NEAR(m.det(), diag.det(), err);
    }
    {
        auto m = diagonal(11, 0, 1);
        auto diag = m.diag();
        EXPECT_TRUE(diag.isValid());
        EXPECT_TRUE(isDiagonal(diag));
        EXPECT_NEAR(m.det(), diag.det(), err);
    }
}

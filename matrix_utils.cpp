#include "matrix_utils.h"

Matrix transpose(const Matrix& A) {
    int rows = A.size();
    int cols = A[0].size();
    Matrix At(cols, Vector(rows));
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            At[j][i] = A[i][j];
    return At;
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    int n = A.size(), m = B[0].size(), p = B.size();
    Matrix C(n, Vector(m, 0.0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            for (int k = 0; k < p; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Vector multiply(const Matrix& A, const Vector& x) {
    int rows = A.size(), cols = A[0].size();
    Vector b(rows, 0.0);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            b[i] += A[i][j] * x[j];
    return b;
}

Matrix inverse2x2(const Matrix& A) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    Matrix inv(2, Vector(2));
    inv[0][0] =  A[1][1] / det;
    inv[0][1] = -A[0][1] / det;
    inv[1][0] = -A[1][0] / det;
    inv[1][1] =  A[0][0] / det;
    return inv;
}

Matrix subtract(const Matrix& A, const Matrix& B) {
    Matrix C(A.size(), Vector(A[0].size()));
    for (size_t i = 0; i < A.size(); ++i)
        for (size_t j = 0; j < A[0].size(); ++j)
            C[i][j] = A[i][j] - B[i][j];
    return C;
}

Vector subtract(const Vector& a, const Vector& b) {
    Vector c(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        c[i] = a[i] - b[i];
    return c;
}

Matrix add(const Matrix& A, const Matrix& B) {
    Matrix C(A.size(), Vector(A[0].size()));
    for (size_t i = 0; i < A.size(); ++i)
        for (size_t j = 0; j < A[0].size(); ++j)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

Vector add(const Vector& a, const Vector& b) {
    Vector c(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        c[i] = a[i] + b[i];
    return c;
}

Matrix outerProduct(const Vector& a, const Vector& b) {
    Matrix C(a.size(), Vector(b.size()));
    for (size_t i = 0; i < a.size(); ++i)
        for (size_t j = 0; j < b.size(); ++j)
            C[i][j] = a[i] * b[j];
    return C;
}

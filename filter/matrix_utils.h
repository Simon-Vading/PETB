#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include <vector>

using Vector = std::vector<double>;
using Matrix = std::vector<std::vector<double>>;

Matrix transpose(const Matrix& A);
Matrix multiply(const Matrix& A, const Matrix& B);
Vector multiply(const Matrix& A, const Vector& x);
Matrix inverse2x2(const Matrix& A);
Matrix subtract(const Matrix& A, const Matrix& B);
Vector subtract(const Vector& a, const Vector& b);
Matrix add(const Matrix& A, const Matrix& B);
Vector add(const Vector& a, const Vector& b);
Matrix outerProduct(const Vector& a, const Vector& b);

#endif

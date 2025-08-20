#ifndef FILTER_TEST_H
#define FILTER_TEST_H

#include "matrix_utils.h"
#include "filter.h"
#include <random>

Vector sampleGaussian(const Vector& mean, const Matrix& cov, std::mt19937& gen);
std::vector<Vector> generateStateVector(const Vector& x0,const Matrix& P0, MotionModel f, const Matrix& Q, int N);
std::vector<Vector> generateMeasurementVector(const std::vector<Vector>& state_vector, MotionModel h, const Matrix& R);

#endif
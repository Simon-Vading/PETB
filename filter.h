#ifndef FILTER_H
#define FILTER_H

#include "matrix_utils.h"

using StateSpaceModel = std::pair<Vector, Matrix>(*)(const Vector&);

void EKFprediction(Vector& x, Matrix& P, StateSpaceModel f, const Matrix& Q);
void EKFupdate(Vector& x, Matrix& P, const Vector& y, StateSpaceModel h, const Matrix& R);
void ExtendedKalmanFilter(
    Vector& x,             // State vector
    Matrix& P,             // State covariance
    const Vector& y,       // Measurement vector
    StateSpaceModel f,     // Motion model
    const Matrix& Q,       // Process noise covariance
    StateSpaceModel h,     // Measurement model
    const Matrix& R        // Measurement noise covariance
);

#endif

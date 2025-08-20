#ifndef FILTER_H
#define FILTER_H

#include "matrix_utils.h"

using MeasurementModel = std::pair<Vector, Matrix>(*)(const Vector&);
using MotionModel = std::pair<Vector, Matrix>(*)(const Vector&, const double T);

void EKFprediction(Vector& x, Matrix& P, MotionModel f, const Matrix& Q, const double T);
void EKFupdate(Vector& x, Matrix& P, const Vector& y, MeasurementModel h, const Matrix& R);
void ExtendedKalmanFilter(
    Vector& x,             // State vector
    Matrix& P,             // State covariance
    const Vector& y,       // Measurement vector
    MotionModel f,         // Motion model
    const Matrix& Q,       // Process noise covariance
    MeasurementModel h,    // Measurement model
    const Matrix& R,       // Measurement noise covariance
    const double T         // Sample time
);

#endif

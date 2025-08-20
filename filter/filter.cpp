#include <stdexcept>
#include "filter.h"


// EKF Prediction Step
void EKFprediction(Vector& x, Matrix& P, MotionModel f, const Matrix& Q, double T) {
    auto [fx, Fx] = f(x, T);  // Evaluate motion model and Jacobian

    x = fx; 

    // P = Fx * P * Fx' + Q
    Matrix FxP = multiply(Fx, P);
    Matrix FxT = transpose(Fx);
    Matrix FxP_FxT = multiply(FxP, FxT);
    P = add(FxP_FxT, Q);
}

// EKF Update Step
void EKFupdate(Vector& x, Matrix& P, const Vector& y, MeasurementModel h, const Matrix& R) {
    auto [hx, Hx] = h(x);  // Evaluate measurement model and Jacobian

    // S = Hx * P * Hx' + R
    Matrix HxP = multiply(Hx, P);
    Matrix HxT = transpose(Hx);
    Matrix S = multiply(HxP, HxT);
    S = add(S, R);

    // K = P * Hx' * inv(S)
    Matrix PHxT = multiply(P, HxT);
    Matrix S_inv;
    if (S.size() == 2 && S[0].size() == 2) {
        S_inv = inverse2x2(S);
    } else {
        throw std::runtime_error("Matrix inversion only implemented for 2x2");
    }

    Matrix K = multiply(PHxT, S_inv);

    // x = x + K * (y - hx)
    Vector innovation = subtract(y, hx);
    Vector K_innov = multiply(K, innovation);
    x = add(x, K_innov);

    // P = P - K * S * K'
    Matrix KS = multiply(K, S);
    Matrix KT = transpose(K);
    Matrix KSKT = multiply(KS, KT);
    P = subtract(P, KSKT);
}

void ExtendedKalmanFilter(
    Vector& x,             // State vector
    Matrix& P,             // State covariance
    const Vector& y,       // Measurement vector
    MotionModel f,         // Motion model
    const Matrix& Q,       // Process noise covariance
    MeasurementModel h,    // Measurement model
    const Matrix& R,       // Measurement noise covariance
    double T               // Sample time
) {
    EKFprediction(x, P, f, Q, T);
    EKFupdate(x, P, y, h, R);
}

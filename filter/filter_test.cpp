#include "filter_test.h"
#include "models.h"
#include <iostream>

#define T 0.1 //sample time
//#define n 4 //state vector length
//#define m 2 //measurment vector length

Vector sampleGaussian(const Vector& mean, const Matrix& cov, std::mt19937& gen) {
    int n = mean.size();
    Vector sample(n, 0.0);
    std::normal_distribution<> dist(0.0, 1.0);

    // Cholesky decomposition (lower triangular L so cov = L*L^T)
    Matrix L(n, Vector(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = cov[i][j];
            for (int k = 0; k < j; ++k) {
                sum -= L[i][k] * L[j][k];
            }
            if (i == j) {
                L[i][j] = std::sqrt(std::max(sum, 1e-12)); // avoid neg sqrt
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }

    // Generate standard normal vector z
    Vector z(n);
    for (int i = 0; i < n; ++i) z[i] = dist(gen);

    // sample = mean + L * z
    for (int i = 0; i < n; ++i) {
        sample[i] = mean[i];
        for (int j = 0; j <= i; ++j) {
            sample[i] += L[i][j] * z[j];
        }
    }

    return sample;
}

std::vector<Vector> generateStateVector(Vector& x0, Matrix& P0, MotionModel f, Matrix& Q, int N){
    int n = x0.size();
    std::vector<Vector> X(N + 1, Vector(n));

    std::random_device rd;
    std::mt19937 gen(rd());

    // Initial state: x0 ~ N(x0, P0)
    X[0] = sampleGaussian(x0, P0, gen);

    // Noise is zero-mean Gaussian
    Vector mu(n, 0.0);

    for (int i = 1; i <= N; ++i) {
        auto [fx, Fx] = f(X[i - 1], T);
        Vector q = sampleGaussian(mu, Q, gen);

        Vector next(n);
        for (int k = 0; k < n; ++k) {
            next[k] = fx[k] + q[k];
        }
        X[i] = next;
    }

    return X;
}

std::vector<Vector> generateMeasurementVector(const std::vector<Vector>& X, MeasurementModel h, const Matrix& R) {
    int N = X.size() - 1;      // number of measurements
    int m = R.size();          // measurement dimension

    std::vector<Vector> Y(N, Vector(m, 0.0));

    std::random_device rd;
    std::mt19937 gen(rd());

    Vector mu(m, 0.0);

    for (int i = 0; i < N; ++i) {
        // Evaluate measurement model
        auto [hx, Hx] = h(X[i + 1]);

        // Add Gaussian measurement noise
        Vector noise = sampleGaussian(mu, R, gen);

        Vector yi(m);
        for (int j = 0; j < m; ++j) {
            yi[j] = hx[j] + noise[j];
        }

        Y[i] = yi;
    }

    return Y;
}



int main() {
    Vector x0 = {1.0, 3.0, 0.0, 0.0};
    Matrix P0 = {{1.0, 0.0, 0.0, 0.0}, 
                 {0.0, 1.0, 0.0, 0.0}, 
                 {0.0, 0.0, 1.0, 0.0}, 
                 {0.0, 0.0, 0.0, 1.0}};
    Matrix Q = {{0.01, 0.0, 0.0, 0.0}, 
                {0.0, 0.01, 0.0, 0.0}, 
                {0.0, 0.0, 0.01, 0.0}, 
                {0.0, 0.0, 0.0, 0.01}};
    Matrix R = {{0.1, 0.0}, 
                {0.0, 0.1}};

    int N = 15;
    auto X = generateStateVector(x0, P0, randomWalkBearingModel, Q, N);
    auto Y = generateMeasurementVector(X, rangeBearingModel, R);

    // Initial estimate
    Vector x_est = x0;
    Matrix P_est = P0;
    for (int i = 0; i < N; ++i) {  // only N steps (not N+1!)
        ExtendedKalmanFilter(x_est, P_est, Y[i],
                             randomWalkBearingModel, Q,
                             rangeBearingModel, R, T);

        std::cout << "Step " << i+1 << ":\n";
        std::cout << " True state: [";
        for (size_t j = 0; j < 2; ++j) {
            std::cout << X[i+1][j] << (j+1 == 2 ? "" : ", ");
        }
        std::cout << "]\n";

        std::cout << " Filtered: [";
        for (size_t j = 0; j < 2; ++j) {
            std::cout << x_est[j] << (j+1 == 2 ? "" : ", ");
        }
        std::cout << "]\n";
    }
    return 0;
}

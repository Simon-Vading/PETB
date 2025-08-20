#include "models.h"

std::pair<Vector, Matrix> rangeBearingModel(const Vector& x) {
    double range = std::sqrt(x[0]*x[0]+x[1]*x[1]);
    double bearing = std::atan2(x[1],x[0]);
    Vector h = {range,bearing};

    Matrix H = {
        {x[0]/range, x[1]/range},
        {-x[1]/(range*range), x[0]/(range*range)}
    };

    return {h,H};
}
std::pair<Vector, Matrix> randomWalkBearingModel(const Vector& x, double T) {
    Vector f = {x[0] + T*x[2]*std::sin(x[3]),
                x[1] + T*x[2]*std::cos(x[3]),
                x[2],
                x[3]};
    Matrix F = {{1,0,T*std::cos(x[3]),-T*x[3]*std::sin(x[3])},
                {0,1,T*std::sin(x[3]), T*x[3]*std::cos(x[3])},
                {0,0,1,0},
                {0,0,0,1}};

    return {f,F};
}
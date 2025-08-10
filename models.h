#ifndef MODELS_H
#define MODELS_H

#include "matrix_utils.h"

std::pair<Vector, Matrix> rangeBearingModel(const Vector& x);
std::pair<Vector, Matrix> randomWalkBearingModel(const Vector& x);

#endif
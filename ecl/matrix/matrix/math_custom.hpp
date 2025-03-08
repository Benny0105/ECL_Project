#pragma once

#include <cmath>   // for sqrtf, etc.
#include <float.h> // for FLT_MIN, FLT_MAX

namespace matrix {
namespace math {

inline float radians(float deg)
{
    return deg * M_PI / 180.f;
}

inline float constrain(float val, float min_val, float max_val)
{
    if (val < min_val) { return min_val; }
    else if (val > max_val) { return max_val; }
    return val;
}

inline float max(float a, float b)
{
    return (a > b) ? a : b;
}

} // namespace math
} // namespace matrix


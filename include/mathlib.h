#pragma once
#include <cmath>
#include <float.h> // for FLT_MIN, FLT_MAX, etc.

namespace math
{

inline float radians(float deg) {
    return deg * static_cast<float>(M_PI) / 180.f;
}

template<typename T>
inline T constrain(const T val, const T min_val, const T max_val)
{
    if (val < min_val) return min_val;
    else if (val > max_val) return max_val;
    else return val;
}

template<typename T>
inline T max(const T &a, const T &b)
{
    return (a > b) ? a : b;
}

} // namespace math


#pragma once

#include <mathlib/mathlib.h>

namespace math
{
    constexpr float PI = M_PI;
    constexpr float PI_2 = M_PI_2;
    
    // 只保留角度和弧度轉換函數
    constexpr float radians(float degrees) {
        return degrees * (PI / 180.0f);
    }
    
    constexpr float degrees(float radians) {
        return radians * (180.0f / PI);
    }
} 
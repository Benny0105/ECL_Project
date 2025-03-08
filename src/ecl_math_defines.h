#ifndef ECL_MATH_DEFINES_H
#define ECL_MATH_DEFINES_H

#include <cmath>  // 標準庫中的數學函數

// 定義數學常量
#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#ifndef M_PI_2_F
#define M_PI_2_F (M_PI_F / 2.0f)
#endif

// 提供 math::constrain 的兼容性定義
namespace math {
    template<typename T>
    constexpr T constrain(T val, T min, T max) {
        return (val < min) ? min : ((val > max) ? max : val);
    }
    
    // 添加 math::max 函數
    template<typename T>
    constexpr T max(const T x, const T y) {
        return (x > y) ? x : y;
    }
    
    // 為安全起見，也添加 math::min 函數
    template<typename T>
    constexpr T min(const T x, const T y) {
        return (x < y) ? x : y;
    }
    
    // 添加 math::radians 和 math::degrees 函數
    template<typename T>
    constexpr T radians(T degrees) {
        return (degrees / T(180)) * T(M_PI);
    }
    
    template<typename T>
    constexpr T degrees(T radians) {
        return (radians * T(180)) / T(M_PI);
    }
}

// 修正全局 powf 函數，使用 std::pow 而不是 std::powf
inline float powf(float x, float y) {
    return std::pow(x, y);
}

#endif // ECL_MATH_DEFINES_H
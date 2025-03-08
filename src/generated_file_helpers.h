#ifndef GENERATED_FILE_HELPERS_H
#define GENERATED_FILE_HELPERS_H

#include <matrix/matrix/math.hpp>
#include <cmath>

// 常見變量定義
float magE = 0.0f;
float magD = 0.0f;
float magN = 0.0f;
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float R_MAG = 0.0f;

// 使用標準庫的 powf
using std::powf;

// 如果需要，為 Matrix::at 提供一個簡單的替代
template<int i>
float at() { return 0.0f; }

#endif // GENERATED_FILE_HELPERS_H 
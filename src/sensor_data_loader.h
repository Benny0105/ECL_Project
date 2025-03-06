#pragma once

#include <vector>
#include <string>

// (A) 暫存 vehicle_imu 資料 (加速度 & 角速度)
struct ImuData {
    double timestamp;  // us (microseconds)
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
};

// (B) 暫存 vehicle_air_data 資料 (氣壓高度)
struct AirData {
    double timestamp;   // us
    float baro_alt_m;   // 由 vehicle_air_data 產生的 baro altitude
};

// (C) 暫存 vehicle_gps_position 資料 (GPS)
struct GpsData {
    double timestamp; // us
    double lat, lon;  // degree
    float alt;        // m (海拔)
    float vel_n_m_s;  // 北向速度
    float vel_e_m_s;  // 東向速度
    float vel_d_m_s;  // 垂直速度
};

// (D) 統合後，EKF 每個時刻需要的資料
struct SensorData {
    double timestamp_s;    // 以「秒」為單位 => 將 us 轉成 s

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;

    float baro_alt;   // 氣壓高度
    double lat, lon;  // GPS (度)
    float gps_alt;    // GPS 高度 (m)
    float velN, velE, velD; // GPS NED 速度 (m/s)
};

// 函式宣告
std::vector<SensorData> mergeThreeCsv(
    const std::string &imu_csv_path,
    const std::string &airdata_csv_path,
    const std::string &gps_csv_path);



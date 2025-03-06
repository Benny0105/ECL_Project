#include "sensor_data_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>

// 輔助函式：讀 vehicle_imu CSV
static std::vector<ImuData> loadImuCsv(const std::string &path) {
    std::vector<ImuData> imu_list;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Cannot open IMU csv: " << path << std::endl;
        return imu_list;
    }
    // 假設第一行是標頭
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);

        ImuData imu;
        // 例如 vehicle_imu 欄位 (只是示例，你要依實際欄位順序)
        // timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        char comma;
        ss >> imu.timestamp >> comma
           >> imu.accel_x >> comma
           >> imu.accel_y >> comma
           >> imu.accel_z >> comma
           >> imu.gyro_x  >> comma
           >> imu.gyro_y  >> comma
           >> imu.gyro_z;

        imu_list.push_back(imu);
    }
    return imu_list;
}

// 讀 vehicle_air_data CSV
static std::vector<AirData> loadAirDataCsv(const std::string &path) {
    std::vector<AirData> air_list;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Cannot open AirData csv: " << path << std::endl;
        return air_list;
    }
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);

        AirData ad;
        char comma;
        // ex: timestamp, baro_alt_meter
        ss >> ad.timestamp >> comma
           >> ad.baro_alt_m;
        air_list.push_back(ad);
    }
    return air_list;
}

// 讀 vehicle_gps_position CSV
static std::vector<GpsData> loadGpsCsv(const std::string &path) {
    std::vector<GpsData> gps_list;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Cannot open GPS csv: " << path << std::endl;
        return gps_list;
    }
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);

        GpsData gd;
        char comma;
        // ex: timestamp, lat, lon, alt, vel_n_m_s, vel_e_m_s, vel_d_m_s
        ss >> gd.timestamp >> comma
           >> gd.lat >> comma
           >> gd.lon >> comma
           >> gd.alt >> comma
           >> gd.vel_n_m_s >> comma
           >> gd.vel_e_m_s >> comma
           >> gd.vel_d_m_s;
        gps_list.push_back(gd);
    }
    return gps_list;
}

// 合併三種資料 => 輸出給 EKF 用
std::vector<SensorData> mergeThreeCsv(
    const std::string &imu_csv_path,
    const std::string &airdata_csv_path,
    const std::string &gps_csv_path)
{
    auto imu_list = loadImuCsv(imu_csv_path);
    auto air_list = loadAirDataCsv(airdata_csv_path);
    auto gps_list = loadGpsCsv(gps_csv_path);

    // (A) 確保依 timestamp 排序
    auto cmp_imu = [](const ImuData &a, const ImuData &b){return a.timestamp < b.timestamp;};
    std::sort(imu_list.begin(), imu_list.end(), cmp_imu);

    auto cmp_air = [](const AirData &a, const AirData &b){return a.timestamp < b.timestamp;};
    std::sort(air_list.begin(), air_list.end(), cmp_air);

    auto cmp_gps = [](const GpsData &a, const GpsData &b){return a.timestamp < b.timestamp;};
    std::sort(gps_list.begin(), gps_list.end(), cmp_gps);

    // (B) 依 IMU 為主時間序列，每筆都找"最接近"的 air_data & gps
    //     可改成插值，但以下示範最近值
    std::vector<SensorData> out;
    out.reserve(imu_list.size());

    size_t idxAir = 0;
    size_t idxGps = 0;
    for (auto &imu : imu_list) {
        double t_imu = imu.timestamp; // us

        // 找最接近的 air
        while (idxAir + 1 < air_list.size() &&
               std::fabs((double)air_list[idxAir+1].timestamp - t_imu) <
               std::fabs((double)air_list[idxAir].timestamp - t_imu)) {
            idxAir++;
        }

        // 找最接近的 gps
        while (idxGps + 1 < gps_list.size() &&
               std::fabs((double)gps_list[idxGps+1].timestamp - t_imu) <
               std::fabs((double)gps_list[idxGps].timestamp - t_imu)) {
            idxGps++;
        }

        // 整合
        SensorData sd;
        // timestamp_s = us -> s
        sd.timestamp_s = t_imu * 1e-6;

        sd.accel_x = imu.accel_x;
        sd.accel_y = imu.accel_y;
        sd.accel_z = imu.accel_z;
        sd.gyro_x  = imu.gyro_x;
        sd.gyro_y  = imu.gyro_y;
        sd.gyro_z  = imu.gyro_z;

        sd.baro_alt = air_list.empty() ? 0.0f : air_list[idxAir].baro_alt_m;

        sd.lat = gps_list.empty() ? 0.0 : gps_list[idxGps].lat;
        sd.lon = gps_list.empty() ? 0.0 : gps_list[idxGps].lon;
        sd.gps_alt = gps_list.empty() ? 0.0f : gps_list[idxGps].alt;
        sd.velN = gps_list.empty() ? 0.0f : gps_list[idxGps].vel_n_m_s;
        sd.velE = gps_list.empty() ? 0.0f : gps_list[idxGps].vel_e_m_s;
        sd.velD = gps_list.empty() ? 0.0f : gps_list[idxGps].vel_d_m_s;

        out.push_back(sd);
    }

    return out;
}


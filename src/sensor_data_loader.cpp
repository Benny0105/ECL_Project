#include "sensor_data_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>

SensorDataLoader::SensorType SensorDataLoader::identifySensorType(const std::string& csv_path) {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "無法打開文件: " << csv_path << std::endl;
        return UNKNOWN;
    }
    
    std::string header_line;
    std::getline(file, header_line);
    
    // 檢查標題行特徵
    if (header_line.find("delta_angle") != std::string::npos &&
        header_line.find("delta_velocity") != std::string::npos) {
        return IMU;
    } else if (header_line.find("baro_alt_meter") != std::string::npos ||
               header_line.find("baro_pressure_pa") != std::string::npos) {
        return BARO;
    } else if (header_line.find("latitude_deg") != std::string::npos ||
               header_line.find("longitude_deg") != std::string::npos ||
               header_line.find("vel_n_m_s") != std::string::npos) {
        return GPS;
    } else if (header_line.find("magnetometer_ga") != std::string::npos) {
        return MAG;
    }
    
    return UNKNOWN;
}

// 解析CSV標頭，返回列名到索引的映射
std::unordered_map<std::string, int> parseHeader(const std::string& header_line) {
    std::unordered_map<std::string, int> header_map;
    std::stringstream ss(header_line);
    std::string column_name;
    int index = 0;
    
    while (std::getline(ss, column_name, ',')) {
        header_map[column_name] = index++;
    }
    
    return header_map;
}

// 解析CSV行
std::vector<std::string> parseCsvLine(const std::string& line) {
        std::vector<std::string> fields;
        std::stringstream ss(line);
        std::string field;
        
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }
        
    return fields;
}

std::vector<ImuSample> SensorDataLoader::loadImuData(const std::string& csv_path) {
    std::vector<ImuSample> imu_data;
    std::ifstream file(csv_path);
    
    if (!file.is_open()) {
        std::cerr << "無法打開IMU文件: " << csv_path << std::endl;
        return imu_data;
    }
    
    std::string header_line;
    std::getline(file, header_line);
    std::cout << "IMU CSV標題行: " << header_line << std::endl;
    
    auto header_map = parseHeader(header_line);
    
    // 檢查是否包含所需列
    bool has_required_fields = 
        header_map.count("timestamp") &&
        header_map.count("delta_angle[0]") &&
        header_map.count("delta_angle[1]") &&
        header_map.count("delta_angle[2]") &&
        header_map.count("delta_velocity[0]") &&
        header_map.count("delta_velocity[1]") &&
        header_map.count("delta_velocity[2]") &&
        header_map.count("delta_angle_dt") &&
        header_map.count("delta_velocity_dt");
    
    if (!has_required_fields) {
        std::cerr << "IMU文件缺少必要列" << std::endl;
        return imu_data;
    }
    
    // 獲取列索引
    int timestamp_idx = header_map["timestamp"];
    int delta_angle_x_idx = header_map["delta_angle[0]"];
    int delta_angle_y_idx = header_map["delta_angle[1]"];
    int delta_angle_z_idx = header_map["delta_angle[2]"];
    int delta_vel_x_idx = header_map["delta_velocity[0]"];
    int delta_vel_y_idx = header_map["delta_velocity[1]"];
    int delta_vel_z_idx = header_map["delta_velocity[2]"];
    int delta_angle_dt_idx = header_map["delta_angle_dt"];
    int delta_vel_dt_idx = header_map["delta_velocity_dt"];
    
    std::string line;
    while (std::getline(file, line)) {
        auto fields = parseCsvLine(line);
        
        // 確保有足夠的字段
        if (fields.size() <= static_cast<size_t>(std::max({
            delta_angle_x_idx, delta_angle_y_idx, delta_angle_z_idx,
            delta_vel_x_idx, delta_vel_y_idx, delta_vel_z_idx,
            delta_angle_dt_idx, delta_vel_dt_idx
        }))) {
            continue;
        }
        
        try {
            ImuSample sample;
            sample.timestamp_us = std::stoull(fields[timestamp_idx]);
            sample.delta_angle[0] = std::stof(fields[delta_angle_x_idx]);
            sample.delta_angle[1] = std::stof(fields[delta_angle_y_idx]);
            sample.delta_angle[2] = std::stof(fields[delta_angle_z_idx]);
            sample.delta_velocity[0] = std::stof(fields[delta_vel_x_idx]);
            sample.delta_velocity[1] = std::stof(fields[delta_vel_y_idx]);
            sample.delta_velocity[2] = std::stof(fields[delta_vel_z_idx]);
            sample.delta_angle_dt = std::stof(fields[delta_angle_dt_idx]);
            sample.delta_velocity_dt = std::stof(fields[delta_vel_dt_idx]);
            
            imu_data.push_back(sample);
        } catch (const std::exception& e) {
            std::cerr << "解析IMU數據時出錯: " << e.what() << std::endl;
        }
    }
    
    std::cout << "成功讀取 " << imu_data.size() << " 條IMU數據" << std::endl;
    if (!imu_data.empty()) {
        const auto& first = imu_data.front();
        std::cout << "第一條IMU數據：時間戳=" << first.timestamp_us
                  << ", 角度增量=[" << first.delta_angle[0] << "," << first.delta_angle[1] << "," << first.delta_angle[2] << "]"
                  << ", 速度增量=[" << first.delta_velocity[0] << "," << first.delta_velocity[1] << "," << first.delta_velocity[2] << "]"
                  << ", dt=" << first.delta_angle_dt << std::endl;
    }
    
    return imu_data;
}

std::vector<BaroSample> SensorDataLoader::loadBaroData(const std::string& csv_path) {
    std::vector<BaroSample> baro_data;
    std::ifstream file(csv_path);
    
    if (!file.is_open()) {
        std::cerr << "無法打開氣壓計文件: " << csv_path << std::endl;
        return baro_data;
    }
    
    std::string header_line;
    std::getline(file, header_line);
    std::cout << "氣壓計CSV標題行: " << header_line << std::endl;
    
    auto header_map = parseHeader(header_line);
    
    // 檢查氣壓計文件格式
    bool has_baro_fields = 
        header_map.count("timestamp") &&
        header_map.count("baro_pressure_pa") &&
        header_map.count("baro_alt_meter");
    
    // 如果找不到氣壓計字段，嘗試PX4 air_data格式
    if (!has_baro_fields && header_map.count("timestamp")) {
        // 嘗試其他可能的列名
        if (header_map.count("pressure_pa") || header_map.count("altitude_msl_m")) {
            has_baro_fields = true;
        }
    }
    
    if (!has_baro_fields) {
        std::cerr << "氣壓計文件格式錯誤" << std::endl;
        return baro_data;
    }
    
    // 獲取列索引
    int timestamp_idx = header_map["timestamp"];
    int pressure_idx = header_map.count("baro_pressure_pa") ? header_map["baro_pressure_pa"] : 
                     (header_map.count("pressure_pa") ? header_map["pressure_pa"] : -1);
    int altitude_idx = header_map.count("baro_alt_meter") ? header_map["baro_alt_meter"] : 
                     (header_map.count("altitude_msl_m") ? header_map["altitude_msl_m"] : -1);
    
    std::string line;
    while (std::getline(file, line)) {
        auto fields = parseCsvLine(line);
        
        // 確保有足夠的字段
        if (fields.size() <= static_cast<size_t>(std::max({timestamp_idx, pressure_idx, altitude_idx}))) {
            continue;
        }
        
        try {
            BaroSample sample;
            sample.timestamp_us = std::stoull(fields[timestamp_idx]);
            
            if (pressure_idx >= 0) {
                sample.pressure_pa = std::stof(fields[pressure_idx]);
            } else {
                sample.pressure_pa = 0.0f;
            }
            
            if (altitude_idx >= 0) {
                sample.altitude_m = std::stof(fields[altitude_idx]);
            } else {
                sample.altitude_m = 0.0f;
            }
            
            baro_data.push_back(sample);
        } catch (const std::exception& e) {
            std::cerr << "解析氣壓計數據時出錯: " << e.what() << std::endl;
        }
    }
    
    std::cout << "成功讀取 " << baro_data.size() << " 條氣壓計數據" << std::endl;
    if (!baro_data.empty()) {
        const auto& first = baro_data.front();
        std::cout << "第一條氣壓計數據：時間戳=" << first.timestamp_us
                  << ", 壓力=" << first.pressure_pa 
                  << ", 高度=" << first.altitude_m << std::endl;
    }
    
    return baro_data;
}

std::vector<GpsSample> SensorDataLoader::loadGpsData(const std::string& csv_path) {
    std::vector<GpsSample> gps_data;
    std::ifstream file(csv_path);
    
    if (!file.is_open()) {
        std::cerr << "無法打開GPS文件: " << csv_path << std::endl;
        return gps_data;
    }
    
    std::string header_line;
    std::getline(file, header_line);
    std::cout << "GPS CSV標題行: " << header_line << std::endl;
    
    auto header_map = parseHeader(header_line);
    
    // 檢查是否包含所需列
    bool has_gps_fields = 
        header_map.count("timestamp") &&
        (header_map.count("latitude_deg") || header_map.count("lat")) &&
        (header_map.count("longitude_deg") || header_map.count("lon")) &&
        (header_map.count("altitude_msl_m") || header_map.count("alt"));
    
    if (!has_gps_fields) {
        std::cerr << "GPS文件格式錯誤" << std::endl;
        return gps_data;
    }
    
    // 獲取列索引
    int timestamp_idx = header_map["timestamp"];
    int lat_idx = header_map.count("latitude_deg") ? header_map["latitude_deg"] : header_map["lat"];
    int lon_idx = header_map.count("longitude_deg") ? header_map["longitude_deg"] : header_map["lon"];
    int alt_idx = header_map.count("altitude_msl_m") ? header_map["altitude_msl_m"] : header_map["alt"];
    
    // 可選的速度列
    int vel_n_idx = header_map.count("vel_n_m_s") ? header_map["vel_n_m_s"] : -1;
    int vel_e_idx = header_map.count("vel_e_m_s") ? header_map["vel_e_m_s"] : -1;
    int vel_d_idx = header_map.count("vel_d_m_s") ? header_map["vel_d_m_s"] : -1;
    
    // 可選的精度列
    int eph_idx = header_map.count("eph") ? header_map["eph"] : -1;
    int epv_idx = header_map.count("epv") ? header_map["epv"] : -1;
    int fix_type_idx = header_map.count("fix_type") ? header_map["fix_type"] : -1;
    int nsats_idx = header_map.count("satellites_used") ? header_map["satellites_used"] : -1;
    
    std::string line;
    while (std::getline(file, line)) {
        auto fields = parseCsvLine(line);
        
        // 確保有足夠的字段
        if (fields.size() <= static_cast<size_t>(std::max({lat_idx, lon_idx, alt_idx}))) {
            continue;
        }
        
        try {
            GpsSample sample;
            sample.timestamp_us = std::stoull(fields[timestamp_idx]);
            sample.lat = std::stod(fields[lat_idx]);
            sample.lon = std::stod(fields[lon_idx]);
            sample.alt = std::stof(fields[alt_idx]);
            
            // 讀取速度，如果可用
            if (vel_n_idx >= 0 && vel_e_idx >= 0 && vel_d_idx >= 0 &&
                static_cast<size_t>(std::max({vel_n_idx, vel_e_idx, vel_d_idx})) < fields.size()) {
                sample.vel_ned[0] = std::stof(fields[vel_n_idx]);
                sample.vel_ned[1] = std::stof(fields[vel_e_idx]);
                sample.vel_ned[2] = std::stof(fields[vel_d_idx]);
            } else {
                sample.vel_ned.setZero();
            }
            
            // 讀取精度信息，如果可用
            if (eph_idx >= 0 && static_cast<size_t>(eph_idx) < fields.size()) {
                sample.eph = std::stof(fields[eph_idx]);
            } else {
                sample.eph = 2.0f; // 默認值
            }
            
            if (epv_idx >= 0 && static_cast<size_t>(epv_idx) < fields.size()) {
                sample.epv = std::stof(fields[epv_idx]);
            } else {
                sample.epv = 3.0f; // 默認值
            }
            
            if (fix_type_idx >= 0 && static_cast<size_t>(fix_type_idx) < fields.size()) {
                sample.fix_type = std::stoi(fields[fix_type_idx]);
            } else {
                sample.fix_type = 3; // 默認為3D定位
            }
            
            if (nsats_idx >= 0 && static_cast<size_t>(nsats_idx) < fields.size()) {
                sample.nsats = std::stoi(fields[nsats_idx]);
            } else {
                sample.nsats = 10; // 默認值
            }
            
            sample.sacc = 0.5f; // 默認值
            
            // 檢查有效性
            if (sample.lat >= -90 && sample.lat <= 90 && 
                sample.lon >= -180 && sample.lon <= 180) {
                gps_data.push_back(sample);
            }
        } catch (const std::exception& e) {
            std::cerr << "解析GPS數據時出錯: " << e.what() << std::endl;
        }
    }
    
    std::cout << "成功讀取 " << gps_data.size() << " 條GPS數據" << std::endl;
    if (!gps_data.empty()) {
        const auto& first = gps_data.front();
        std::cout << "第一條GPS數據：時間戳=" << first.timestamp_us
                  << ", 經緯度=(" << first.lat << "," << first.lon << ")"
                  << ", 高度=" << first.alt 
                  << ", 速度=[" << first.vel_ned[0] << "," << first.vel_ned[1] << "," << first.vel_ned[2] << "]" << std::endl;
    }
    
    return gps_data;
}

std::vector<MagSample> SensorDataLoader::loadMagData(const std::string& csv_path) {
    std::vector<MagSample> mag_data;
    std::ifstream file(csv_path);
    
    if (!file.is_open()) {
        std::cerr << "無法打開磁力計文件: " << csv_path << std::endl;
        return mag_data;
    }
    
    std::string header_line;
    std::getline(file, header_line);
    std::cout << "磁力計CSV標題行: " << header_line << std::endl;
    
    auto header_map = parseHeader(header_line);
    
    // 檢查是否包含所需列
    bool has_mag_fields = 
        header_map.count("timestamp") &&
        header_map.count("magnetometer_ga[0]") &&
        header_map.count("magnetometer_ga[1]") &&
        header_map.count("magnetometer_ga[2]");
    
    if (!has_mag_fields) {
        std::cerr << "磁力計文件格式錯誤" << std::endl;
        return mag_data;
    }
    
    // 獲取列索引
    int timestamp_idx = header_map["timestamp"];
    int mag_x_idx = header_map["magnetometer_ga[0]"];
    int mag_y_idx = header_map["magnetometer_ga[1]"];
    int mag_z_idx = header_map["magnetometer_ga[2]"];
    
    std::string line;
    while (std::getline(file, line)) {
        auto fields = parseCsvLine(line);
        
        // 確保有足夠的字段
        if (fields.size() <= static_cast<size_t>(std::max({mag_x_idx, mag_y_idx, mag_z_idx}))) {
            continue;
        }
        
        try {
            MagSample sample;
            sample.timestamp_us = std::stoull(fields[timestamp_idx]);
            sample.mag[0] = std::stof(fields[mag_x_idx]);
            sample.mag[1] = std::stof(fields[mag_y_idx]);
            sample.mag[2] = std::stof(fields[mag_z_idx]);
            
            mag_data.push_back(sample);
        } catch (const std::exception& e) {
            std::cerr << "解析磁力計數據時出錯: " << e.what() << std::endl;
        }
    }
    
    std::cout << "成功讀取 " << mag_data.size() << " 條磁力計數據" << std::endl;
    if (!mag_data.empty()) {
        const auto& first = mag_data.front();
        std::cout << "第一條磁力計數據：時間戳=" << first.timestamp_us
                  << ", 磁場=[" << first.mag[0] << "," << first.mag[1] << "," << first.mag[2] << "]" << std::endl;
    }
    
    return mag_data;
}

bool SensorDataLoader::loadAllSensorData(int argc, char** argv,
                                          std::vector<ImuSample>& imu_data,
                                          std::vector<BaroSample>& baro_data,
                                          std::vector<GpsSample>& gps_data,
                                          std::vector<MagSample>& mag_data) {
    if (argc < 5) {
        std::cerr << "使用方法: " << argv[0] 
                  << " <IMU_CSV> <BARO_CSV> <GPS_CSV> <MAG_CSV>" << std::endl;
        return false;
    }
    
    std::vector<std::string> file_paths;
    for (int i = 1; i < 5 && i < argc; ++i) {
        file_paths.push_back(argv[i]);
    }
    
    // 自動識別文件類型
    std::string imu_path, baro_path, gps_path, mag_path;
    
    std::cout << "讀取文件:" << std::endl;
    
    for (const auto& path : file_paths) {
        SensorType type = identifySensorType(path);
        
        switch (type) {
            case IMU:
                imu_path = path;
                std::cout << "IMU: " << path << std::endl;
                break;
            case BARO:
                baro_path = path;
                std::cout << "氣壓計: " << path << std::endl;
                break;
            case GPS:
                gps_path = path;
                std::cout << "GPS: " << path << std::endl;
                break;
            case MAG:
                mag_path = path;
                std::cout << "磁力計: " << path << std::endl;
                break;
            default:
                std::cerr << "未知文件類型: " << path << std::endl;
                break;
        }
    }
    
    // 載入數據
    if (!imu_path.empty()) {
        imu_data = loadImuData(imu_path);
    }
    
    if (!baro_path.empty()) {
        baro_data = loadBaroData(baro_path);
    }
    
    if (!gps_path.empty()) {
        gps_data = loadGpsData(gps_path);
    }
    
    if (!mag_path.empty()) {
        mag_data = loadMagData(mag_path);
    }
    
    // 檢查是否成功載入所有數據
    if (imu_data.empty()) {
        std::cerr << "警告：未載入IMU數據，EKF無法正常運行！" << std::endl;
        return false;
    }
    
    // 檢查時間戳範圍
    uint64_t min_time = std::numeric_limits<uint64_t>::max();
    uint64_t max_time = 0;
    
    if (!imu_data.empty()) {
        min_time = std::min(min_time, imu_data.front().timestamp_us);
        max_time = std::max(max_time, imu_data.back().timestamp_us);
    }
    
    if (!gps_data.empty()) {
        min_time = std::min(min_time, gps_data.front().timestamp_us);
        max_time = std::max(max_time, gps_data.back().timestamp_us);
    }
    
    if (!mag_data.empty()) {
        min_time = std::min(min_time, mag_data.front().timestamp_us);
        max_time = std::max(max_time, mag_data.back().timestamp_us);
    }
    
    if (!baro_data.empty()) {
        min_time = std::min(min_time, baro_data.front().timestamp_us);
        max_time = std::max(max_time, baro_data.back().timestamp_us);
    }
    
    std::cout << "數據時間戳檢查：" << std::endl;
    std::cout << "最小時間戳：" << min_time << " 微秒" << std::endl;
    std::cout << "最大時間戳：" << max_time << " 微秒" << std::endl;
    std::cout << "數據時長：" << (max_time - min_time) / 1e6 << " 秒" << std::endl;
    
    return true;
}


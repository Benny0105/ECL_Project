#pragma once

#include "sensor_data_loader.h"
#include <memory>
#include <vector>
#include "ecl/EKF/ekf.h"

// EKF運行器類
class EkfRunner {
public:
    EkfRunner();
    ~EkfRunner();
    
    // 初始化EKF
    bool initialize(uint64_t timestamp);
    
    // 處理傳感器數據
    bool runEkf(const std::vector<ImuSample>& imu_data,
               const std::vector<BaroSample>& baro_data,
               const std::vector<GpsSample>& gps_data,
               const std::vector<MagSample>& mag_data);
    
private:
    // 處理IMU數據
    bool processImuData(const ImuSample& imu);
    
    // 處理磁力計數據
    bool processMagData(const MagSample& mag);
    
    // 處理GPS數據
    bool processGpsData(const GpsSample& gps);
    
    // 處理氣壓計數據
    bool processBaroData(const BaroSample& baro);
    
    // 按時間順序排序和處理所有數據
    void processSensorDataInOrder();
    
    // EKF實例
    std::unique_ptr<Ekf> ekf_;
    
    // 當前處理時間
    uint64_t current_time_us_;
    
    // 用於處理的數據
    struct SensorDataPoint {
        enum Type { IMU, BARO, GPS, MAG } type;
        uint64_t timestamp_us;
        size_t index;
    };
    
    std::vector<SensorDataPoint> merged_data_;
    
    // 原始數據指針
    const std::vector<ImuSample>* imu_data_;
    const std::vector<BaroSample>* baro_data_;
    const std::vector<GpsSample>* gps_data_;
    const std::vector<MagSample>* mag_data_;
    
    // 處理狀態
    bool is_initialized_;
    bool imu_updated_;
    int warmup_count_;
    bool warmup_complete_;
    bool position_valid_;
    bool attitude_valid_;
    
    // 輸出EKF狀態
    void outputEkfState(uint64_t timestamp);
};
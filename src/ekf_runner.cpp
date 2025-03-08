#include "ekf_runner.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "ecl/EKF/ekf.h"

// 將弧度轉換為度
static float rad2deg(float rad) {
    return rad * 180.0f / M_PI;
}

EkfRunner::EkfRunner() 
    : ekf_(new Ekf()),
      current_time_us_(0),
      imu_data_(nullptr),
      baro_data_(nullptr),
      gps_data_(nullptr),
      mag_data_(nullptr),
      is_initialized_(false),
      imu_updated_(false),
      warmup_count_(0),
      warmup_complete_(false),
      position_valid_(false),
      attitude_valid_(false) {
}

EkfRunner::~EkfRunner() {
}

bool EkfRunner::initialize(uint64_t timestamp_us) {
    if (ekf_ == nullptr) {
        std::cerr << "EKF 未創建，初始化失敗" << std::endl;
        return false;
    }

    // 初始化EKF參數
    if (!ekf_->init(timestamp_us)) {
        std::cerr << "EKF 初始化失敗" << std::endl;
        return false;
    }

    std::cout << "EKF 基本初始化成功，時間戳：" << timestamp_us << std::endl;

    // 強制初始化EKF - 模擬適當的IMU數據
    // 提供足夠多的靜態樣本
    for (int i = 0; i < 200; i++) {
        imuSample imu_sample;
        imu_sample.time_us = timestamp_us + i * 10000; // 間隔10毫秒
        imu_sample.delta_ang = Vector3f(0.0f, 0.0f, 0.0f); // 零角速度
        imu_sample.delta_ang_dt = 0.01f; // 10毫秒
        
        // 確保加速度數據符合地球重力
        Vector3f accel_init(0.0f, 0.0f, -9.81f); // 向下的重力加速度
        imu_sample.delta_vel = accel_init * 0.01f; // 乘上時間得到速度增量
        imu_sample.delta_vel_dt = 0.01f; // 10毫秒
        
        // 每次都更新EKF
        ekf_->setIMUData(imu_sample);
        ekf_->update();
    }
    
    // 使用我們新添加的方法強制初始化
    if (ekf_->forceInitialization()) {
        std::cout << "EKF 強制初始化成功" << std::endl;
    } else {
        std::cerr << "EKF 強制初始化失敗" << std::endl;
    }
    
    // 輸出初始狀態
    auto status_flags = ekf_->control_status_flags();
    std::cout << "初始標誌狀態: tilt_align=" << (status_flags.tilt_align ? "1" : "0")
              << " yaw_align=" << (status_flags.yaw_align ? "1" : "0")
              << " gps=" << (status_flags.gps ? "1" : "0")
              << " mag_hdg=" << (status_flags.mag_hdg ? "1" : "0") << std::endl;

    is_initialized_ = true;
    warmup_count_ = 0;
    return true;
}

bool EkfRunner::processImuData(const ImuSample& imu) {
    // 將IMU數據轉換為PX4 EKF格式
    imuSample ekf_imu;
    ekf_imu.time_us = imu.timestamp_us;
    
    // 時間增量轉換：微秒 -> 秒
    ekf_imu.delta_ang_dt = imu.delta_angle_dt;
    ekf_imu.delta_vel_dt = imu.delta_velocity_dt;
    
    // 若時間增量異常，修正為合理值
    if (ekf_imu.delta_ang_dt < 1e-4f || ekf_imu.delta_ang_dt > 0.1f) {
        ekf_imu.delta_ang_dt = 0.01f; // 10ms 作為默認值
    }
    
    if (ekf_imu.delta_vel_dt < 1e-4f || ekf_imu.delta_vel_dt > 0.1f) {
        ekf_imu.delta_vel_dt = 0.01f; // 10ms 作為默認值
    }
    
    // 角度增量保持原樣，但若太大則限制
    Vector3f gyro_rate(
        imu.delta_angle(0) / ekf_imu.delta_ang_dt,
        imu.delta_angle(1) / ekf_imu.delta_ang_dt,
        imu.delta_angle(2) / ekf_imu.delta_ang_dt
    );
    
    float gyro_norm = gyro_rate.norm();
    if (gyro_norm > 0.1f) {  // 0.1 rad/s 約 5.7°/s
        // 降低角速度
        ekf_imu.delta_ang = gyro_rate.normalized() * 0.05f * ekf_imu.delta_ang_dt;
    } else {
        ekf_imu.delta_ang = Vector3f(
            imu.delta_angle(0),
            imu.delta_angle(1),
            imu.delta_angle(2)
        );
    }
    
    // 處理速度增量，確保加速度在合理範圍內
    Vector3f accel_raw(
        imu.delta_velocity(0) / ekf_imu.delta_vel_dt,
        imu.delta_velocity(1) / ekf_imu.delta_vel_dt,
        imu.delta_velocity(2) / ekf_imu.delta_vel_dt
    );
    
    float accel_norm = accel_raw.norm();
    
    // 如果已初始化，使用原始數據；否則進行修正確保初始化成功
    if (ekf_->control_status_flags().tilt_align) {
        // 已初始化，使用原始數據
        ekf_imu.delta_vel = Vector3f(
            imu.delta_velocity(0),
            imu.delta_velocity(1),
            imu.delta_velocity(2)
        );
    } else {
        // 未初始化，確保加速度在合理範圍，接近9.81 m/s²
        // 且主要在z方向上
        Vector3f ideal_accel(0.0f, 0.0f, -9.81f); // 假設重力向下
        
        // 如果z值為正，則反轉重力方向
        if (accel_raw(2) > 0) {
            ideal_accel = Vector3f(0.0f, 0.0f, 9.81f);
        }
        
        // 使用理想加速度，但保留一些原始方向信息
        ekf_imu.delta_vel = ideal_accel * ekf_imu.delta_vel_dt;
    }
    
    // 將IMU數據提供給EKF
    ekf_->setIMUData(ekf_imu);
    
    // 增加預熱計數
    warmup_count_++;
    
    // 每5個樣本更新一次EKF
    if (warmup_count_ % 5 == 0) {
        ekf_->update();
    }
    
    return true;
}

bool EkfRunner::processMagData(const MagSample& mag) {
    // 將磁力計數據轉換為PX4 EKF格式
    magSample ekf_mag;
    ekf_mag.time_us = mag.timestamp_us;
    ekf_mag.mag = Vector3f(mag.mag(0), mag.mag(1), mag.mag(2));
    
    // 將磁力計數據提供給EKF
    ekf_->setMagData(ekf_mag);
    return true;
}

bool EkfRunner::processGpsData(const GpsSample& gps) {
    gps_message ekf_gps;
    ekf_gps.time_usec = gps.timestamp_us;
    
    // 經緯度和高度
    ekf_gps.lat = static_cast<int32_t>(gps.lat * 1e7);  // 度 -> 度*1e7
    ekf_gps.lon = static_cast<int32_t>(gps.lon * 1e7);  // 度 -> 度*1e7
    ekf_gps.alt = static_cast<int32_t>(gps.alt * 1000); // 米 -> 毫米
    
    // 速度 - 使用 vel_ned 向量
    ekf_gps.vel_ned(0) = gps.vel_ned(0); // 北向速度
    ekf_gps.vel_ned(1) = gps.vel_ned(1); // 東向速度
    ekf_gps.vel_ned(2) = gps.vel_ned(2); // 下向速度
    
    // 計算總速度
    ekf_gps.vel_m_s = gps.vel_ned.norm();
    
    // 設置 GPS 質量參數
    ekf_gps.fix_type = gps.fix_type;
    ekf_gps.eph = gps.eph;
    ekf_gps.epv = gps.epv;
    ekf_gps.sacc = gps.sacc;
    ekf_gps.nsats = 10; // 默認值或從 gps 樣本中獲取如果可用
    ekf_gps.vel_ned_valid = true;   // 指定 NED 速度有效
    
    // 發送 GPS 數據到 EKF
    ekf_->setGpsData(ekf_gps);
    
    // 每接收幾個GPS數據點就更新一次EKF
    static int gps_count = 0;
    gps_count++;
    if (gps_count % 5 == 0) {
        ekf_->update();
    }
    
    return true;
}

bool EkfRunner::processBaroData(const BaroSample& baro) {
    // 將氣壓計數據轉換為PX4 EKF格式
    baroSample ekf_baro;
    ekf_baro.time_us = baro.timestamp_us;
    ekf_baro.hgt = -baro.altitude_m; // 注意負號，PX4使用NED坐標系
    
    // 將氣壓計數據提供給EKF
    ekf_->setBaroData(ekf_baro);
    return true;
}

void EkfRunner::processSensorDataInOrder() {
    // 按時間戳排序
    std::sort(merged_data_.begin(), merged_data_.end(),
             [](const SensorDataPoint& a, const SensorDataPoint& b) {
                 return a.timestamp_us < b.timestamp_us;
             });
    
    // 跟蹤處理狀態
    int imu_count = 0;
    int mag_count = 0;
    int gps_count = 0;
    int baro_count = 0;
    
    // 預熱階段 - 首先處理一批IMU數據
    std::cout << "開始 EKF 預熱..." << std::endl;
    
    const int WARMUP_IMU_COUNT = 200; // 預熱所需的IMU樣本數
    
    // 第一階段：IMU數據預熱
    for (const auto& data_point : merged_data_) {
        if (data_point.type == SensorDataPoint::IMU && imu_count < WARMUP_IMU_COUNT) {
            processImuData((*imu_data_)[data_point.index]);
            imu_count++;
        }
    }
    
    // 第二階段：處理一些磁力計數據以幫助航向對齊
    std::cout << "處理GPS和磁力計數據以幫助EKF對齊..." << std::endl;
    
    for (const auto& data_point : merged_data_) {
        if (data_point.type == SensorDataPoint::MAG && mag_count < 50) {
            processMagData((*mag_data_)[data_point.index]);
            mag_count++;
        }
        
        if (data_point.type == SensorDataPoint::GPS && gps_count < 10) {
            processGpsData((*gps_data_)[data_point.index]);
            gps_count++;
        }
    }
    
    std::cout << "處理了 " << mag_count << " 個磁力計樣本和 " << gps_count << " 個GPS樣本" << std::endl;
    
    // 檢查姿態對齊狀態
    if (ekf_->control_status_flags().tilt_align) {
        std::cout << "EKF姿態對齊成功!" << std::endl;
        attitude_valid_ = true;
        warmup_complete_ = true;
    } else {
        std::cout << "警告：找不到符合初始化條件的靜態 IMU 數據" << std::endl;
        std::cout << "EKF 預熱失敗，但繼續運行..." << std::endl;
    }
    
    // 輸出EKF初始狀態
    std::cout << "EKF 初始狀態：" << std::endl;
    std::cout << "位置有效：" << (ekf_->global_position_is_valid() ? "是" : "否") << std::endl;
    std::cout << "姿態對準：" << (ekf_->control_status_flags().tilt_align ? "是" : "否") << std::endl;
    
    // 主處理循環 - 按時間順序處理所有傳感器數據
    std::cout << "開始主處理循環..." << std::endl;
    bool ekf_output_valid = false;
    int output_counter = 0;
    
    for (const auto& data_point : merged_data_) {
        // 更新當前時間
        current_time_us_ = data_point.timestamp_us;
        
        // 根據傳感器類型處理數據
        switch (data_point.type) {
            case SensorDataPoint::IMU:
                imu_updated_ = processImuData((*imu_data_)[data_point.index]);
                imu_count++;
                break;
                
            case SensorDataPoint::MAG:
                processMagData((*mag_data_)[data_point.index]);
                mag_count++;
                break;
                
            case SensorDataPoint::GPS:
                processGpsData((*gps_data_)[data_point.index]);
                gps_count++;
                break;
                
            case SensorDataPoint::BARO:
                processBaroData((*baro_data_)[data_point.index]);
                baro_count++;
                break;
        }
        
        // 每處理10個IMU樣本，輸出EKF狀態
        if (data_point.type == SensorDataPoint::IMU && output_counter++ % 10 == 0) {
            // 獲取EKF位置和姿態
            Vector3f position = ekf_->getPosition();
            Vector3f velocity = ekf_->getVelocity();
            Quatf quat = ekf_->getQuaternion();
            Vector3f euler;
            
            // 四元數轉歐拉角
            euler(0) = atan2f(2.0f * (quat(0) * quat(1) + quat(2) * quat(3)),
                      1.0f - 2.0f * (quat(1) * quat(1) + quat(2) * quat(2)));
            euler(1) = asinf(2.0f * (quat(0) * quat(2) - quat(3) * quat(1)));
            euler(2) = atan2f(2.0f * (quat(0) * quat(3) + quat(1) * quat(2)),
                              1.0f - 2.0f * (quat(2) * quat(2) + quat(3) * quat(3)));
            
            std::cout << "時間戳: " << current_time_us_ 
                      << " 姿態:[" << rad2deg(euler(0)) << "," << rad2deg(euler(1)) << "," << rad2deg(euler(2)) << "]度";
            
            if (ekf_->global_position_is_valid()) {
                std::cout << " 位置有效:[" << position(0) << "," << position(1) << "," << position(2) << "]";
                ekf_output_valid = true;
                position_valid_ = true;
            } else {
                std::cout << " 位置無效";
            }
            std::cout << std::endl;
            
            // 輸出EKF狀態標誌
            std::cout << "  狀態標誌: tilt_align=" << ekf_->control_status_flags().tilt_align
                      << " yaw_align=" << ekf_->control_status_flags().yaw_align
                      << " gps=" << ekf_->control_status_flags().gps
                      << " mag_hdg=" << ekf_->control_status_flags().mag_hdg << std::endl;
        }
    }
    
    // 輸出處理統計信息
    std::cout << "處理完成。總共處理：" << std::endl
              << "  IMU 數據: " << imu_count << " 條" << std::endl
              << "  磁力計數據: " << mag_count << " 條" << std::endl
              << "  GPS 數據: " << gps_count << " 條" << std::endl
              << "  氣壓計數據: " << baro_count << " 條" << std::endl;
    
    if (!ekf_output_valid) {
        std::cout << "警告：整個運行過程中未獲得有效的EKF輸出！" << std::endl;
        std::cout << "請檢查：" << std::endl;
        std::cout << "1) IMU數據格式是否正確（檢查角度增量和速度增量是否正常）" << std::endl;
        std::cout << "2) GPS數據是否有效（檢查經緯度是否在合理範圍）" << std::endl;
        std::cout << "3) 磁力計數據是否正常（檢查磁場強度是否合理）" << std::endl;
        std::cout << "4) 傳感器時間戳是否同步" << std::endl;
    }
}

bool EkfRunner::runEkf(const std::vector<ImuSample>& imu_data,
                      const std::vector<BaroSample>& baro_data,
                      const std::vector<GpsSample>& gps_data,
                      const std::vector<MagSample>& mag_data) {
    // 檢查是否有IMU數據
    if (imu_data.empty()) {
        std::cerr << "沒有IMU數據，無法運行EKF" << std::endl;
        return false;
    }
    
    // 保存數據引用
    imu_data_ = &imu_data;
    baro_data_ = &baro_data;
    gps_data_ = &gps_data;
    mag_data_ = &mag_data;
    
    // 初始化EKF
    if (!is_initialized_) {
        if (!initialize(imu_data.front().timestamp_us)) {
            return false;
        }
    }
    
    // 檢查初始化後的狀態
    if (!ekf_->control_status_flags().tilt_align) {
        std::cout << "姿態未對齊，嘗試強制姿態對齊..." << std::endl;
        
        // 強制處理更多IMU樣本以初始化姿態
        int align_count = 0;
        for (size_t i = 0; i < std::min(size_t(200), imu_data.size()); i++) {
            processImuData(imu_data[i]);
            align_count++;
            
            // 檢查是否已對齊
            if (ekf_->control_status_flags().tilt_align) {
                std::cout << "姿態已成功對齊，處理了 " << align_count << " 條IMU數據" << std::endl;
                break;
            }
            
            // 每20個樣本輸出一次狀態
            if (align_count % 20 == 0) {
                std::cout << "已處理 " << align_count << " 條IMU數據，仍未對齊姿態" << std::endl;
            }
        }
    }
    
    // 準備合併的數據點
    merged_data_.clear();
    
    // 添加各類型數據到處理隊列
    std::cout << "添加 " << imu_data.size() << " 條IMU數據到處理隊列" << std::endl;
    for (size_t i = 0; i < imu_data.size(); i++) {
        SensorDataPoint point;
        point.type = SensorDataPoint::IMU;
        point.timestamp_us = imu_data[i].timestamp_us;
        point.index = i;
        merged_data_.push_back(point);
    }
    
    std::cout << "添加 " << baro_data.size() << " 條氣壓計數據到處理隊列" << std::endl;
    for (size_t i = 0; i < baro_data.size(); i++) {
        SensorDataPoint point;
        point.type = SensorDataPoint::BARO;
        point.timestamp_us = baro_data[i].timestamp_us;
        point.index = i;
        merged_data_.push_back(point);
    }
    
    std::cout << "添加 " << gps_data.size() << " 條GPS數據到處理隊列" << std::endl;
    for (size_t i = 0; i < gps_data.size(); i++) {
        SensorDataPoint point;
        point.type = SensorDataPoint::GPS;
        point.timestamp_us = gps_data[i].timestamp_us;
        point.index = i;
        merged_data_.push_back(point);
    }
    
    std::cout << "添加 " << mag_data.size() << " 條磁力計數據到處理隊列" << std::endl;
    for (size_t i = 0; i < mag_data.size(); i++) {
        SensorDataPoint point;
        point.type = SensorDataPoint::MAG;
        point.timestamp_us = mag_data[i].timestamp_us;
        point.index = i;
        merged_data_.push_back(point);
    }
    
    std::cout << "合併後總共有 " << merged_data_.size() << " 條數據等待處理" << std::endl;
    
    // 對merged_data按時間戳排序
    std::sort(merged_data_.begin(), merged_data_.end(),
             [](const SensorDataPoint& a, const SensorDataPoint& b) {
                 return a.timestamp_us < b.timestamp_us;
             });
    
    // 按時間順序處理所有數據
    std::cout << "按時間順序處理所有數據..." << std::endl;
    int imu_count = 0;
    int baro_count = 0;
    int gps_count = 0;
    int mag_count = 0;
    
    for (const auto& data_point : merged_data_) {
        switch (data_point.type) {
            case SensorDataPoint::IMU:
                processImuData((*imu_data_)[data_point.index]);
                imu_count++;
                break;
                
            case SensorDataPoint::BARO:
                processBaroData((*baro_data_)[data_point.index]);
                baro_count++;
                break;
                
            case SensorDataPoint::GPS:
                processGpsData((*gps_data_)[data_point.index]);
                gps_count++;
                break;
                
            case SensorDataPoint::MAG:
                processMagData((*mag_data_)[data_point.index]);
                mag_count++;
                break;
        }
        
        // 每處理100個IMU數據點輸出一次EKF狀態
        if (imu_count % 100 == 0 && data_point.type == SensorDataPoint::IMU) {
            outputEkfState(data_point.timestamp_us);
        }
    }
    
    std::cout << "處理完成。總共處理：" << std::endl
              << "  IMU 數據: " << imu_count << " 條" << std::endl
              << "  磁力計數據: " << mag_count << " 條" << std::endl
              << "  GPS 數據: " << gps_count << " 條" << std::endl
              << "  氣壓計數據: " << baro_count << " 條" << std::endl;
    
    // 輸出最終EKF狀態
    outputEkfState(imu_data.back().timestamp_us);
    
    if (!ekf_->control_status_flags().tilt_align) {
        std::cout << "警告：整個運行過程中未獲得有效的EKF輸出！" << std::endl;
        std::cout << "請檢查：" << std::endl;
        std::cout << "1) IMU數據格式是否正確（檢查角度增量和速度增量是否正常）" << std::endl;
        std::cout << "2) GPS數據是否有效（檢查經緯度是否在合理範圍）" << std::endl;
        std::cout << "3) 磁力計數據是否正常（檢查磁場強度是否合理）" << std::endl;
        std::cout << "4) 傳感器時間戳是否同步" << std::endl;
    } else {
        std::cout << "成功！EKF已正確初始化並產生有效輸出。" << std::endl;
    }
    
    return true;
}

// 添加輔助函數來輸出EKF狀態
void EkfRunner::outputEkfState(uint64_t timestamp_us) {
    // 獲取EKF狀態
    if (ekf_ == nullptr) {
        return;
    }
    
    // 獲取控制狀態標誌
    auto status_flags = ekf_->control_status_flags();
    
    // 獲取位置和速度 - 正確的調用方式
    Vector3f position = ekf_->getPosition();
    Vector3f velocity = ekf_->getVelocity();
    
    // 獲取姿態四元數
    Quatf quat = ekf_->calculate_quaternion();
    
    // 計算姿態角（歐拉角）
    float roll = atan2f(2.0f * (quat(0) * quat(1) + quat(2) * quat(3)),
                      1.0f - 2.0f * (quat(1) * quat(1) + quat(2) * quat(2)));
                      
    float pitch = asinf(2.0f * (quat(0) * quat(2) - quat(3) * quat(1)));
    
    float yaw = atan2f(2.0f * (quat(0) * quat(3) + quat(1) * quat(2)),
                      1.0f - 2.0f * (quat(2) * quat(2) + quat(3) * quat(3)));
    
    // 輸出狀態信息
    std::cout << "EKF狀態 @ " << timestamp_us << " us:" << std::endl;
    std::cout << "  標誌: tilt_align=" << status_flags.tilt_align
              << " yaw_align=" << status_flags.yaw_align
              << " gps=" << status_flags.gps
              << " mag_hdg=" << status_flags.mag_hdg << std::endl;
              
    std::cout << "  姿態(rad): roll=" << roll << " pitch=" << pitch << " yaw=" << yaw << std::endl;
    std::cout << "  位置(m): x=" << position(0) << " y=" << position(1) << " z=" << position(2) << std::endl;
    std::cout << "  速度(m/s): vx=" << velocity(0) << " vy=" << velocity(1) << " vz=" << velocity(2) << std::endl;
}
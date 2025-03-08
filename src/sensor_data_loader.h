#ifndef SENSOR_DATA_LOADER_H
#define SENSOR_DATA_LOADER_H

#include <string>
#include <vector>
#include <cstdint>
#include <Eigen/Dense>

// IMU數據結構，對應PX4格式
struct ImuSample {
    uint64_t timestamp_us;
    float delta_angle_dt;
    float delta_velocity_dt;
    Eigen::Vector3f delta_angle;
    Eigen::Vector3f delta_velocity;
};

// 氣壓計數據結構，對應PX4格式
struct BaroSample {
    uint64_t timestamp_us;
    float pressure_pa;
    float altitude_m;
};

// GPS數據結構，對應PX4格式
struct GpsSample {
    uint64_t timestamp_us;
    double lat;  // 緯度（度）
    double lon;  // 經度（度）
    float alt;   // 高度（米）
    Eigen::Vector3f vel_ned; // 北東下速度（米/秒）
    uint8_t fix_type;
    float eph;   // 水平位置不確定性（米）
    float epv;   // 垂直位置不確定性（米）
    float sacc;  // 速度不確定性（米/秒）
    uint8_t nsats; // 衛星數量
};

// 磁力計數據結構，對應PX4格式
struct MagSample {
    uint64_t timestamp_us;
    Eigen::Vector3f mag; // 磁場（高斯）
};

// 數據載入類
class SensorDataLoader {
public:
    // 通過文件標題行自動識別傳感器類型
    enum SensorType {
        IMU,
        BARO,
        GPS,
        MAG,
        UNKNOWN
    };
    
    // 自動識別並載入CSV文件
    static SensorType identifySensorType(const std::string& csv_path);
    
    // 載入各類型傳感器數據
    static std::vector<ImuSample> loadImuData(const std::string& csv_path);
    static std::vector<BaroSample> loadBaroData(const std::string& csv_path);
    static std::vector<GpsSample> loadGpsData(const std::string& csv_path);
    static std::vector<MagSample> loadMagData(const std::string& csv_path);
    
    // 從命令行參數加載所有數據
    static bool loadAllSensorData(int argc, char** argv,
                                  std::vector<ImuSample>& imu_data,
                                  std::vector<BaroSample>& baro_data,
                                  std::vector<GpsSample>& gps_data,
                                  std::vector<MagSample>& mag_data);
};

#endif // SENSOR_DATA_LOADER_H
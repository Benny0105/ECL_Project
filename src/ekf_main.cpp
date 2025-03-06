#include <iostream>
#include "sensor_data_loader.h"
#include "EKF/ekf.h"

int main(int argc, char* argv[])
{
    // 檢查是否有足夠的輸入參數
    if (argc < 4) {
        std::cerr << "Usage: ./EKF_Project <vehicle_imu.csv> <vehicle_air_data.csv> <vehicle_gps_position.csv>\n";
        return -1;
    }

    // 先宣告變數
    std::string imu_csv     = argv[1];
    std::string airdata_csv = argv[2];
    std::string gps_csv     = argv[3];

    std::cout << "Reading files:\n"
              << "IMU: " << imu_csv << "\n"
              << "Baro: " << airdata_csv << "\n"
              << "GPS: " << gps_csv << "\n";

    // 1. 讀取並合併資料
    std::vector<SensorData> sensor_data_vec = mergeThreeCsv(imu_csv, airdata_csv, gps_csv);
    if (sensor_data_vec.empty()) {
        std::cerr << "No merged sensor data.\n";
        return -1;
    }

    // 2. 建立 EKF
    Ekf ekf;
    ekf.init(0.01f); // 假設 0.01s = 100Hz

    float prev_time_s = 0.0f;

    for (size_t i = 0; i < sensor_data_vec.size(); i++) {
        SensorData &sd = sensor_data_vec[i];

        float dt = sd.timestamp_s - prev_time_s;
        prev_time_s = sd.timestamp_s;

        // (A) 餵 IMU
        estimator::imuSample imu_sample{};
        imu_sample.time_us = sd.timestamp_s * 1e6;
        imu_sample.delta_ang(0) = sd.gyro_x;
        imu_sample.delta_ang(1) = sd.gyro_y;
        imu_sample.delta_ang(2) = sd.gyro_z;
        imu_sample.delta_vel(0) = sd.accel_x;
        imu_sample.delta_vel(1) = sd.accel_y;
        imu_sample.delta_vel(2) = sd.accel_z;
        imu_sample.delta_vel_dt = dt;
        imu_sample.delta_ang_dt = dt;

        ekf.setIMUData(imu_sample);

        // (B) 餵 Baro
        estimator::baroSample baro_sample{};
        baro_sample.time_us = sd.timestamp_s * 1e6;
        baro_sample.hgt = sd.baro_alt;

        ekf.setBaroData(baro_sample);

        // (C) 餵 GPS
        estimator::gps_message gps_msg{};
        gps_msg.time_usec = sd.timestamp_s * 1e6;
        gps_msg.lat = static_cast<int32_t>(sd.lat * 1e7);
        gps_msg.lon = static_cast<int32_t>(sd.lon * 1e7);
        gps_msg.alt = static_cast<int32_t>(sd.gps_alt * 1000);
	gps_msg.vel_ned(0) = sd.velN;  // N
	gps_msg.vel_ned(1) = sd.velE;  // E
	gps_msg.vel_ned(2) = sd.velD;  // D
	
        ekf.setGpsData(gps_msg);
	
	// **關鍵: 這裡加入 EKF 的融合控制**
    	ekf.controlFusionModes();

	
        // (D) update
        ekf.update();

        // (E) 取得姿態、位置
        std::cout << "Time: " << sd.timestamp_s
                  << " | PosNED: " << ekf.getPosition()(0) << ", "
                  << ekf.getPosition()(1) << ", "
                  << ekf.getPosition()(2)
                  << std::endl;
    }

    std::cout << "EKF simulation finished!\n";
    return 0;
}


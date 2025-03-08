#include "sensor_data_loader.h"
#include "ekf_runner.h"
#include <iostream>
#include <vector>

// 提前聲明函數
int runNewProcessor(int argc, char** argv);
int originalMain(int argc, char** argv);

// 原始主程式入口點保持不變
int main(int argc, char** argv) {
    // 檢查命令行參數，是否使用新處理方式
    bool use_new_processing = (argc > 1 && std::string(argv[1]) == "--new");
    
    if (use_new_processing) {
        // 使用新的處理方式
        return runNewProcessor(argc-1, argv+1);
    } else {
        // 調用原始的處理函數
        return originalMain(argc, argv);
    }
}

// 新的處理函數
int runNewProcessor(int argc, char** argv) {
    // 載入所有傳感器數據
    std::vector<ImuSample> imu_data;
    std::vector<BaroSample> baro_data;
    std::vector<GpsSample> gps_data;
    std::vector<MagSample> mag_data;
    
    // 使用自動識別功能載入數據
    if (!SensorDataLoader::loadAllSensorData(argc, argv, imu_data, baro_data, gps_data, mag_data)) {
        std::cerr << "載入傳感器數據失敗！" << std::endl;
        return 1;
    }
    
    // 創建 EKF 運行器
    EkfRunner ekf_runner;
    
    // 運行 EKF
    if (!ekf_runner.runEkf(imu_data, baro_data, gps_data, mag_data)) {
        std::cerr << "EKF 運行失敗！" << std::endl;
        return 1;
    }
    
    std::cout << "處理完成" << std::endl;
    return 0;
}

// 為相容性提供的函數，臨時將使用與新處理函數相同的代碼
// 以後可以改為調用舊的 ekf_main.cpp
int originalMain(int argc, char** argv) {
    // 暫時和新處理方式使用相同代碼
    return runNewProcessor(argc, argv);
} 
#include <iostream>

// 假設 ECL 裡有一個 "EKF/ekf.h" (或 "ekf.h"), 依你實際情況修改
// 有些版本可能在 ECL_Project/ECL/ekf.h 或 ECL_Project/ecl/EKF/ekf.h
#include "EKF/ekf.h"

int main() {
    std::cout << "Hello from ECL_Project main!\n";

    // 建立一個 Ekf 物件 (如果成功編譯 & 連結就表示載入 ECL 成功)
    Ekf my_ekf;  
    std::cout << "Successfully created EKF object.\n";

    // (可選) 呼叫一些簡單的方法
    // my_ekf.init(0.01f);

    return 0;
}


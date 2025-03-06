cmake_minimum_required(VERSION 3.10)
project(ECL_Project)

set(CMAKE_CXX_STANDARD 17)

#
# 1. 指定 EKF_MAIN_SOURCES 為你的 cpp 檔案
#    假設它們在 ~/ECL_Project/src/ 目錄
#
set(EKF_MAIN_SOURCES
    # ${CMAKE_SOURCE_DIR}/src/hello.cpp

    ${CMAKE_SOURCE_DIR}/src/ekf_main.cpp
    ${CMAKE_SOURCE_DIR}/src/sensor_data_loader.cpp

    # **確保這些 ECL 內部檔案 (control.cpp、ekf_helper.cpp) 也被編譯**
    ${CMAKE_SOURCE_DIR}/ecl/EKF/control.cpp
    ${CMAKE_SOURCE_DIR}/ecl/EKF/ekf_helper.cpp
)

# 建立可執行檔 ECL_Project
add_executable(ECL_Project ${EKF_MAIN_SOURCES})
# add_definitions(-DDISABLE_ECL_MATH_ALIAS)

# 2. 設置 include 路徑
#    - ${CMAKE_SOURCE_DIR}/include 放你自定義的 mathlib.h 或其他標頭
#    - ecl/, ecl/EKF/, ecl/matrix/ 等放 ECL 原始碼
#    - src/ 如果你要 include "sensor_data_loader.h"
#
target_include_directories(ECL_Project PRIVATE
    ${CMAKE_SOURCE_DIR}/include      # <--- 關鍵: -I ~/ECL_Project/include
    ${CMAKE_SOURCE_DIR}/ecl         # ECL headers
    ${CMAKE_SOURCE_DIR}/ecl/EKF     # if you include "EKF/ekf.h"
    ${CMAKE_SOURCE_DIR}/ecl/matrix  # if you include <matrix/math.hpp>
    ${CMAKE_SOURCE_DIR}/src         # if you want "sensor_data_loader.h"
)

#
# 3. 連結 ECL 靜態庫 (.a 檔)
#    (在 ~/ECL_Project/ecl/build/ 生成的 libecl_*.a)
#
target_link_libraries(ECL_Project PRIVATE
    ${CMAKE_SOURCE_DIR}/ecl/build/EKF/libecl_EKF.a
    ${CMAKE_SOURCE_DIR}/ecl/build/airdata/libecl_airdata.a
    ${CMAKE_SOURCE_DIR}/ecl/build/geo/libecl_geo.a
    ${CMAKE_SOURCE_DIR}/ecl/build/geo_lookup/libecl_geo_lookup.a
)

#
# 4. 如果程式需要 Eigen，則添加
#
find_package(Eigen3 REQUIRED)
target_include_directories(ECL_Project PRIVATE Eigen3::Eigen)


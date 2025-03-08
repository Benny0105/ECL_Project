cmake_minimum_required(VERSION 3.10)
project(ECL_Project)

# 設置C++標準
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含ECL庫目錄
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/ecl)
include_directories(${CMAKE_SOURCE_DIR}/src)

# 查找Eigen庫
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 選擇使用哪個main文件
# 如果你想使用新的架構，設為TRUE，否則設為FALSE
set(USE_NEW_IMPLEMENTATION TRUE)

if(USE_NEW_IMPLEMENTATION)
    # 新實現 - 使用重構的代碼
    set(SOURCES
        src/main.cpp             # 新的main入口點，會根據參數選擇新舊實現
        src/sensor_data_loader.cpp
        src/ekf_runner.cpp
    )
else()
    # 原始實現 - 使用舊的ekf_main.cpp
    set(SOURCES
        src/ekf_main.cpp
        src/sensor_data_loader.cpp  # 假設這是原始的loader
    )
endif()

# 設置ECL源文件
file(GLOB_RECURSE ECL_SRCS 
    ${CMAKE_SOURCE_DIR}/ecl/EKF/*.cpp
    ${CMAKE_SOURCE_DIR}/ecl/airdata/*.cpp
    ${CMAKE_SOURCE_DIR}/ecl/geo/*.cpp
    ${CMAKE_SOURCE_DIR}/ecl/geo_lookup/*.cpp
    ${CMAKE_SOURCE_DIR}/ecl/mathlib/*.cpp
)

# 排除 Python 生成的文件
file(GLOB_RECURSE PYTHON_GENERATED_FILES 
    "${CMAKE_SOURCE_DIR}/ecl/EKF/python/ekf_derivation/generated/*.cpp"
)

list(REMOVE_ITEM ECL_SRCS ${PYTHON_GENERATED_FILES})

# 創建可執行文件（只定義一次）
add_executable(ECL_Project ${SOURCES} ${ECL_SRCS})

# 設置include目錄
target_include_directories(ECL_Project PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/ecl
    ${CMAKE_SOURCE_DIR}/ecl/EKF
    ${CMAKE_SOURCE_DIR}/ecl/matrix
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

# 鏈接庫
target_link_libraries(ECL_Project PRIVATE
    ${CMAKE_SOURCE_DIR}/ecl/build/EKF/libecl_EKF.a
    ${CMAKE_SOURCE_DIR}/ecl/build/airdata/libecl_airdata.a
    ${CMAKE_SOURCE_DIR}/ecl/build/geo/libecl_geo.a
    ${CMAKE_SOURCE_DIR}/ecl/build/geo_lookup/libecl_geo_lookup.a
    m
)

# 設置編譯選項
target_compile_options(ECL_Project PRIVATE -Wall -Wextra -DDISABLE_ECL_MATH_ALIAS -include "${CMAKE_SOURCE_DIR}/src/ecl_math_defines.h")
target_compile_definitions(ECL_Project PRIVATE MATHLIB_CONSTRAIN_DEFINED)

#
# 1. 指定 EKF_MAIN_SOURCES 為你的 cpp 檔案
#    假設它們在 ~/ECL_Project/src/ 目錄
#
set(EKF_MAIN_SOURCES
    # ${CMAKE_SOURCE_DIR}/src/hello.cpp

    ${CMAKE_SOURCE_DIR}/src/ekf_main.cpp
    ${CMAKE_SOURCE_DIR}/src/sensor_data_loader.cpp

)

# 建立可執行檔 ECL_Project
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
    m
)

#
# 4. 如果程式需要 Eigen，則添加
#
find_package(Eigen3 REQUIRED)
target_include_directories(ECL_Project PRIVATE Eigen3::Eigen)


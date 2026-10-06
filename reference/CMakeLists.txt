cmake_minimum_required(VERSION 3.16)

project(RRTstarDI_Benchmark LANGUAGES CXX)

# ============================================================
# C++ 标准
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# 编译优化
# ============================================================
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -mavx2 -mfma")

# ============================================================
# 查找依赖
# ============================================================

# OMPL
find_package(ompl REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
    serialization
)

# ============================================================
# 可执行文件
# ============================================================
add_executable(test
    MyRRTstar.cpp
    DICostUtils.cpp
    test.cpp
)

# ============================================================
# 头文件
# ============================================================
target_include_directories(test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/include/ompl-1.5
    /usr/include/eigen3
    /usr/include/opencv4
)

# ============================================================
# 链接库
# ============================================================
target_link_directories(test PRIVATE
    /usr/local/lib
)

target_link_libraries(test PRIVATE
    ompl
    Boost::system
    Boost::filesystem
    Boost::serialization
    ${OpenCV_LIBS}
    pthread
)

# ============================================================
# RPATH
# ============================================================
set_target_properties(test PROPERTIES
    BUILD_RPATH "/usr/local/lib"
)

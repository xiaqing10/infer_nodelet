cmake_minimum_required(VERSION 3.0.2)
project(cambricon_bench CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -O3 -DUSE_CAMBRICON=1")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Project headers
include_directories(${CMAKE_SOURCE_DIR}/../../include)
include_directories(${CMAKE_SOURCE_DIR}/../../include/cambricon)
include_directories(${CMAKE_SOURCE_DIR}/../../infer_mlu_lib/3rd/neuware/include)
include_directories(${CMAKE_SOURCE_DIR}/../../infer_mlu_lib/cn/inference/include)
include_directories(${CMAKE_SOURCE_DIR}/../../infer_mlu_lib/cn/tools/include)

# Link directories
link_directories(${CMAKE_SOURCE_DIR}/../../infer_mlu_lib/3rd/neuware/lib64)
link_directories(${CMAKE_SOURCE_DIR}/../../infer_mlu_lib/3rd/opencv/lib)

set(CN_TOOLS_DIR ${CMAKE_SOURCE_DIR}/../../infer_mlu_lib/cn/tools/src)
set(CN_INFERENCE_DIR ${CMAKE_SOURCE_DIR}/../../infer_mlu_lib/cn/inference/src)

add_executable(cambricon_bench
    cambricon_bench.cpp
    cambricon_detector.cpp
    ${CN_TOOLS_DIR}/check.cpp
    ${CN_TOOLS_DIR}/Timer.cpp
    ${CN_INFERENCE_DIR}/Net.cc
)
target_link_libraries(cambricon_bench
    ${OpenCV_LIBS}
    cndrv cnrt
    pthread
)
cmake_minimum_required(VERSION 3.0.2)
project(rknn_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Ofast -O3")

# RKNN SDK
include_directories(.)
link_directories(.)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(rknn_test main.cpp rknn_test.cpp postprocess.cc preprocess.cc)
target_link_libraries(rknn_test ${OpenCV_LIBS} rknn_api pthread)

add_executable(rknn_benchmark benchmark.cpp rknn_test.cpp postprocess.cc preprocess.cc)
target_link_libraries(rknn_benchmark ${OpenCV_LIBS} rknn_api pthread)

add_executable(multi_instance_bench multi_instance_bench.cpp)
target_link_libraries(multi_instance_bench ${OpenCV_LIBS} rknn_api pthread)

add_executable(rknn_4way_bench rknn_4way_bench.cpp)
target_link_libraries(rknn_4way_bench ${OpenCV_LIBS} rknn_api pthread)
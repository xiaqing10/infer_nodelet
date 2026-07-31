cmake_minimum_required(VERSION 3.0.2)
project(rknn_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Ofast -O3")

# RKNN SDK
include_directories(/usr/include /tmp/tests/rknn)
link_directories(/usr/lib /tmp/tests/rknn)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(rknn_test main.cpp rknn_test.cpp)
target_link_libraries(rknn_test ${OpenCV_LIBS} rknn_api pthread)
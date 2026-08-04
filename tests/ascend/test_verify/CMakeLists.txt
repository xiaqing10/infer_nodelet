cmake_minimum_required(VERSION 3.5.1)
project(test_verify CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_DVPP_INTERFACE")

set(ASCEND_NNRT /usr/local/Ascend/nnrt/8.3.RC1)
set(ACLLITE_DIR ${PROJECT_SOURCE_DIR}/../infer_nodelet/infer_mlu_lib/3rd/acllite)

include_directories(
    ${ASCEND_NNRT}/runtime/include/
    /usr/include/opencv4/
    ${ACLLITE_DIR}
)

link_directories(
    ${ASCEND_NNRT}/runtime/lib64/
    ${ACLLITE_DIR}
)

add_executable(main main.cpp)

target_link_libraries(main
    ascendcl
    acl_dvpp
    acllite
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    stdc++
    dl
    rt
    pthread
)
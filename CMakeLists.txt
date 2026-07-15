cmake_minimum_required(VERSION 3.0.2)
project(infer_nodelet CXX C)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Ofast -g -O3 -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
set(ROS_PACKGE_RELOCATE         "/home/files/rvf")
set(ROS_PACKGE_SOURCE_DIR       "${CMAKE_SOURCE_DIR}")

# Platform selection
option(USE_CAMBRICON "Build with Cambricon MLU backend" OFF)
option(USE_SOPHON "Build with Sophon TPU backend" ON)
option(USE_ASCEND "Build with Ascend NPU backend" OFF)
option(USE_NVIDIA "Build with NVIDIA TensorRT backend" OFF)

set(backend_count 0)
if(USE_CAMBRICON)
    math(EXPR backend_count "${backend_count}+1")
endif()
if(USE_SOPHON)
    math(EXPR backend_count "${backend_count}+1")
endif()
if(USE_ASCEND)
    math(EXPR backend_count "${backend_count}+1")
endif()
if(USE_NVIDIA)
    math(EXPR backend_count "${backend_count}+1")
endif()
if(backend_count EQUAL 0)
    message(FATAL_ERROR "One of USE_CAMBRICON, USE_SOPHON, USE_ASCEND, USE_NVIDIA must be ON")
endif()
if(backend_count GREATER 1)
    message(FATAL_ERROR "Only one backend can be selected")
endif()

if(USE_CAMBRICON)
    add_definitions(-DUSE_CAMBRICON=1)
    message(STATUS "Building for Cambricon MLU platform")
elseif(USE_SOPHON)
    add_definitions(-DUSE_SOPHON=1)
    message(STATUS "Building for Sophon TPU platform")
elseif(USE_ASCEND)
    add_definitions(-DUSE_ASCEND=1)
    add_definitions(-DENABLE_DVPP_INTERFACE)
    message(STATUS "Building for Ascend NPU platform")
elseif(USE_NVIDIA)
    add_definitions(-DUSE_NVIDIA=1)
    message(STATUS "Building for NVIDIA TensorRT platform")
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)

if(USE_CAMBRICON)
    include_directories(${PROJECT_SOURCE_DIR}/include/cambricon)
    include_directories(${PROJECT_SOURCE_DIR}//infer_mlu_lib/3rd/neuware/include)
    link_directories(${PROJECT_SOURCE_DIR}/infer_mlu_lib/3rd/neuware/lib64)
    include_directories(${PROJECT_SOURCE_DIR}//infer_mlu_lib/3rd/opencv/include)
    include_directories(/usr/include/aarch64-linux-gnu)
    link_directories(${PROJECT_SOURCE_DIR}//infer_mlu_lib/3rd/opencv/lib)
    include_directories(${PROJECT_SOURCE_DIR}/infer_mlu_lib/cn/inference/include)

    set(CN_INFERENCE_DIR ${PROJECT_SOURCE_DIR}//infer_mlu_lib/cn/inference/src)
    include_directories(${PROJECT_SOURCE_DIR}/infer_mlu_lib/cn/tools/include)
    set(CN_TOOLS_DIR ${PROJECT_SOURCE_DIR}/infer_mlu_lib/cn/tools/src)
    include_directories(/usr/include/eigen3)

    list(APPEND cambricon cndrv)
    list(APPEND cambricon cnrt)
    list(APPEND opencv opencv_core)
    list(APPEND opencv opencv_highgui)
    list(APPEND opencv opencv_imgproc)
    list(APPEND opencv opencv_videoio)
    list(APPEND opencv opencv_imgcodecs)
endif()

if(USE_SOPHON)
    include_directories(${PROJECT_SOURCE_DIR}/include/sophon)
    include_directories(/opt/sophon/libsophon-0.5.1/include/)
    include_directories(${PROJECT_SOURCE_DIR}/include/opencv4)
    include_directories(/usr/include/eigen3)
    link_directories(/opt/sophon/sophon-opencv-latest/lib/)
    link_directories(/opt/sophon/libsophon-current/lib/)
    set(OPENCV_LIBS opencv_imgproc opencv_core opencv_highgui opencv_imgcodecs)
    set(BM_LIBS bmlib bmrt bmcv yuv)
    set(JPU_LIBS bmjpuapi bmjpulite)
endif()

if(USE_ASCEND)
    include_directories(${PROJECT_SOURCE_DIR}/include/ascend)
    include_directories(${PROJECT_SOURCE_DIR}/include/acllite)
    include_directories(/usr/local/Ascend/nnrt/8.3.RC1/runtime/include/)
    include_directories(/usr/include/opencv4/)
    include_directories(/usr/include/eigen3)
    link_directories(/usr/local/Ascend/nnrt/8.3.RC1/runtime/lib64/)
    link_directories(${PROJECT_SOURCE_DIR}/infer_mlu_lib/3rd/acllite)
    set(OPENCV_LIBS opencv_imgproc opencv_core opencv_highgui opencv_imgcodecs)
    set(ASCEND_LIBS ascendcl acl_dvpp acllite)
endif()

if(USE_NVIDIA)
    include_directories(${PROJECT_SOURCE_DIR}/include/nvidia)
    include_directories(/usr/local/cuda/include)
    include_directories(/usr/include/opencv4/)
    include_directories(/usr/include/eigen3)
    link_directories(/usr/local/cuda/targets/aarch64-linux/lib/)
    set(OPENCV_LIBS opencv_imgproc opencv_core opencv_highgui opencv_imgcodecs opencv_dnn)
    set(NVIDIA_LIBS nvinfer cudart nvinfer_plugin)
endif()

#cv_brige
include_directories(/opt/ros/noetic/include)
link_directories(/opt/ros/noetic/lib)
include_directories(/opt/ros/noetic/include/xmlrpcpp/)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  message_generation
  image_transport
  sensor_msgs
)
add_message_files(
  DIRECTORY /home/files/common_msgs/rvf_commons
  FILES
  RadarTrackObjectProject.msg
  RadarTrackObjectProjectSingle.msg
  ImageDetectObject.msg
  ImageDetectObjectSingle.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs  nodelet pluginlib
  LIBRARIES infer_nodelet
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
)

file(GLOB native_srcs src/*.cpp)
file(GLOB cambricon_srcs src/cambricon/*.cpp)
file(GLOB sophon_srcs src/sophon/*.cpp)
file(GLOB ascend_srcs src/ascend/*.cpp)
file(GLOB nvidia_srcs src/nvidia/*.cpp)

if(USE_CAMBRICON)
    add_library(infer_nodelet src/infer.cpp ${CN_TOOLS_DIR}/check.cpp ${CN_TOOLS_DIR}/Timer.cpp ${CN_INFERENCE_DIR}/Net.cc ${native_srcs} ${cambricon_srcs})
    target_link_libraries(infer_nodelet ${catkin_LIBRARIES})
    target_link_libraries(infer_nodelet cv_bridge)
    target_link_libraries(infer_nodelet ${cambricon} ${opencv} pthread avformat avcodec avutil swscale jsoncpp)
endif()

if(USE_SOPHON)
    add_library(infer_nodelet ${native_srcs} ${sophon_srcs})
    add_dependencies(infer_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(infer_nodelet ${catkin_LIBRARIES})
    target_link_libraries(infer_nodelet cv_bridge)
    target_link_libraries(infer_nodelet ${OPENCV_LIBS} ${BM_LIBS} ${JPU_LIBS} -lpthread -lavcodec -lavformat -lavutil)
endif()

if(USE_ASCEND)
    add_library(infer_nodelet ${native_srcs} ${ascend_srcs})
    add_dependencies(infer_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(infer_nodelet ${catkin_LIBRARIES})
    target_link_libraries(infer_nodelet cv_bridge)
    target_link_libraries(infer_nodelet ${OPENCV_LIBS} ${ASCEND_LIBS} -lpthread -lavcodec -lavformat -lavutil)
endif()

if(USE_NVIDIA)
    add_library(infer_nodelet ${native_srcs} ${nvidia_srcs})
    add_dependencies(infer_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(infer_nodelet ${catkin_LIBRARIES})
    target_link_libraries(infer_nodelet cv_bridge)
    target_link_libraries(infer_nodelet ${OPENCV_LIBS} ${NVIDIA_LIBS} -lpthread -lavcodec -lavformat -lavutil)
endif()


install(DIRECTORY launch model
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(USE_CAMBRICON)
install(FILES config/infer_cambricon.yaml config/bytetrack.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
elseif(USE_SOPHON)
install(FILES config/infer_sophon.yaml config/bytetrack.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
elseif(USE_ASCEND)
install(FILES config/infer_ascend.yaml config/bytetrack.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
elseif(USE_NVIDIA)
install(FILES config/infer_nvidia.yaml config/bytetrack.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
endif()

install(TARGETS  infer_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet_plugins.xml package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(USE_CAMBRICON)
install(FILES infer_mlu_lib/3rd/neuware/lib64/libcnrt.so infer_mlu_lib/3rd/neuware/lib64/libcndrv.so
  infer_mlu_lib/3rd/opencv/lib/libopencv_core.so.3.4 infer_mlu_lib/3rd/opencv/lib/libopencv_imgcodecs.so.3.4
  infer_mlu_lib/3rd/opencv/lib/libopencv_imgproc.so.3.4
  infer_mlu_lib/3rd/opencv/lib/libopencv_highgui.so.3.4 infer_mlu_lib/3rd/opencv/lib/libopencv_videoio.so.3.4
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
endif()
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/package_debian.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/package_debian.cmake
        @ONLY
    )

    include(${CMAKE_CURRENT_BINARY_DIR}/cmake/package_debian.cmake)

    if(USE_CAMBRICON)
        set(USE_CAMBRICON_CMAKE TRUE)
    else()
        set(USE_CAMBRICON_CMAKE FALSE)
    endif()
    if(USE_SOPHON)
        set(USE_SOPHON_CMAKE TRUE)
    else()
        set(USE_SOPHON_CMAKE FALSE)
    endif()
    if(USE_ASCEND)
        set(USE_ASCEND_CMAKE TRUE)
    else()
        set(USE_ASCEND_CMAKE FALSE)
    endif()
    if(USE_NVIDIA)
        set(USE_NVIDIA_CMAKE TRUE)
    else()
        set(USE_NVIDIA_CMAKE FALSE)
    endif()
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/install_hook.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/install_hook.cmake
        @ONLY
    )

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/control.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/control
        @ONLY
    )

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/postinst.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/postinst
        @ONLY
    )

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/preinst.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/preinst
        @ONLY
    )

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/postrm.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/postrm
        @ONLY
    )

    install(SCRIPT
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/install_hook.cmake
    )

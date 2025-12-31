cmake_minimum_required(VERSION 3.10)
project(infer_nodelet CXX C)
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Ofast -g -O3 -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

set(ROS_PACKGE_RELOCATE         "home/files/rvf")
set(ROS_PACKGE_SOURCE_DIR       "${CMAKE_SOURCE_DIR}")

#include_directories(${CMAKE_SOURCE_DIR}/)
#include_directories(${PROJECT_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  image_transport
  sensor_msgs
  #cv_bridg
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

include_directories(
  #include                    
  ${catkin_INCLUDE_DIRS}     
  ${CATKIN_DEVEL_PREFIX}/include 
)
message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message(STATUS "CATKIN_DEVEL_PREFIX: ${CATKIN_DEVEL_PREFIX}")

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs  nodelet pluginlib
  LIBRARIES infer_nodelet
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
)

if(NOT PROJECT_SOURCE_DIR)
    set(PROJECT_SOURCE_DIR "${CMAKE_SOURCE_DIR}/src/infer_nodelet")
endif()

include_directories(${PROJECT_SOURCE_DIR}/include /opt/sophon/libsophon-0.5.1/include/ ${PROJECT_SOURCE_DIR}/include/opencv4)
include_directories(/opt/ros/noetic/include)
include_directories(/usr/include/eigen3)

link_directories(/opt/ros/noetic/lib)
link_directories(/opt/sophon/sophon-opencv-latest/lib/)
link_directories(/opt/sophon/libsophon-current/lib/)
set(OPENCV_LIBS  opencv_imgproc opencv_core opencv_highgui opencv_imgcodecs )
set(BM_LIBS bmlib bmrt bmcv yuv)

set(JPU_LIBS bmjpuapi bmjpulite)

aux_source_directory(src native_srcs)
add_library(infer_nodelet   ${native_srcs})
add_dependencies(infer_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(infer_nodelet  ${catkin_LIBRARIES})
target_link_libraries(infer_nodelet  cv_bridge)
target_link_libraries(infer_nodelet  ${OPENCV_LIBS} ${BM_LIBS}  ${JPU_LIBS} -lpthread -lavcodec -lavformat -lavutil)


install(DIRECTORY launch config model
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS  infer_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet_plugins.xml package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/package_debian.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/package_debian.cmake
        @ONLY
    )

include(${CMAKE_CURRENT_BINARY_DIR}/cmake/package_debian.cmake)

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

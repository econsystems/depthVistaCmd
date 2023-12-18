cmake_minimum_required(VERSION 3.5)

project(DepthVistaIMU LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(DepthVistaIMU
DepthVistaIMU.cpp
DepthVistaIMU.h
)

target_link_libraries(DepthVistaIMU PRIVATE pthread DepthVistaSDK /usr/lib/libopencv_world.so.4.2.0 )
target_include_directories(DepthVistaIMU PRIVATE ${DEPTHVISTA_INCLUDE_DIRS} /usr/local/include/DepthVistaSDK)

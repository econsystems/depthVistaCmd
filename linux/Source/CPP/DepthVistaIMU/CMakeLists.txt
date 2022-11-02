cmake_minimum_required(VERSION 3.5)

project(DepthVistaIMU LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(DepthVistaIMU
DepthVistaIMU.cpp
DepthVistaIMU.h
)

target_link_libraries(DepthVistaIMU PRIVATE pthread ${OpenCV_LIBRARIES} DepthVistaSDK)
target_include_directories(DepthVistaIMU PRIVATE ${OpenCV_INCLUDE_DIRS} ${DEPTHVISTA_INCLUDE_DIRS})

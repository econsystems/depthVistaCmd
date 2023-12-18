cmake_minimum_required(VERSION 3.5)

project(DepthVistaConsole LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(DepthVistaConsole
DepthVistaConsoleApp.cpp
)

target_link_libraries(DepthVistaConsole PRIVATE pthread DepthVistaSDK /usr/lib/libopencv_world.so.4.2.0 )
target_include_directories(DepthVistaConsole PRIVATE ${DEPTHVISTA_INCLUDE_DIRS} /usr/local/include/DepthVistaSDK)

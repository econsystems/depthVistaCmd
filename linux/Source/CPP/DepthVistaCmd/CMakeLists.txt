cmake_minimum_required(VERSION 3.5)

project(DepthVistaConsole LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(DepthVistaConsole
DepthVistaConsoleApp.cpp
DepthVistaConsoleApp.h
)

target_link_libraries(DepthVistaConsole PRIVATE pthread DepthVistaSDK ${OpenCV_LIBRARIES} )
target_include_directories(DepthVistaConsole PRIVATE /usr/local/include/DepthVistaSDK ${OpenCV_INCLUDE_DIRS})

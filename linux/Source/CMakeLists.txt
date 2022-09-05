cmake_minimum_required(VERSION 3.5)

project(DepthVistaConsole LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(DepthVistaConsole
DepthVistaConsoleApp.cpp
)

target_link_libraries(DepthVistaConsole PRIVATE pthread ${OpenCV_LIBRARIES} DepthVistaSDK)
target_include_directories(DepthVistaConsole PRIVATE ${OpenCV_INCLUDE_DIRS} ${DEPTHVISTA_INCLUDE_DIRS})

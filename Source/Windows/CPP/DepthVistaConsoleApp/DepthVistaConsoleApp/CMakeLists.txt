cmake_minimum_required(VERSION 3.5)

project(DepthVistaConsole LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use the below to debug build with ASAN
# if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -g")
#     set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
# endif()

find_package(OpenCV REQUIRED)

add_executable(DepthVistaConsole
DepthVistaConsoleApp.cpp
DepthVistaConsoleApp.h
)
target_include_directories(DepthVistaConsole PRIVATE /usr/local/include/DepthVistaSDK ${OpenCV_INCLUDE_DIRS})
if( UNIX )
	target_link_libraries(DepthVistaConsole PRIVATE pthread DepthVistaSDK ${OpenCV_LIBRARIES} )
elseif( WIN32  )
	target_link_libraries(DepthVistaConsole PRIVATE DepthVistaSDK ${OpenCV_LIBRARIES} )
endif()

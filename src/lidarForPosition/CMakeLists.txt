# cmake version
cmake_minimum_required(VERSION 3.10.2)

# Project Name
set(PROJ_NAME URG_Lidar)
project(${PROJ_NAME})

# Check if c++11 is available
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Binary output path
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

# 设置源文件变量，生成可执行文件
aux_source_directory(${CMAKE_SOURCE_DIR}/src/lidarForPosition/src SOURCE_LIST)
add_executable(${PROJ_NAME} ${SOURCE_LIST})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJ_NAME})

# URG
include_directories(${CMAKE_SOURCE_DIR}/src/lidarForPosition/urg_library-1.2.5/include/c)
include_directories(${CMAKE_SOURCE_DIR}/src/lidarForPosition/urg_library-1.2.5/include/cpp)
target_link_libraries(${PROJ_NAME} ${CMAKE_SOURCE_DIR}/src/lidarForPosition/urg_library-1.2.5/src/liburg_cpp.so)
target_link_libraries(${PROJ_NAME} ${CMAKE_SOURCE_DIR}/src/lidarForPosition/urg_library-1.2.5/src/liburg_c.so)
#　OpenCV
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJ_NAME} ${OpenCV_LIBS})
endif()

FIND_PACKAGE(Threads)
TARGET_LINK_LIBRARIES(${PROJ_NAME} ${CMAKE_THREAD_LIBS_INIT})
TARGET_LINK_LIBRARIES(${PROJ_NAME} process_communication)


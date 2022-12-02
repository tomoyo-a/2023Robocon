cmake_minimum_required(VERSION 3.0)

set(CMAKE_BUILD_TYPE "Debug")

# Project Name
set(PROJ_NAME control)
project(${PROJ_NAME})
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  #设置c++的编译选项
#set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")  #设置c的编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -lpthread -fopenmp")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/src/motion_control/src/*.h" "${CMAKE_SOURCE_DIR}/src/motion_control/Action_User/*.h" "${CMAKE_SOURCE_DIR}/src/motion_control/MotionCard/*.h")

FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/src/motion_control/src/*.c" "${CMAKE_SOURCE_DIR}/src/motion_control/Action_User/*.c" "${CMAKE_SOURCE_DIR}/src/motion_control/MotionCard/*.c" )

FILE(GLOB SOURCE_LIST1 "${CMAKE_SOURCE_DIR}/src/motion_control/src/*.cpp" "${CMAKE_SOURCE_DIR}/src/motion_control/Action_User/*.cpp" "${CMAKE_SOURCE_DIR}/src/motion_control/MotionCard/*.cpp")

add_executable(${PROJ_NAME} ${SOURCE_LIST} ${HEADER_LIST} ${SOURCE_LIST1})

include_directories(${PROCESS_COMM_DIR}
                    ${CMAKE_SOURCE_DIR}/src/motion_control/Action_User
		    ${CMAKE_SOURCE_DIR}/src/motion_control/MotionCard
    		    ${CMAKE_SOURCE_DIR}/src/motion_control/src)


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
FIND_PACKAGE(Threads)
TARGET_LINK_LIBRARIES(${PROJ_NAME} ${CMAKE_THREAD_LIBS_INIT})
TARGET_LINK_LIBRARIES(${PROJ_NAME} process_communication)
target_link_libraries(${PROJ_NAME} ${CMAKE_SOURCE_DIR}/src/motion_control/lib/libGinkgo_Driver.so)

target_link_libraries(${PROJ_NAME} ${CMAKE_SOURCE_DIR}/src/motion_control/lib/libusb.so)






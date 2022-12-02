# cmake version
cmake_minimum_required(VERSION 3.0)

project(process_communication)

add_library(process_communication STATIC process_comm.c)
target_include_directories(process_communication PUBLIC .)
target_link_libraries(process_communication )




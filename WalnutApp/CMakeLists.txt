cmake_minimum_required(VERSION 3.16)

project(walnut_app)
set(CMAKE_CXX_STANDARD 17)

FILE(GLOB_RECURSE HDR_FILES include/*.h*)
FILE(GLOB_RECURSE SRC_FILES src/*.c*)

include_directories(walnutapp ${HDR_FILES})
add_executable(walnutapp ${SRC_FILES})

target_link_libraries(walnutapp walnut libzedtracker)

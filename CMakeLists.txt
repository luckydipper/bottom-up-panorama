cmake_minimum_required(VERSION 3.1)
project(Panorama)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#                    "/usr/include/eigen3/"
add_executable(show_img.o src/main.cpp)
target_link_libraries(show_img.o ${OpenCV_LIBS})

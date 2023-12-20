cmake_minimum_required(VERSION 3.1)
project(Panorama)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#                    "/usr/include/eigen3/"


#SET(GCC_COVERAGE_COMPILE_FLAGS "-g")
#SET(CMAKE_CXX_FLAGS "{}")
add_compile_options("-g")

add_executable(show_img.o src/main.cpp src/matcher.cpp src/util.cpp src/stitcher.cpp src/optimizer.cpp)
target_link_libraries(show_img.o ${OpenCV_LIBS})

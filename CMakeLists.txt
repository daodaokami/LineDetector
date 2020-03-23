project(LineDetector)

cmake_minimum_required(VERSION 2.8)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( CMAKE_BUILD_TYPE "RELEASE" )

include_directories(${PROJECT_SOURCE_DIR}/detect)

find_package(OpenCV REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/detect
    ${OpenCV_INCLUDE_DIRS}
)

add_library(detector SHARED
    ${PROJECT_SOURCE_DIR}/detect/LineDetector.cpp
)

add_executable(linedetector_test test_linedetector.cpp)
target_link_libraries(linedetector_test
    detector ${OpenCV_LIBS}
    )
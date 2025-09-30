cmake_minimum_required(VERSION 3.10)
project(lidar_calibration LANGUAGES CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Define executable and sources
add_executable(lidar_cal
    lidarCal.cpp
    segmentation.cpp
    generateTransform.cpp
    ICP.cpp
    segmentation.h
    generateTransform.h
    ICP.h
)

# Include directories
target_include_directories(lidar_cal PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)

# Link libraries
target_link_libraries(lidar_cal
    ${PCL_LIBRARIES}
)

# Compiler warnings (optional but recommended)
if (MSVC)
    target_compile_options(lidar_cal PRIVATE /W4)
else()
    target_compile_options(lidar_cal PRIVATE -Wall -Wextra -pedantic)
endif()

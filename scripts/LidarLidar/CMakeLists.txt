cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
project(lidar_cal)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (lidar_cal lidarCal.cpp segmentation.cpp generateTransform.cpp ICP.cpp)
target_link_libraries (lidar_cal ${PCL_LIBRARIES})


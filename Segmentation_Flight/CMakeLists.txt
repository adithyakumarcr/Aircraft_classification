	

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(flight_segmentation)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (flight_segmentation flight_segmentation.cpp)
target_link_libraries (flight_segmentation ${PCL_LIBRARIES})


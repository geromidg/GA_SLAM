###############
### General ###
###############
cmake_minimum_required(VERSION 3.5.1)  # Ubuntu 16.04 (Xenial)

project(ga_slam)

set(TARGET_NAME ${PROJECT_NAME})
set(CMAKE_INCLUDE_DIR ${CMAKE_SOURCE_DIR})
set(CMAKE_SOURCE_DIR ${CMAKE_SOURCE_DIR}/${TARGET_NAME}/)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

enable_testing()

####################
### Dependencies ###
####################
find_package(grid_map_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(GTest REQUIRED)

#############
### Build ###
#############
include_directories(
    ${CMAKE_INCLUDE_DIR}
    ${grid_map_core_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED
    ${CMAKE_SOURCE_DIR}/GaSlam.cpp
    ${CMAKE_SOURCE_DIR}/localization/ParticleFilter.cpp
    ${CMAKE_SOURCE_DIR}/localization/PoseEstimation.cpp
    ${CMAKE_SOURCE_DIR}/localization/PoseCorrection.cpp
    ${CMAKE_SOURCE_DIR}/mapping/Map.cpp
    ${CMAKE_SOURCE_DIR}/mapping/DataRegistration.cpp
    ${CMAKE_SOURCE_DIR}/mapping/DataFusion.cpp
    ${CMAKE_SOURCE_DIR}/processing/CloudProcessing.cpp
    ${CMAKE_SOURCE_DIR}/processing/ImageProcessing.cpp
)

target_link_libraries(${TARGET_NAME}
    ${grid_map_core_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

##################
### Pkg Config ###
##################
configure_file(
    ${TARGET_NAME}.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY
)

###############
### Install ###
###############
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc
    DESTINATION lib/pkgconfig
)

install(
    TARGETS ${TARGET_NAME}
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY ${CMAKE_INCLUDE_DIR}/${TARGET_NAME}
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

###############
### Testing ###
###############


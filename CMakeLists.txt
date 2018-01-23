# This file is part of GA SLAM.
# Copyright (C) 2018 Dimitris Geromichalos,
# Planetary Robotics Lab (PRL), European Space Agency (ESA)
#
# GA SLAM is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GA SLAM is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GA SLAM. If not, see <http://www.gnu.org/licenses/>.

###############
### General ###
###############
cmake_minimum_required(VERSION 3.5.1)  # Ubuntu 16.04 (Xenial)

project(ga_slam)

set(TARGET_NAME ${PROJECT_NAME})
set(CMAKE_INCLUDE_DIR ${PROJECT_SOURCE_DIR})
set(CMAKE_SOURCE_DIR ${PROJECT_SOURCE_DIR}/${TARGET_NAME})

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
    ${CMAKE_SOURCE_DIR}/GaSlam.cc
    ${CMAKE_SOURCE_DIR}/localization/ParticleFilter.cc
    ${CMAKE_SOURCE_DIR}/localization/PoseEstimation.cc
    ${CMAKE_SOURCE_DIR}/localization/PoseCorrection.cc
    ${CMAKE_SOURCE_DIR}/mapping/Map.cc
    ${CMAKE_SOURCE_DIR}/mapping/DataRegistration.cc
    ${CMAKE_SOURCE_DIR}/mapping/DataFusion.cc
    ${CMAKE_SOURCE_DIR}/processing/CloudProcessing.cc
    ${CMAKE_SOURCE_DIR}/processing/ImageProcessing.cc
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
    ${PROJECT_SOURCE_DIR}/cmake/${TARGET_NAME}.pc.in
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
    FILES_MATCHING PATTERN "*.h"
)

###############
### Testing ###
###############
add_executable(GaSlamTest ${CMAKE_SOURCE_DIR}/GaSlamTest.cc)
target_link_libraries(GaSlamTest ${TARGET_NAME} ${GTEST_BOTH_LIBRARIES})
add_test(GaSlamTest GaSlamTest)

add_executable(ParticleFilterTest
    ${CMAKE_SOURCE_DIR}/localization/ParticleFilterTest.cc)
target_link_libraries(ParticleFilterTest ${TARGET_NAME} ${GTEST_BOTH_LIBRARIES})
add_test(ParticleFilterTest ParticleFilterTest)


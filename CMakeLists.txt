cmake_minimum_required(VERSION 2.6)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Rock)
rock_init(ga_slam 0.1)
rock_standard_layout()

rock_library(ga_slam
    SOURCES
        src/GaSlam.cpp
        src/localization/ParticleFilter.cpp
        src/localization/PoseEstimation.cpp
        src/localization/PoseCorrection.cpp
        src/mapping/Map.cpp
        src/mapping/DataRegistration.cpp
        src/mapping/DataFusion.cpp
        src/processing/CloudProcessing.cpp
        src/processing/ImageProcessing.cpp
    HEADERS
        src/GaSlam.hpp
        src/TypeDefs.hpp
        src/localization/ParticleFilter.hpp
        src/localization/PoseEstimation.hpp
        src/localization/PoseCorrection.hpp
        src/mapping/Map.hpp
        src/mapping/DataRegistration.hpp
        src/mapping/DataFusion.hpp
        src/processing/CloudProcessing.hpp
        src/processing/ImageProcessing.hpp
    DEPS_PKGCONFIG grid_map_core)

target_link_libraries(ga_slam
    ${PCL_LIBRARIES})


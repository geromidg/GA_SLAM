rock_library(ga_slam
    SOURCES
        GaSlam.cpp
        PoseEstimation.cpp
        PoseCorrection.cpp
        DataRegistration.cpp
        DataFusion.cpp
    HEADERS
        GaSlam.hpp
        PoseEstimation.hpp
        PoseCorrection.hpp
        DataRegistration.hpp
        DataFusion.hpp
    DEPS_PKGCONFIG grid_map_core)

target_link_libraries(ga_slam
    ${PCL_LIBRARIES})


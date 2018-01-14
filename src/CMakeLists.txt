rock_library(ga_slam
    SOURCES
        GaSlam.cpp
        localization/ParticleFilter.cpp
        localization/PoseEstimation.cpp
        localization/PoseCorrection.cpp
        mapping/Map.cpp
        mapping/DataRegistration.cpp
        mapping/DataFusion.cpp
        processing/CloudProcessing.cpp
        processing/ImageProcessing.cpp
    HEADERS
        GaSlam.hpp
        TypeDefs.hpp
        localization/ParticleFilter.hpp
        localization/PoseEstimation.hpp
        localization/PoseCorrection.hpp
        mapping/Map.hpp
        mapping/DataRegistration.hpp
        mapping/DataFusion.hpp
        processing/CloudProcessing.hpp
        processing/ImageProcessing.hpp
    DEPS_PKGCONFIG grid_map_core)

target_link_libraries(ga_slam
    ${PCL_LIBRARIES})


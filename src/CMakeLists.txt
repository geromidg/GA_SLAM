rock_library(ga_slam
    SOURCES GaSlam.cpp
    HEADERS GaSlam.hpp
    DEPS_PKGCONFIG grid_map_core)

target_link_libraries(ga_slam
    ${PCL_LIBRARIES})


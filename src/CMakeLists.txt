rock_library(ga_slam
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(ga_slam_bin Main.cpp
    DEPS ga_slam)


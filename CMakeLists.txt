cmake_minimum_required(VERSION 2.6)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Rock)
rock_init(ga_slam 0.1)
rock_standard_layout()


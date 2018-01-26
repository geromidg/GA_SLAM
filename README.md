## GA SLAM: Globally Adaptive Simultaneous Localization And Mapping

[![Travis](https://img.shields.io/travis/geromidg/GA_SLAM.svg?style=plastic)](https://travis-ci.org/geromidg/GA_SLAM)
[![Coveralls](https://img.shields.io/coveralls/github/geromidg/GA_SLAM.svg?style=plastic)](https://coveralls.io/r/geromidg/GA_SLAM)
[![License](https://img.shields.io/github/license/geromidg/GA_SLAM.svg?style=plastic)](https://github.com/geromidg/GA_SLAM/blob/master/LICENSE)

---------

This is an open-source and research-oriented C++ library for solving the SLAM problem on planetary rovers that<br>
aim to navigate and explore extreme terrains. It adopts a globally adaptive technique which uses an a priori global<br>
low-resolution map provided by orbital imagery to enhance the robot's perception and correct its global position.

Created by: Dimitris Geromichalos (geromidg@gmail.com)<br>
Supervised by: Martin Azkarate (martin.azkarate@esa.int)<br>
Developed at: Planetary Robotics Lab (PRL), European Space Agency (ESA)

### Features

* :sunrise_over_mountains: Suitable for **rough terrain** navigation by utilizing 2.5D (elevation) maps
* :globe_with_meridians: Global pose correction using **map matching** of local (robot-centric) and global (orbiter) elevation maps
* :camera: **Sensor-agnostic** data registration using point clouds (support for lidar, stereo camera, ToF camera etc.)
* :collision: Automatic **sensor fusion** when multiple inputs are provided (without prior configuration)
* :page_with_curl: **Easily tunable** parameter set to fit the needs of different robots and applications

### Installation

Although the library can be used on other systems, it was validated using:
* Ubuntu 16.04 (Xenial)
* CMake 3.5.1
* GCC 5.4.0

#### Dependencies

Required packages:
* [Eigen 3](http://eigen.tuxfamily.org) (linear algebra calculations)
* [PCL 1.7](http://pointclouds.org/) (point cloud processing)
* [OpenCV 2.4](https://opencv.org/) (image processing)
* [Grid Map Core](https://github.com/ethz-asl/grid_map) (map structure and calculations)

Optional packages:
* [Google Mock](https://github.com/google/googletest/tree/master/googlemock) (testing)

The dependecies listed above can be installed using [this script](https://github.com/geromidg/GA_SLAM/blob/master/scripts/install_dependencies.sh) as well.

#### Building

To build the library, run:

    git clone https://github.com/geromidg/GA_SLAM.git
    cd ga_slam
    mkdir build && cd build
    cmake ..
    make

To build and execute the tests, run:

    cmake -DENABLE_TESTS=ON ..
    make
    make test

Make sure to set CMAKE_PREFIX_PATH if you installed the grid_map_core library from the debian package.<br>
For example if you installed it using `sudo apt-get install ros-kinetic-grid-map-core`, run:

    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/ros/kinetic


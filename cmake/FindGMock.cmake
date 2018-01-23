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

list(APPEND GMOCK_CHECK_SOURCES
    src/gmock.cc
)

list(APPEND GMOCK_CHECK_SOURCE_DIRS
    /usr/src/gmock
    /usr/src/googletest/googlemock
)

find_path(GMOCK_SOURCE_DIR
    NAMES ${GMOCK_CHECK_SOURCES}
    PATHS ${GMOCK_CHECK_SOURCE_DIRS}
)

if (GMOCK_SOURCE_DIR)
    add_subdirectory(${GMOCK_SOURCE_DIR} "${CMAKE_CURRENT_BINARY_DIR}/gmock")
    set(GMOCK_LIBRARIES gmock_main)
endif()


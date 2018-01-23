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

FROM ubuntu:xenial

RUN apt-get update && apt-get install -y sudo
RUN rm -rf /var/lib/apt/lists/*

COPY scripts/install_dependencies.sh ga_slam/scripts/
RUN ga_slam/scripts/install_dependencies.sh
RUN rm -rf /var/lib/apt/lists/*

COPY . ga_slam
RUN ga_slam/scripts/build_library.sh
RUN rm -rf ga_slam


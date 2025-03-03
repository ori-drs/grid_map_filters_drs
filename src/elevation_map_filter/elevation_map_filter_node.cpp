//----------------------------------------
// This file is part of grid_map_filters_drs
//
// Copyright (C) 2020-2025 Matías Mattamala, University of Oxford.
//
// grid_map_filters_drs is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// grid_map_filters_drs is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with grid_map_filters_drs.
// If not, see <http://www.gnu.org/licenses/>.
//----------------------------------------
/*
 *  Based on
 *  filters_demo_node.cpp
 *  by Peter Fankhauser (ETH Zurich, ANYbotics)
 *
 *  Author: Matias Mattamala
 */

#include <elevation_map_filter/elevation_map_filter.hpp>

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevation_map_filter");
  ros::NodeHandle nodeHandle("~");

  bool success;
  ElevationMapFilter filter(nodeHandle, success);
  if (success) ros::spin();
  return 0;
}
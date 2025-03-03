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
 * CropFilter.cpp
 *
 *  Crops the full grid map into a smaller size
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_filters_drs/CropFilter.hpp>
#include <stdexcept>
#include <string>

using namespace filters;

namespace grid_map {

template <typename T>
CropFilter<T>::CropFilter() {}

template <typename T>
CropFilter<T>::~CropFilter() {}

template <typename T>
bool CropFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("CropFilter");

  // Load Parameters
  // Input layer to be processed
  double x = 0.0;
  if (!FilterBase<T>::getParam(std::string("center_x"), x)) {
    ROS_ERROR("[CropFilter] did not find parameter `x`. Using default %f", x);
  }
  ROS_DEBUG("[CropFilter] x = %f.", x);

  double y = 0.0;
  if (!FilterBase<T>::getParam(std::string("center_y"), y)) {
    ROS_ERROR("[CropFilter] did not find parameter `y`. Using default %f", y);
  }
  ROS_DEBUG("[CropFilter] y = %s.", y);

  // Initialize center
  crop_center_ = grid_map::Position(x, y);

  // Dimensions for the cropping
  double length_x = 0.0;
  if (!FilterBase<T>::getParam(std::string("length_x"), length_x)) {
    ROS_ERROR("[CropFilter] did not find parameter `length_x`.");
    return false;
  }
  ROS_DEBUG("[CropFilter] length_x = %f.", length_x);

  double length_y = 0.0;
  if (!FilterBase<T>::getParam(std::string("length_y"), length_y)) {
    ROS_ERROR("[CropFilter] did not find parameter `length_y`.");
    return false;
  }
  ROS_DEBUG("[CropFilter] length_y = %f.", length_y);

  // Length
  crop_length_ = grid_map::Length(length_x, length_y);

  return true;
}

template <typename T>
bool CropFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  bool success = false;

  // Get crop in local coordinates
  grid_map::Position center = mapIn.getPosition() + crop_center_;

  // Attempt crop
  grid_map::GridMap sub_map = mapIn.getSubmap(center, crop_length_, success);
  
  if (success){
    mapOut = sub_map;
  }
  else{
    mapOut = mapIn;
  }
  
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::CropFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::CropFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
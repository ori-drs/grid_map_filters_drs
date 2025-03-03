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
 * NanMaskFilter.cpp
 *
 *  Generates a new layer with a mask that indicates the NaN values
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_filters_drs/NanMaskFilter.hpp>
#include <stdexcept>
#include <string>

using namespace filters;

namespace grid_map {

template <typename T>
NanMaskFilter<T>::NanMaskFilter() : setTo_("0.0") {}

template <typename T>
NanMaskFilter<T>::~NanMaskFilter() {}

template <typename T>
bool NanMaskFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("NanMaskFilter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("[NanMaskFilter] did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("[NanMaskFilter] input_layer = %s.", inputLayer_.c_str());

  // Output layer to be processed
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("[NanMaskFilter] did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("[NanMaskFilter] output_layer = %s.", outputLayer_.c_str());

  return true;
}

template <typename T>
bool NanMaskFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("[NanMaskFilter] Check your layer type! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Add filtered layer
  mapOut.add(outputLayer_, mapIn[inputLayer_]);

  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (mapOut.isValid(*iterator, inputLayer_))
      mapOut.at(outputLayer_, *iterator) = 0.0;
    else
      mapOut.at(outputLayer_, *iterator) = 1.0;
  }

  mapOut.setBasicLayers({});

  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::NanMaskFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::NanMaskFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)